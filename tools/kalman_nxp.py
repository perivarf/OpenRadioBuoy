"""NXP Sensor Fusion 9DOF Kalman - det samme filteret SFY-bøya kjører.

SFY implementerer ikke noe eget filter: sfy4-buoy/src/waves/buf.rs kaller
`NxpFusion` fra ahrs-fusion-craten, som igjen er bindinger mot NXP-ens
SensorFusion-bibliotek (12 tilstander: attitude-feil θ, gyro-bias b, lineær
akselerasjon a og magnetisk forstyrrelse d - 3 hver). Denne modulen wrapper den
SAMME kompilerte koden via python-modulen `ahrs_fusion`, i stedet for å skrive
1505 linjer C++ om til Python. Det er derfor ikke en tilnærming til SFY-filteret,
men filteret selv.

Grensesnittet er delt med madgwick.py og kalman.py, så postprocess-løkka kan
behandle alle tre likt:

    f = KalmanNxp(fs=50.0)
    f.init_from_accel(ax, ay, az)          # ett sample -> startorientering
    f.update(gx, gy, gz, ax, ay, az, dt)   # gyro i rad/s, accel i m/s²
    f.q                                    # [w, x, y, z], body -> verden

ENHETER. Utad tar modulen rad/s og m/s², som de to andre filtrene, slik at
kalleren slipper å vite hvilket filter den snakker med. Innad vil NXP-koden ha
grader/s og g, så konverteringen skjer her. Det speiler buf.rs, som mater
`g_dps` og `a_g`. Magnetometeret sendes som null - SFY gjør det samme, med
begrunnelsen at et ukalibrert magnetometer gjør mer skade enn nytte.

KONVENSJON. NXP-ens qPl er [w, x, y, z] og roterer body -> verden, altså samme
konvensjon som resten av prosjektet (se rotation.py). Verifisert numerisk mot
statiske tilt: world_z(q, a_body) gir +g til 1e-4 for roll/pitch opp til 30°,
mens den konjugerte gir feil svar. rotation.world_z kan derfor brukes rett på
.q, uten konjugering. Det tilsvarer `q.rotate(axl)` i buf.rs.

FAST RATE - VIKTIGSTE FORBEHOLD. `NxpFusion::new(freq)` baker inn dt = 1/freq ved
konstruksjon, og `update()` tar ikke dt. Raten er derfor ikke en innstilling man
kan la stå omtrentlig: dt er tidsbasisen gyroen integreres med, så feil rate
skalerer hvert rotasjonsinkrement feil, attityden driver, og ω⁻⁴ nedstrøms
forstørrer driften til noe ufysisk.

MÅLT på Skjærhalden, samme rådata, kun konstruert fs variert (Hs_nxp i m):

    fs [Hz]        25      50     100     200
    110303 (50)  2.147   0.177   0.640   1.054
    110314 (100) 26.85   2.023   0.098   0.653

Riktig svar er minimumet, og det treffer nøyaktig raten fra cfg.csv i begge
øktene - som samtidig er en uavhengig bekreftelse på at rate-utledningen i
postprocess.run er riktig. Alt annet enn den ekte raten gir søppel, ikke bare en
annen båndbredde.

Selv MED riktig rate står et svakere forbehold igjen: båndbredden avhenger av
raten, så 50 Hz-øktene (window_ms=20) og 100 Hz-øktene (window_ms=10) kjører
filteret med to ulike båndbredder. Forskjeller MELLOM de to gruppene er dermed
dels filter og dels sjøtilstand. Madgwick har samme svakhet; kalman.py er den
eneste av de fire som er immun, fordi den skalerer R med 1/dt nettopp for å bli
rate-invariant (se RATE INVARIANCE der).

dt-argumentet i update() blir følgelig ignorert (utover at dt <= 0 hopper over
steget, som hos de andre). Filteret antar uniform tidsakse. Der radene glir mot
tidsaksen - f.eks. når bøttelengden ikke går opp i radperioden, se advarselen i
fir.py - er den antakelsen brutt for dette filteret på en måte den ikke er for
kalman.py.

KONVERGENS. Filteret låser orienteringen fra første accel-sample selv (NXP-ens
"first orientation lock"), så init_from_accel trenger ikke å sette quaternionen -
og kan det heller ikke: set_quaternion overskrives av den låsingen, målt til
bit-identisk resultat med og uten. Restransienten i de indre tilstandene er målt
til ~1.8 s ved 50 Hz før feilen er under 1 mm/s². Delt på ω⁴ nedstrøms ville en
slik transient blitt en falsk lavfrekvenstopp, så bruk --skip-start på minst et
par sekunder når denne metoden er med.
"""

import math

GRAVITY = 9.80665                     # kGravity [m/s²] - samme som postprocess
RADS2DPS = 180.0 / math.pi            # rad/s -> grader/s (NXP-ens gyro-enhet)


def available():
    """Er python-modulen ahrs_fusion installert? Brukes til å gi én tydelig
    feilmelding i stedet for en ImportError midt i en analysekjøring."""
    try:
        import ahrs_fusion                        # noqa: F401
        return True
    except ImportError:
        return False


class KalmanNxp:
    """NXP 12-tilstands Kalman. All tilstand ligger i den kompilerte modulen."""

    def __init__(self, fs, gravity=GRAVITY):
        # Importen er lazy og ikke på modulnivå: postprocess.py importeres også
        # av verktøy som aldri kjører denne metoden (selfnoise.py, plotteskript),
        # og de skal ikke knekke fordi en valgfri binærmodul mangler.
        try:
            from ahrs_fusion import NxpFusion
        except ImportError as e:
            raise ImportError(
                "kalman_nxp krever python-modulen 'ahrs_fusion' (samme kode som "
                "SFY kjører). Bygg den fra ahrs-fusion-repoet med "
                "'maturin develop --features python'."
            ) from e
        if not (fs > 0.0):
            raise ValueError(f"kalman_nxp: fs må være > 0, fikk {fs}")
        self.fs = float(fs)
        self.gravity = float(gravity)
        # Raten er en KONSTRUKSJONSPARAMETER her, ikke et argument til update() -
        # se FAST RATE i moduldokumentasjonen.
        self._f = NxpFusion.new(self.fs)

    @property
    def q(self):
        """Orientering som [w, x, y, z], body -> verden."""
        return list(self._f.quaternion)

    def _step(self, gx, gy, gz, ax, ay, az):
        """Ett filtersteg i NXP-ens egne enheter. Samlet her fordi både
        init_from_accel og update må konvertere likt."""
        self._f.update(gx * RADS2DPS, gy * RADS2DPS, gz * RADS2DPS,
                       ax / self.gravity, ay / self.gravity, az / self.gravity,
                       0.0, 0.0, 0.0)          # magnetometer av, som i buf.rs

    def init_from_accel(self, ax, ay, az):
        """Start orienteringen fra første accel-sample.

        Til forskjell fra madgwick.py og kalman.py settes ikke quaternionen
        direkte: NXP-koden gjør sin egen 6DOF-låsing fra accel ved første kall og
        overskriver enhver quaternion vi måtte ha satt. Vi kjører derfor ett steg
        med null gyro i stedet, som utløser nettopp den låsingen. Uten dette ville
        filteret stått på identitet gjennom den første raden."""
        self._step(0.0, 0.0, 0.0, ax, ay, az)

    def update(self, gx, gy, gz, ax, ay, az, dt):
        """Ett filtersteg. dt brukes KUN til å hoppe over ikke-monotone rader -
        selve tidssteget er låst til 1/fs inne i NXP-koden (se FAST RATE)."""
        if dt <= 0.0:
            return self.q
        self._step(gx, gy, gz, ax, ay, az)
        return self.q
