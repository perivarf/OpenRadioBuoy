"""MEKF-kolonnen: kalman.py kjørt med DRIFTERENS parametre.

Fra 2026-08-03 er firmwarens KalmanAhrs (kalman.cpp) en direkte port av kalman.py
- adaptiv R, rate-invarians, Joseph-form, eksakt quat_exp, samme P0. Da skal ikke
denne fila re-implementere algoritmen; den skal bare velge parametersettet. Det
som står igjen som forskjell mellom "Kalman"- og "MEKF"-kolonnen er derfor
NØYAKTIG det som står i wave_config.h, og ingenting annet.

Så lenge konstantene under er like kalman.py-defaultene, er de to kolonnene
identiske - og det er meningen: da er avvik mellom dem beviset på at firmware og
offline har drevet fra hverandre. Endres kKalmanParams i wave_config.h (typisk
under tuning), speiles endringen HER, og MEKF-kolonnen viser umiddelbart hva
drifteren ville regnet ut mens Kalman-kolonnen står igjen som referansetuningen.

    f = Mekf()
    f.init_from_accel(ax, ay, az)
    f.update(gx, gy, gz, ax, ay, az, dt)   # gyro rad/s, ACCEL M/S² - se under
    f.q                                    # [w, x, y, z], body -> verden

ACCEL-ENHETEN ER IKKE LIKEGYLDIG, i motsetning til Madgwick: R-en veier |a| mot
g, så mg ville sett ut som 100 g slag i hvert eneste sample.

To ting speiles ikke:
  - float32 om bord mot float64 her. Målt på en syntetisk 20 s-sekvens er
    forskjellen ~1e-6 i quaternionen (scratchpad: kalman_ref.cpp/.py).
  - P0. kalman.py holder P0_ANGLE/P0_BIAS som modulkonstanter, ikke ctor-args,
    så en endring av p0Angle/p0Bias i wave_config.h må gjøres i kalman.py for å
    slå gjennom her. De to er like i dag (5 deg / 1 deg/s).
Chi²-gaten i kalman.py er av (gate = 0) og finnes ikke i firmware i det hele
tatt, så den settes eksplisitt av her.
"""

from kalman import Kalman

# --- Speiler kKalmanParams i wave_config.h -----------------------------------
SIGMA_G = 0.005              # rad/s/sqrt(Hz)
SIGMA_B = 1.0e-5             # rad/s²/sqrt(Hz)
R0 = 1.0e-3                  # var 1e-5 fram til 2026-08-04 - se sveipet i wave_config.h
DT_REF = 0.020               # s
LAMBDA_A = 0.0
LAMBDA_W = 2.0
W0 = 1.0                     # rad/s
GRAVITY = 9.80665            # kGravity


class Mekf(Kalman):
    """kalman.py-filteret med firmwarens tuning. Arver, ikke kopierer: skulle de
    to implementasjonene noen gang skille lag, er det en feil - ikke et valg."""

    def __init__(self):
        super().__init__(sigma_g=SIGMA_G, sigma_b=SIGMA_B, r0=R0,
                         lambda_a=LAMBDA_A, lambda_w=LAMBDA_W, w0=W0,
                         gate=0.0, gravity=GRAVITY, dt_ref=DT_REF)
