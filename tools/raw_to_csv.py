"""raw.bin -> DataFrame -> CSV, og videre -> rekonstruert imu.csv.

To utganger, valgt med --mode:

  samples   Rådataene som de er: én rad per accel-sample ved kImuOdrHz, med gyro og
            SFLP paret inn på nærmeste sample. Dette er hva binærfila inneholder,
            skrevet ut flatt. Bruk den til å se på råsignalet.

  imu       Rekonstruksjonen av device-formatet: FIR-desimering til rad-raten, AHRS
            på råstrømmen, vertikal-accel - altså det firmwaren regner ut om bord,
            gjort om igjen offline. Kolonnene er de samme som kImuCsvHeader, så
            postprocess.py og alt bygget på read_imu_rows leser resultatet uendret.

HVORFOR REKONSTRUKSJONEN ER VERDT NOE. Kjør en fangst med WaveLogMode::Both, og
sammenlign denne mot enhetens egen imu.csv fra SAMME opptak: da er FIR-en, AHRS-en og
vertikal-accel-kjeden om bord verifisert mot en uavhengig implementasjon på nøyaktig
de samplene enheten så. --compare gjør nettopp den sammenligningen.

Bit-likhet er ikke ventet: om bord er float32, her float64, og det gir ~1e-6. Avvik
STØRRE enn det er et funn, ikke støy.

AHRS-EN KJØRER PÅ RÅSTRØMMEN HER. Det er forskjellen fra postprocess.py, som bare har
radene og derfor må kjøre filteret på rad-raten - en kjent og dokumentert avvik
(se kalman_nxp.py / postprocess.vacc_series). Med råloggen forsvinner det avviket.

VENTET ADVARSEL. --mode imu skriver "bøtte 10 ms går ikke opp i radperioden 2.08333
ms - desimeringen (D=5) glir mot tidsaksen". Den er RIKTIG og ikke et problem her:
det ekte forholdet er 4.8, og D=5 er fir.py sin avrunding i selve meldingsteksten -
det er nettopp derfor den advarer. FIR trinn 1 er per design ikke en
heltalls desimering (kImuOdrHz/kRowOdrHz = 4.8), og firmwaren håndterer det på samme
måte - tidsdrevet utgangsrutenett, maks en halv råperiode jitter. Se kommentaren ved
kFirS1DecimX10 i wave_config.h. Advarselen betyr noe helt annet i trinn 2, der
delelighet ER et krav og firmwaren asserter på den.

    python3 raw_to_csv.py <fangst>_raw.bin --mode imu -o rekonstruert_imu.csv
    python3 raw_to_csv.py <mappe> --mode imu --compare
"""

import argparse
import math
import os
import sys
from glob import glob

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import postprocess as pp                        # noqa: E402
import rawlog                                   # noqa: E402
import fir                                      # noqa: E402
from madgwick import Madgwick                   # noqa: E402
from kalman import Kalman                       # noqa: E402
from kalman_nxp import KalmanNxp                # noqa: E402
from mekf import Mekf                           # noqa: E402

# Filtrene som kan kjøres på råstrømmen. Rekkefølgen er den samme som metodene
# står i postprocess og i bølgetabellen, så en --ahrs-liste og en tabellkolonne
# alltid leses i samme rekkefølge.
AHRS_NAVN = ("madgwick", "kalman", "nxp", "mekf")

# Enheter og vertikal-accel kommer fra postprocess, ikke fra egne kopier: vacc må
# defineres på nøyaktig én plass, ellers kan rekonstruksjonen og analysen gli fra
# hverandre uten at noe sier fra.
GRAVITY = pp.GRAVITY
MG2MS2 = pp.MG2MS2
MDPS2RADS = pp.MDPS2RADS

# FALLBACKS, ikke fasit. Verdiene under gjelder bare når cfg.csv mangler - altså når
# man kjører på en løs raw.bin. Ellers leses alt fra fangstens egen cfg.csv.
#
# Hvorfor det ikke er pedanteri: Skjaerhalden-fangstene fra juli har
# madgwick_beta = 0.2, mens wave_config.h i dag har 0.05. Hardkodet ville
# rekonstruksjonen av dem kjørt et ANNET filter enn enheten gjorde, og --compare
# ville rapportert et avvik som så ut som en firmware-feil.
#
# NB at pp.MADGWICK_BETA (0.2) heller ikke er riktig kilde: den er postprocess sitt
# eget valg for at metode-kolonnene skal ligne hverandre. Fasit er fangsten.
ROW_ODR_HZ = 100          # kRowOdrHz
MADGWICK_BETA = 0.05      # kMadgwickBeta
BRAKE_G_THRESHOLD = 0.5   # kBrakeGThreshold
BRAKE_MIN_SAMPLES = 3     # kBrakeMinSamples (om bord utledet av kBrakeMinMs * ODR)


class Params:
    """Rekonstruksjonsparametre, lest fra fangstens cfg.csv der de finnes.

    .beskriv() lister hva som faktisk kom fra fila, slik at en kjøring på en løs
    raw.bin ikke stilltiende ser ut som en kjøring med fangstens egne konstanter."""

    def __init__(self, cfg):
        self.fra_cfg, self.mangler = [], []

        def hent(nokkel, default, cast=float):
            try:
                v = cast(cfg[nokkel])
                self.fra_cfg.append(f"{nokkel}={v:g}")
                return v
            except (KeyError, ValueError):
                self.mangler.append(nokkel)
                return default

        self.row_odr_hz = hent("output_rate_hz", ROW_ODR_HZ, lambda s: int(float(s)))
        self.madgwick_beta = hent("madgwick_beta", MADGWICK_BETA)
        self.brake_g = hent("brake_g_thresh", BRAKE_G_THRESHOLD)
        # brake_min_samples er den UTLEDEDE verdien om bord. Å lese den i stedet for
        # å regne brake_min_ms * ODR på nytt fjerner et avrundingsvalg som ellers kan
        # gi ett sample forskjell i debouncen.
        self.brake_min_samples = hent("brake_min_samples", BRAKE_MIN_SAMPLES,
                                      lambda s: int(float(s)))
        self.gravity = hent("gravity", pp.GRAVITY)
        # Hvilket filter fangsten FAKTISK kjørte. --ahrs overstyrer; uten den følger
        # rekonstruksjonen fangsten i stedet for å anta Madgwick.
        self.orientation = cfg.get("orientation_name", "")

    def ahrs_navn(self, override=None):
        """Filternavnet fangsten kjørte, eller det --ahrs ber om.

        cfg.csv kan bare inneholde det firmware skriver: "Madgwick", "Kalman"
        eller "SFLP" (wave_orientation_name -> WaveAhrs::kName). nxp og mekf er
        derfor bare tilgjengelige via --ahrs - de er ALTERNATIVER man ber om, ikke
        noe en fangst kan ha kjørt.

        Selve oppslaget ligger i postprocess.ahrs_fra_cfg, ikke her: analysen
        gjør nøyaktig samme oversettelse, og to kopier av den kan bli uenige om
        hvilket filter en fangst kjørte."""
        if override:
            return override
        return pp.ahrs_fra_cfg(self.orientation)

    def beskriv(self):
        s = "  cfg: " + (", ".join(self.fra_cfg) if self.fra_cfg else "ingen")
        if self.orientation:
            s += f", orientation_name={self.orientation}"
        if self.mangler:
            s += ("\n  MANGLER i cfg.csv, bruker innebygde defaults: "
                  + ", ".join(self.mangler))
        return s


def les_params(binpath):
    return Params(pp.read_kv(binpath.replace("_raw.bin", "_cfg.csv")))


def check_integrity(cap, allow_damaged=False):
    """Stopp før en desynkronisert fil blir til tall noen tror på.

    Dette er en HARD stopp med vilje. En rekonstruksjon fra en fil som har mistet
    bytes går ikke i stykker synlig - den produserer et fullt sett kolonner og et Hs
    med tre desimaler, regnet på payload-bytes lest som tagger. Et selvsikkert galt
    svar er verre enn ingen, så det skal koste et flagg å be om det."""
    if not cap.n_write_fail:
        return
    msg = (f"raaloggen mistet {cap.n_write_fail} blokker og er desynkronisert fra "
           f"accel-sample {cap.first_write_fail_n} av {len(cap.acc)}")
    if not allow_damaged:
        sys.exit(f"AVBRYTER: {msg}.\n"
                 f"  Se ogsaa raw_write_failures i ana.csv. Bruk --allow-damaged "
                 f"hvis du vil se paa den delen som er hel.")
    print(f"  [!] --allow-damaged: {msg}")

IMU_COLUMNS = ("win_start_ms,n,ax_mg,ay_mg,az_mg,ax_ned_sflp,ay_ned_sflp,az_ned_sflp,"
               "gx_mdps,gy_mdps,gz_mdps,qw_sflp,qx_sflp,qy_sflp,qz_sflp,braking,"
               "qw,qx,qy,qz,vacc,vacc_sflp,sflp_nan,fifo_ovf,vacc_fir,"
               "vacc_sflp_fir").split(",")


def resolve(path):
    """Godta en .bin, eller en øktmappe som inneholder én."""
    if os.path.isdir(path):
        cand = sorted(glob(os.path.join(path, "*_raw.bin")))
        if not cand:
            sys.exit(f"fant ingen *_raw.bin i {path}")
        return cand[0]
    if not os.path.isfile(path):
        sys.exit(f"fant ikke {path}")
    return path


def align(cap):
    """Gyro og SFLP inn på accel-tidsaksen.

    Gyroen holdes (siste verdi), ikke interpoleres: det er nøyaktig det firmwaren
    gjør - latestGx_ oppdateres når gyro-ordet kommer, og accel-grenen bruker den
    verdien som står der. Interpolasjon ville gitt et penere, men annet, signal enn
    enheten regnet på. Samme for SFLP-quaternionen."""
    n = len(cap.acc_t)
    gyr = np.zeros((n, 3))
    quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (n, 1))
    if len(cap.gyr_t):
        j = np.searchsorted(cap.gyr_t, cap.acc_t, side="right") - 1
        gyr = cap.gyr[np.clip(j, 0, len(cap.gyr) - 1)]
        gyr[j < 0] = 0.0
    if len(cap.quat_t):
        j = np.searchsorted(cap.quat_t, cap.acc_t, side="right") - 1
        quat = cap.quat[np.clip(j, 0, len(cap.quat) - 1)]
        quat[j < 0] = np.array([1.0, 0.0, 0.0, 0.0])
    return gyr, quat


def samples_frame(cap):
    gyr, quat = align(cap)
    return pd.DataFrame({
        "t_s": cap.acc_t,
        "ax_mg": cap.acc[:, 0], "ay_mg": cap.acc[:, 1], "az_mg": cap.acc[:, 2],
        "gx_mdps": gyr[:, 0], "gy_mdps": gyr[:, 1], "gz_mdps": gyr[:, 2],
        "qw_sflp": quat[:, 0], "qx_sflp": quat[:, 1],
        "qy_sflp": quat[:, 2], "qz_sflp": quat[:, 3],
    })


def world_ned(quat, acc_mg):
    """R(q)·a for hele serien, tyngdekraften trukket fra Z. Vektorisert utgave av
    den innlinede rotasjonen i imu_sampler.cpp."""
    qw, qx, qy, qz = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    ax, ay, az = acc_mg[:, 0], acc_mg[:, 1], acc_mg[:, 2]
    wx = ((1 - 2 * (qy * qy + qz * qz)) * ax + 2 * (qx * qy - qw * qz) * ay
          + 2 * (qx * qz + qw * qy) * az)
    wy = (2 * (qx * qy + qw * qz) * ax + (1 - 2 * (qx * qx + qz * qz)) * ay
          + 2 * (qy * qz - qw * qx) * az)
    wz = (2 * (qx * qz - qw * qy) * ax + 2 * (qy * qz + qw * qx) * ay
          + (1 - 2 * (qx * qx + qy * qy)) * az)
    return wx, wy, wz - 1000.0


def lag_ahrs(navn, par, odr_hz):
    """Filteret som skal kjøre på råstrømmen, i fangstens egne konstanter.

    Alle fire har samme grensesnitt (init_from_accel/update/.q), så run_ahrs ser
    ingen forskjell - men NXP er unntaket ved KONSTRUKSJON: den baker inn 1/fs i
    stedet for å ta dt per steg (se FAST RATE i kalman_nxp.py). Raten her er
    RÅRATEN, ikke radraten: filteret kjører per sample. Feil rate der er ikke en
    mild feil - Hs_nxp på 110314 går fra 0.098 m til 26.8 m ved fjerdedelen av
    riktig rate."""
    if navn == "madgwick":
        return Madgwick(beta=par.madgwick_beta)
    if navn == "kalman":
        return Kalman()
    if navn == "nxp":
        return KalmanNxp(fs=float(odr_hz))
    if navn == "mekf":
        return Mekf()
    sys.exit(f"ukjent AHRS '{navn}' - kjenner {', '.join(AHRS_NAVN)}")


def run_ahrs(t_s, acc_mg, gyr_mdps, filt):
    """AHRS på RÅSTRØMMEN - én update per sample, som imu_sampler.cpp. Returnerer
    quaternionene og vertikal lineær accel (m/s²) per sample."""
    n = len(t_s)
    q = np.empty((n, 4))
    vacc = np.empty(n)
    a_si = acc_mg * MG2MS2
    g_si = gyr_mdps * MDPS2RADS
    for i in range(n):
        if i == 0:
            filt.init_from_accel(*a_si[0])
        else:
            dt = t_s[i] - t_s[i - 1]
            if dt > 0:
                filt.update(*g_si[i], *a_si[i], dt)
        q[i] = filt.q
        vacc[i] = pp.vertical_accel(filt.q, *a_si[i])
    return q, vacc


def braking_flags(wx, wy, wz, brake_g, min_samples):
    """Brems-flagget: |lineær a| over terskel i min_samples påfølgende samples."""
    thr2 = (brake_g * 1000.0) ** 2
    over = (wx * wx + wy * wy + wz * wz) > thr2
    need = max(1, min_samples)
    run = 0
    out = np.zeros(len(over), dtype=bool)
    for i, o in enumerate(over):
        run = run + 1 if o else 0
        out[i] = run >= need
    return out


def imu_frame(cap, par, ahrs=None):
    """Rekonstruer device-formatet fra råsamplene, med fangstens egne konstanter."""
    gyr, quat = align(cap)
    acc = cap.acc
    t_s = cap.acc_t
    odr = float(cap.imu_odr_hz)
    row_odr_hz = par.row_odr_hz
    row_ms = 1000 // row_odr_hz

    wx, wy, wz = world_ned(quat, acc)
    navn = par.ahrs_navn(ahrs)
    q_ahrs, vacc = run_ahrs(t_s, acc, gyr, lag_ahrs(navn, par, odr))
    brake = braking_flags(wx, wy, wz, par.brake_g, par.brake_min_samples)

    # FIR trinn 1: alle ti seriene gjennom samme filter, levert på rad-rutenettet -
    # samme kjede som FirRowBank om bord, og samme koeffisienter (fir.py genererte
    # tabellen i fir_coeffs.h).
    t_ms = t_s * 1000.0
    series = {"ax": acc[:, 0], "ay": acc[:, 1], "az": acc[:, 2],
              "nx": wx, "ny": wy, "nz": wz,
              "gx": gyr[:, 0], "gy": gyr[:, 1], "gz": gyr[:, 2],
              "vacc": vacc}
    win = np.arange(int(t_ms[0]) // row_ms, int(t_ms[-1]) // row_ms + 1)
    dec, gap, _ = fir.fir_decimate_series(t_ms, series, win, row_ms, row_odr_hz,
                                          raw_dt_ms=1000.0 / odr)

    # Usifiltrerte verdier ved samme instans (senter-tappen om bord). Nærmeste
    # råsample til bøttesenteret er den samme konvensjonen som kFirS1CenterMs.
    centre_t = (win * row_ms + row_ms / 2.0) / 1000.0
    ci = np.clip(np.searchsorted(t_s, centre_t), 0, len(t_s) - 1)

    out = pd.DataFrame({
        "win_start_ms": win * row_ms,
        "n": np.bincount(np.clip((t_ms // row_ms).astype(int) - win[0], 0, len(win) - 1),
                         minlength=len(win)),
        "ax_mg": dec["ax"], "ay_mg": dec["ay"], "az_mg": dec["az"],
        "ax_ned_sflp": dec["nx"], "ay_ned_sflp": dec["ny"], "az_ned_sflp": dec["nz"],
        "gx_mdps": dec["gx"], "gy_mdps": dec["gy"], "gz_mdps": dec["gz"],
        "qw_sflp": quat[ci, 0], "qx_sflp": quat[ci, 1],
        "qy_sflp": quat[ci, 2], "qz_sflp": quat[ci, 3],
        "braking": brake[ci].astype(int),
        "qw": q_ahrs[ci, 0], "qx": q_ahrs[ci, 1],
        "qy": q_ahrs[ci, 2], "qz": q_ahrs[ci, 3],
        "vacc": vacc[ci],
        "vacc_sflp": wz[ci] * MG2MS2,
        "sflp_nan": 0,
        "fifo_ovf": 0,
        "vacc_fir": dec["vacc"],
        "vacc_sflp_fir": dec["nz"] * MG2MS2,
    })
    # fifo_ovf fra sync-postene. Denne kolonnen kan ikke utledes fra samplene - den
    # kommer fra sensorens statusregister - så råloggen er eneste kilde.
    #
    # HVILKEN rad som skal flagges er ikke rada ved sync-tidspunktet. Om bord settes
    # winFifoOvf_ ved starten av dreneringen og gjelder vinduet som ALT ER ÅPENT, altså
    # vinduet til forrige drenerings SISTE sample; det emitteres av closeWindow() før
    # flagget nullstilles. Sync-postens t_us er derimot tida til dreneringens FØRSTE
    # sample. De to er ulike hver gang en drenering starter på en vindusgrense - ved
    # 480 Hz og 10 ms vinduer er det ~1 av 4.8 dreneringer. Derfor indekseres det på
    # accel_n - 1 og ikke på t_us.
    for s in cap.sync:
        if s.get("fifo_ovf", bool(s["flags"] & rawlog.FLAG_FIFO_OVF)):
            i = max(0, int(s["accel_n"]) - 1)          # accel_n == 0: første vindu
            if i < len(t_ms):
                w = int(t_ms[i] // row_ms) - win[0]
                if 0 <= w < len(out):
                    out.loc[w, "fifo_ovf"] = 1
    return out.reindex(columns=IMU_COLUMNS)


def compare(recon, device_csv):
    """Rekonstruksjon mot enhetens egen imu.csv, kolonne for kolonne."""
    dev = pd.read_csv(device_csv)
    key = "win_start_ms"
    m = recon.merge(dev, on=key, suffixes=("_r", "_d"))
    if m.empty:
        return "  ingen felles win_start_ms - er det samme opptak?"
    lines = [f"  {len(m)} felles rader av {len(recon)} rekonstruerte / {len(dev)} device",
             f"  {'kolonne':16s} {'maks |avvik|':>12s} {'rms':>12s} {'device rms':>12s}"]
    for c in IMU_COLUMNS:
        if c == key or f"{c}_r" not in m:
            continue
        a, b = m[f"{c}_r"].to_numpy(float), m[f"{c}_d"].to_numpy(float)
        d = np.abs(a - b)
        lines.append(f"  {c:16s} {d.max():12.3e} {np.sqrt((d**2).mean()):12.3e} "
                     f"{np.sqrt((b**2).mean()):12.3e}")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="<stamp>_raw.bin eller øktmappa")
    ap.add_argument("--mode", choices=["samples", "imu"], default="samples")
    ap.add_argument("--ahrs", choices=list(AHRS_NAVN), default=None,
                    help="overstyr filteret; uten den følges cfg.csv "
                         "orientation_name. nxp og mekf finnes bare her - de er "
                         "alternativer man ber om, ikke noe en fangst kan ha kjørt")
    ap.add_argument("-o", "--out", help="skriv CSV hit (default: ved siden av .bin)")
    ap.add_argument("--compare", action="store_true",
                    help="--mode imu: sammenlign mot enhetens egen imu.csv")
    ap.add_argument("--allow-damaged", action="store_true",
                    help="fortsett selv om raaloggen mistet blokker (se kRawFlagWriteFail). "
                         "Alt etter foerste tap er feiltolket")
    args = ap.parse_args()

    binpath = resolve(args.path)
    cap = rawlog.read(binpath)
    print(rawlog.summary(cap))
    check_integrity(cap, args.allow_damaged)

    par = les_params(binpath)
    if args.mode == "imu":
        print(par.beskriv())
        print(f"  AHRS: {par.ahrs_navn(args.ahrs)}"
              f"{' (--ahrs)' if args.ahrs else ' (fra cfg.csv)'}")
    df = (samples_frame(cap) if args.mode == "samples"
          else imu_frame(cap, par, args.ahrs))
    out = args.out or binpath.replace("_raw.bin", f"_{args.mode}_python.csv")
    df.to_csv(out, index=False, float_format="%.5f")
    print(f"\nskrev {out}  ({len(df)} rader, {len(df.columns)} kolonner)")

    if args.compare and args.mode == "imu":
        dev = binpath.replace("_raw.bin", "_imu.csv")
        if not os.path.isfile(dev):
            print(f"  --compare: fant ikke {dev} (krever WaveLogMode::Both)")
        else:
            print(f"\nmot {os.path.basename(dev)}:")
            print(compare(df, dev))


if __name__ == "__main__":
    main()
