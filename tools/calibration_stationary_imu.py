#!/usr/bin/env python3
"""Gyro- og akselerometer-nullpunkt fra en raalogg der bøya lå stille.

Leser <stamp>_raw.bin, skalerer med sensitivitetene i headeren og skriver ut
bias, støy og drift per akse. Ingenting lagres - dette er en måling du gjør før
et tokt, ikke et steg i analysekjeden.

HVA SOM FAKTISK MÅLES. En bøye i ro ser bare to ting: gyroens nullpunkt (den
skal lese 0, alt annet er bias) og tyngdekraften (|a| skal lese lokal g). Begge
er per eksemplar og temperaturavhengige, så tallene gjelder denne enheten ved
denne temperaturen.

HVA SOM IKKE KAN MÅLES SLIK. For akselerometeret gir én orientering ett tall,
|a|, og både en akse-bias b og en skalafeil s forklarer det like godt - de er
ikke separerbare uten å snu enheten. Skriptet rapporterer derfor avviket i |a|
og lar være å foreslå en korreksjonsfaktor per akse. Skal du kalibrere på
ordentlig, trengs 6-posisjons tumble (±x, ±y, ±z opp) eller ellipsoidetilpasning.

RO ER IKKE NOE SKRIPTET ANTAR. Terskelen settes fra medianen og MAD, ikke fra
snittet, så en dytt midt i opptaket ikke får definere sitt eget aksept-vindu.
Forstyrrede perioder skrives ut med tidspunkt - er det mange, lå ikke enheten
stille, og tallene under er ikke bias.

    python3 calibration_stationary_imu.py <mappe|_raw.bin>
    python3 calibration_stationary_imu.py <mappe> --latitude 59.1
    python3 calibration_stationary_imu.py <mappe> --gravity 9.8190
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rawlog                                    # noqa: E402
import raw_to_csv                                # noqa: E402

G_STANDARD = 9.80665          # kGravity i wave_config.h
AXES = ("x", "y", "z")


def local_gravity(lat_deg, alt_m=0.0):
    """Internasjonal tyngdeformel (1967) + fritt-luft-korreksjon.

    Hvorfor dette er verdt å ta med: på 59 grader nord er g 9.8184, altså 0.13 %
    over standardverdien firmwaren regner med. Uten korreksjonen tilskrives den
    forskjellen sensoren.
    """
    s2 = math.sin(math.radians(lat_deg)) ** 2
    s2_2 = math.sin(math.radians(2.0 * lat_deg)) ** 2
    g = 9.780327 * (1.0 + 0.0053024 * s2 - 0.0000058 * s2_2)
    return g - 3.086e-6 * alt_m


def quiet_mask(v, k):
    """Samples innenfor k*MAD av medianen, alle akser samtidig.

    MAD*1.4826 er std for normalfordelt støy, men i motsetning til std lar den
    seg ikke dra av utliggerne den skal finne."""
    med = np.median(v, axis=0)
    mad = np.median(np.abs(v - med), axis=0) * 1.4826
    mad = np.where(mad > 0.0, mad, np.finfo(float).eps)
    return np.all(np.abs(v - med) < k * mad, axis=1)


def report_disturbances(t, mask, label, gap_s=1.0, min_samples=5):
    bad = t[~mask]
    if not len(bad):
        print(f"  {label}: ingen forstyrrelser")
        return
    pct = 100.0 * len(bad) / len(t)
    print(f"  {label}: {len(bad)} samples forkastet ({pct:.3f} %)")
    bad = np.sort(bad)
    for seg in np.split(bad, np.where(np.diff(bad) > gap_s)[0] + 1):
        if len(seg) >= min_samples:
            print(f"      t = {seg[0]:8.1f} - {seg[-1]:8.1f} s   ({len(seg)} samples)")


def drift_per_hour(t, v):
    """Stigningstall fra lineær tilpasning, i enhet/time."""
    return np.array([np.polyfit(t, v[:, i], 1)[0] * 3600.0 for i in range(v.shape[1])])


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="øktmappe med <stamp>_raw.bin, eller .bin-fila selv")
    ap.add_argument("--latitude", type=float, default=None,
                    help="breddegrad for lokal g. Uten den brukes standard g, "
                         "og et 0.1-0.2 %% avvik tilskrives sensoren")
    ap.add_argument("--altitude", type=float, default=0.0,
                    help="meter over havet, til fritt-luft-korreksjonen (default 0)")
    ap.add_argument("--gravity", type=float, default=None,
                    help="oppgi lokal g direkte i m/s^2; overstyrer --latitude")
    ap.add_argument("--sigma", type=float, default=8.0,
                    help="hvor mange MAD-sigma som regnes som ro (default 8)")
    args = ap.parse_args()

    binpath = raw_to_csv.resolve(args.path)
    cap = rawlog.read(binpath)

    print(f"=== {os.path.basename(binpath)} ===")
    print(f"format v{cap.version}, reading {cap.reading_id}")
    print(f"acc_sens {cap.acc_sens_mg} mg/LSB, gyr_sens {cap.gyr_sens_mdps} mdps/LSB")

    # Kutt ved første tapte blokk: alt etter den kan være feiltolket, og en
    # feiltolket byte blir til et helt vanlig utseende sample.
    n_ok = cap.trustworthy_upto()
    acc, acc_t = cap.acc[:n_ok], cap.acc_t[:n_ok]
    if n_ok < len(cap.acc):
        t_cut = acc_t[-1]
        keep = cap.gyr_t <= t_cut
        gyr, gyr_t = cap.gyr[keep], cap.gyr_t[keep]
        print(f"ADVARSEL: raaloggen mistet blokker - kuttet ved t = {t_cut:.1f} s")
    else:
        gyr, gyr_t = cap.gyr, cap.gyr_t

    print(f"{len(acc)} accel- og {len(gyr)} gyro-samples over "
          f"{acc_t[-1] - acc_t[0]:.0f} s")
    if cap.n_overflow:
        print(f"MERK: {cap.n_overflow} dreneringer med FIFO-overflow - "
              f"tidsaksen er komprimert der, men bias er upåvirket")

    print("\n--- utvalg av rolige samples ---")
    # Gyro testes per akse: i ro skal alle tre lese det samme, uansett positur.
    gq_mask = quiet_mask(gyr, args.sigma)
    # Accel testes paa |a|, ikke per akse. En enhet som star stille kan fortsatt
    # SKIFTE POSITUR - underlaget setter seg, festet gir etter - og da flytter
    # aksene seg mye mens |a| ikke rikker seg. Per-akse-test ville kastet alt etter
    # en slik dreining som "forstyrret", og stille regnet bias paa bare det som var
    # foer. |a| er rotasjonsinvariant og treffer det testen faktisk er ute etter.
    aq_mask = quiet_mask(np.linalg.norm(acc, axis=1).reshape(-1, 1), args.sigma)
    report_disturbances(gyr_t, gq_mask, "gyro")
    report_disturbances(acc_t, aq_mask, "accel (paa |a|)")

    gq, gqt = gyr[gq_mask], gyr_t[gq_mask]
    aq, aqt = acc[aq_mask], acc_t[aq_mask]
    if len(gq) < 100 or len(aq) < 100:
        sys.exit("for få rolige samples - lå enheten i det hele tatt stille?")

    gdrift = drift_per_hour(gqt, gq)
    print(f"\n--- GYRO ({len(gq)} samples) ---")
    print(f"{'':4}{'bias [mdps]':>14}{'bias [dps]':>13}{'stoy [mdps]':>14}"
          f"{'drift [mdps/t]':>17}")
    for i, ax in enumerate(AXES):
        c = gq[:, i]
        print(f"g{ax:<3}{c.mean():14.2f}{c.mean() / 1000.0:13.5f}"
              f"{c.std():14.2f}{gdrift[i]:+17.2f}")
    bias = gq.mean(axis=0)
    print(f"|bias| = {np.linalg.norm(bias):.1f} mdps = "
          f"{np.linalg.norm(bias) / 1000.0:.4f} dps")

    adrift = drift_per_hour(aqt, aq)
    mag = np.linalg.norm(aq, axis=1)
    print(f"\n--- AKSELEROMETER ({len(aq)} samples) ---")
    print(f"{'':4}{'snitt [mg]':>14}{'stoy [mg]':>12}{'drift [mg/t]':>15}")
    for i, ax in enumerate(AXES):
        c = aq[:, i]
        print(f"a{ax:<3}{c.mean():14.2f}{c.std():12.3f}{adrift[i]:+15.3f}")
    print(f"{'|a|':<4}{mag.mean():14.2f}{mag.std():12.3f}"
          f"{np.polyfit(aqt, mag, 1)[0] * 3600.0:+15.3f}")

    # Skiftet posituren underveis? Sammenlign retningen paa g i foerste og siste
    # tidel. Bare |a| er meningsfull paa tvers av en dreining - akse-snittene over
    # blander da to orienteringer og skal ikke leses som bias.
    u = aq / mag[:, None]
    n = max(1, len(u) // 10)
    u0, u1 = u[:n].mean(axis=0), u[-n:].mean(axis=0)
    u0 /= np.linalg.norm(u0)
    u1 /= np.linalg.norm(u1)
    tilt = math.degrees(math.acos(float(np.clip(np.dot(u0, u1), -1.0, 1.0))))
    print(f"  positurendring foerste -> siste tidel: {tilt:.2f} grader")
    if tilt > 0.5:
        print("  ADVARSEL: enheten dreide seg under opptaket. |a| og "
              "tyngdekraft-sjekken\n"
              "  under er fortsatt gyldige, men akse-snittene over blander to "
              "orienteringer.")

    if args.gravity is not None:
        g_ref, how = args.gravity, "oppgitt"
    elif args.latitude is not None:
        g_ref = local_gravity(args.latitude, args.altitude)
        how = f"{args.latitude:.4g} grader nord, {args.altitude:.0f} moh"
    else:
        g_ref, how = G_STANDARD, "standard - oppgi --latitude for lokal g"

    exp_mg = 1000.0 * g_ref / G_STANDARD
    dev = mag.mean() - exp_mg
    print(f"\n--- MOT TYNGDEKRAFTEN ---")
    print(f"referanse g = {g_ref:.5f} m/s^2  ({how})")
    print(f"forventet |a| = {exp_mg:.2f} mg, malt {mag.mean():.2f} mg")
    print(f"avvik {dev:+.2f} mg = {100.0 * dev / exp_mg:+.3f} %")
    print(f"DC-rest hvis firmwaren trekker fra 1000 mg: "
          f"{mag.mean() - 1000.0:+.2f} mg = "
          f"{(mag.mean() - 1000.0) * G_STANDARD / 1000.0:+.4f} m/s^2")
    print("  Avviket er skalafeil OG akse-bias blandet sammen - én orientering "
          "kan ikke skille dem.\n  Bruk det som en helsesjekk, ikke som en "
          "korreksjonsfaktor.")


if __name__ == "__main__":
    main()
