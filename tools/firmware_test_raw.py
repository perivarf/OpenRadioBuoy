#!/usr/bin/env python3
"""firmware_test.py, men matet fra _raw.bin i stedet for imu.csv.

HVORFOR DENNE FINNES. firmware_test.py sier det selv i toppen sin: den kan replaye
alt NEDENFOR radgrensen, men ikke AHRS-en paa raastroemmen, ikke trinn-1-FIR-en og
ikke vindusdriften, fordi "de krever raasamples som ikke finnes i fila". Raaloggen
inneholder nettopp de samplene. Denne fila lukker gapet ved aa sette de to
eksisterende verktoeyene etter hverandre:

    _raw.bin -> raw_to_csv.imu_frame()  -> rekonstruert imu.csv   (AHRS paa 480 Hz)
             -> firmware_test.run()     -> Hs/Tz/Tc/Tp + spektrum (firmwarekonstanter)

Ingen matematikk bor her. Rekonstruksjonen er raw_to_csv sin, analysekjeden er
firmware_test sin; blir det to versjoner av vertikal-accel eller av Welch-kjeden,
er hele poenget borte.

HVA SOM ER NYTT AA MAALE. Med --compare koeres BEGGE gjennom NOEYAKTIG samme
analysekjede: enhetens egen imu.csv og rekonstruksjonen. Begge har hatt AHRS-en paa
raastroemmen - enheten gjorde det om bord, rekonstruksjonen gjoer det her - saa
differansen isolerer firmwarens float32-kjede mot float64 paa de SAMME samplene.
Relativt avvik over ~1e-4 er et funn, ikke avrunding.

    Kjoerer man i stedet --ahrs madgwick, koerer firmware_test filteret paa RADRATEN
    (det er alt en CSV gir den). Differansen mot default --ahrs device er da direkte
    prisen paa aa desimere foer orienteringsfilteret - tallet som ikke fantes foer.

VALG AV --ahrs. Default er 'device', som her IKKE betyr enheten: det betyr
vacc_fir-kolonnen i den rekonstruerte fila, altsaa AHRS-en koert paa raastroemmen.
Det er den kolonnen hele oevelsen handler om, saa den er default.

    python3 firmware_test_raw.py <mappe|_raw.bin>
    python3 firmware_test_raw.py <mappe> --compare --match-device
    python3 firmware_test_raw.py <mappe> --ahrs madgwick     # AHRS paa radraten
"""

import argparse
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import postprocess as pp                        # noqa: E402
import rawlog                                   # noqa: E402
import raw_to_csv                               # noqa: E402
import firmware_test as ft                      # noqa: E402


def rebuild(binpath, workdir, ahrs_override, reuse=False, allow_damaged=False):
    """Rekonstruer <stamp>_imu.csv fra raaloggen og legg den i en komplett
    oektmappe: cfg/ana kopieres med, saa resultatet er en gyldig inngang til
    firmware_test, postprocess og alt annet som tar en opptaksmappe."""
    stamp = os.path.basename(binpath)[:-len("_raw.bin")]
    src = os.path.dirname(os.path.abspath(binpath))
    os.makedirs(workdir, exist_ok=True)
    imu_out = os.path.join(workdir, f"{stamp}_imu.csv")

    # cfg foerst: den styrer BAADE rekonstruksjonen (beta, radrate, brems) og
    # firmware_test sin --match-device. Uten den blir begge gjetning.
    cfg_src = os.path.join(src, f"{stamp}_cfg.csv")
    if os.path.isfile(cfg_src):
        shutil.copy2(cfg_src, os.path.join(workdir, f"{stamp}_cfg.csv"))
    else:
        print(f"  [!] fant ikke {stamp}_cfg.csv - rekonstruksjonen bruker "
              f"innebygde defaults, og --match-device har ingenting aa adoptere")
    ana_src = os.path.join(src, f"{stamp}_ana.csv")
    if os.path.isfile(ana_src):
        shutil.copy2(ana_src, os.path.join(workdir, f"{stamp}_ana.csv"))

    if reuse and os.path.isfile(imu_out):
        print(f"  --reuse: bruker eksisterende {imu_out}")
        return stamp, imu_out

    cap = rawlog.read(binpath)
    print(rawlog.summary(cap))
    # Foer noe regnes: en desynkronisert raalogg gir et Hs som ser helt normalt ut.
    raw_to_csv.check_integrity(cap, allow_damaged)
    par = raw_to_csv.les_params(binpath)
    print(par.beskriv())
    navn = par.ahrs_navn(ahrs_override)
    print(f"  rekonstruksjons-AHRS: {navn}"
          f"{' (--recon-ahrs)' if ahrs_override else ' (fra cfg.csv)'}"
          f"   paa raastroemmen, {cap.imu_odr_hz} Hz")

    df = raw_to_csv.imu_frame(cap, par, ahrs_override)
    df.to_csv(imu_out, index=False, float_format="%.5f")
    print(f"  skrev {imu_out}  ({len(df)} rader)")
    return stamp, imu_out


def delta(label, a, b, names=("Hs", "Tz", "Tc", "Tp")):
    print(f"\n  {label}")
    print(f"    {'':<4}{'device imu.csv':>16}{'fra raw.bin':>16}{'avvik':>12}")
    for k in names:
        x, y = a.get(k, float("nan")), b.get(k, float("nan"))
        d = 100.0 * (y / x - 1.0) if x else float("nan")
        print(f"    {k:<4}{x:>16.3f}{y:>16.3f}{d:>11.2f} %")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="oektmappe med <stamp>_raw.bin, eller .bin-fila selv")
    ap.add_argument("--ahrs", default="device",
                    choices=["device", "madgwick", "kalman", "mekf", "sflp"],
                    help="hvilken vacc analysekjeden mates med. 'device' (default) = "
                         "vacc_fir fra rekonstruksjonen, altsaa AHRS paa raastroemmen. "
                         "De andre lar firmware_test koere filteret paa RADRATEN")
    ap.add_argument("--recon-ahrs", default=None, choices=["madgwick", "kalman"],
                    help="overstyr filteret i selve rekonstruksjonen; uten den "
                         "foelges cfg.csv orientation_name")
    ap.add_argument("--compare", action="store_true",
                    help="koer ogsaa enhetens EGEN imu.csv gjennom samme kjede og "
                         "still resultatene mot hverandre (krever WaveLogMode::Both)")
    ap.add_argument("--workdir", default=None,
                    help="hvor den rekonstruerte oekta legges "
                         "(default: <mappe>/<stamp>_recon)")
    ap.add_argument("--reuse", action="store_true",
                    help="hopp over rekonstruksjonen hvis CSV-en alt finnes "
                         "(AHRS-loekka er den dyre delen)")
    ap.add_argument("--allow-damaged", action="store_true",
                    help="fortsett selv om raaloggen mistet blokker under skriving")
    # Videresendes uendret til firmware_test.
    ap.add_argument("--rates", type=float, nargs="+", default=None)
    ap.add_argument("--seglen", type=int, default=None)
    ap.add_argument("--ntap", type=int, default=ft.FIR_NTAP)
    ap.add_argument("--detrend", default=ft.DETREND,
                    choices=["none", "mean", "linear"])
    ap.add_argument("--decimate", default="fir", choices=["fir", "mean"])
    ap.add_argument("--match-device", action="store_true")
    args = ap.parse_args()

    if args.rates is not None and len(args.rates) < 2:
        sys.exit("--rates trenger minst inn- og utrate, f.eks. '100 10'")

    binpath = raw_to_csv.resolve(args.path)
    srcdir = os.path.dirname(os.path.abspath(binpath))
    stamp = os.path.basename(binpath)[:-len("_raw.bin")]
    workdir = args.workdir or os.path.join(srcdir, f"{stamp}_recon")

    print(f"=== rekonstruerer {os.path.basename(binpath)} ===")
    rebuild(binpath, workdir, args.recon_ahrs, reuse=args.reuse,
            allow_damaged=args.allow_damaged)

    kw = dict(match_device=args.match_device, detrend=args.detrend,
              decimate=args.decimate)
    p_raw = ft.run(workdir, args.ahrs, args.rates, args.seglen, args.ntap, **kw)

    if not args.compare:
        return
    dev_imu = os.path.join(srcdir, f"{stamp}_imu.csv")
    if not os.path.isfile(dev_imu):
        print(f"\n--compare: fant ikke {os.path.basename(dev_imu)} "
              f"(krever at fangsten ble tatt med WaveLogMode::Both)")
        return
    print(f"\n=== samme kjede paa enhetens egen imu.csv ===")
    p_dev = ft.run(srcdir, args.ahrs, args.rates, args.seglen, args.ntap, **kw)
    delta(f"BOELGEPARAMETRE, AHRS={args.ahrs}", p_dev, p_raw)
    print("\n  Begge er koert med identiske analysekonstanter, saa forskjellen er")
    print("  rekonstruksjonen alene: float32 om bord mot float64 her, paa de samme")
    print("  samplene. Relativt avvik over ~1e-4 er et funn, ikke avrunding.")


if __name__ == "__main__":
    main()
