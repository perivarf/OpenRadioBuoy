"""Hele kjeden fra en logget økt til kartfigur, i én kommando.

    raw.bin --(raw_to_csv --mode imu)--> imu.csv --(postprocess)--> _ana_python.csv
                                                                 --> _spec_python.csv
                                             --(mapplot)--------> _kart.png

Økter logget med bare rålogg har ingen <stamp>_imu.csv, og da stopper både
postprocess.py og mapplot.py med "Fant ingen *_imu.csv". Dette skriptet
rekonstruerer den fra <stamp>_raw.bin først, og kjører så mapplot.py som vanlig.
Med --source device brukes enhetens egen imu.csv i stedet, så de to kildene kan
sammenlignes på samme analysekjede.

Én generasjon økter (2026-08-13/14) har _gps.csv i modulens heltallsenheter
(lat_e7, gspeed_mms, hacc_mm), og det formatet stopper mapplot.py med "Mangler
kolonne 'lat'". Kopien inn i arbeidskatalogen skaleres derfor til det dekodede
formatet. Firmware skriver dekodet igjen, og de øktene kopieres uendret.

INGENTING SKRIVES I ØKTKATALOGEN. Alt som produseres havner i en egen
arbeidskatalog (default <øktkatalog>/<stamp>_<kilde>/), og inn dit kopieres de
filene analysen trenger (_gps.csv, _cfg.csv, _ses.csv, ...). Originalene leses,
aldri skrives. Det er også grunnen til at det kopieres og ikke symlinkes: en
symlink er en skriveveg tilbake til originalen, en kopi er det ikke.

Ingen fil overskrives uten --force, og --force gjelder bare inne i
arbeidskatalogen - peker --out-dir på selve øktkatalogen, avbrytes kjøringen så
snart én av målfilene finnes fra før.

Argumenter skriptet ikke kjenner sendes videre til mapplot.py uendret:

    python3 rawplot.py <øktkatalog>                          # raw.bin -> kart
    python3 rawplot.py <øktkatalog> --taper-f1 0.15 --taper-f2 0.3
    python3 rawplot.py <øktkatalog> --source device          # enhetens imu.csv
    python3 rawplot.py <øktkatalog> --reuse                  # behold imu.csv fra sist
    python3 rawplot.py <sti>/20260813_182728_raw.bin --dry-run
"""

import argparse
import os
import shutil
import subprocess
import sys
from glob import glob

HER = os.path.dirname(os.path.abspath(__file__))
RAW_TO_CSV = os.path.join(HER, "raw_to_csv.py")
MAPPLOT = os.path.join(HER, "mapplot.py")

sys.path.insert(0, HER)
from postprocess import read_csv_stream  # noqa: E402  (hale-regelen, ett sted)

# Filer som kopieres fra økta inn i arbeidskatalogen. _gps.csv er den eneste
# mapplot ikke klarer seg uten; _cfg.csv trengs av postprocess og av
# parametertabellen i figuren. _ana.csv/_spec.csv leses ikke av noen av dem, men
# blir med så arbeidskatalogen er en komplett økt man kan sammenligne i.
# _notat.txt kopieres bare hvis arbeidskatalogen ikke har en fra før: den er
# håndskrevet (mapplot --init-notat), og en kopi over den er et tapt notat.
KOPIER = ["_gps.csv", "_cfg.csv", "_ses.csv", "_ana.csv", "_spec.csv",
          "_notat.txt"]
BEVAR = ["_notat.txt"]
MAA_FINNES = ["_gps.csv"]

# Filene kjeden lager i arbeidskatalogen. Listet her for én grunn: å kunne si
# nei FØR noe kjøres, i stedet for å oppdage kollisjonen halvveis uti. Kopiene
# over står ikke her: de er reproduserbare avtrykk av originalene, ikke
# resultater, og å friske dem opp er alltid trygt.
LAGES = ["_imu.csv", "_ana_python.csv", "_spec_python.csv", "_spec.png",
         "_kart.png"]
# I tillegg lager postprocess <stamp>_imu_raw_<filter>.csv her - ett filter kjørt
# på råstrømmen per fil. De står IKKE i LAGES: de er mellomlagre som gjenbrukes
# på tvers av kjøringer (det er hele poenget med dem), så en kollisjon med dem er
# ikke noe å avbryte for. postprocess --regen-raw lager dem på nytt.
MELLOMLAGER = "_imu_raw_*.csv"

SUFFIKSER = ["_raw.bin", "_gps.csv", "_imu.csv", "_cfg.csv", "_ses.csv",
             "_ana.csv", "_spec.csv", "_notat.txt"]


def finn_okt(path):
    """(øktkatalog, stamp) fra en katalog- eller filsti.

    Godtar hvilken som helst av øktas filer, ikke bare .bin-en: man har som
    regel akkurat den fila man så på i utklippstavla."""
    if os.path.isdir(path):
        for suf in ("_raw.bin", "_gps.csv", "_imu.csv"):
            cand = sorted(glob(os.path.join(path, "*" + suf)))
            if cand:
                return os.path.abspath(path), os.path.basename(cand[0])[:-len(suf)]
        sys.exit(f"fant verken *_raw.bin, *_gps.csv eller *_imu.csv i {path}")
    if not os.path.isfile(path):
        sys.exit(f"fant ikke {path}")
    base = os.path.basename(path)
    for suf in SUFFIKSER:
        if base.endswith(suf):
            return os.path.dirname(os.path.abspath(path)), base[:-len(suf)]
    sys.exit(f"skjønner ikke hvilken økt {base} hører til "
             f"(ventet et navn som slutter på {', '.join(SUFFIKSER)})")


def velg_kilde(katalog, stamp, ønsket):
    """auto: rålogg hvis den finnes, ellers enhetens egen imu.csv."""
    raw = os.path.join(katalog, f"{stamp}_raw.bin")
    imu = os.path.join(katalog, f"{stamp}_imu.csv")
    if ønsket == "raw":
        if not os.path.isfile(raw):
            sys.exit(f"--source raw, men fant ikke {raw}")
        return "raw"
    if ønsket == "device":
        if not os.path.isfile(imu):
            sys.exit(f"--source device, men fant ikke {imu} "
                     f"(økta er logget uten WaveLogMode::Both?)")
        return "device"
    if os.path.isfile(raw):
        return "raw"
    if os.path.isfile(imu):
        return "device"
    sys.exit(f"fant verken {os.path.basename(raw)} eller "
             f"{os.path.basename(imu)} i {katalog}")


# Én firmware-generasjon skrev _gps.csv i modulens egne heltallsenheter (lat_e7,
# lng_e7, gspeed_mms, head_e5, hacc_mm) - økter fra 13. og 14. august 2026.
# mapplot.py og postprocess.py leser bare det dekodede formatet og stopper med
# "Mangler kolonne 'lat'" på en slik fil. Skaleringen gjøres derfor her, på
# kopien inn i arbeidskatalogen - samme rolle som rekonstruksjonen av imu.csv:
# gjør en økt lesbar for verktøykjeden, uten å røre originalen. Tabellen blir
# stående selv om firmware nå skriver dekodet igjen: de øktene finnes.
GPS_SKALA = {                 # nytt navn -> (gammelt navn, skala)
    "lat_e7": ("lat", 1e-7),
    "lng_e7": ("lon", 1e-7),
    "gspeed_mms": ("gspeed", 1e-3),          # mm/s -> m/s
    "vup_mms": ("vUp", 1e-3),                # mm/s -> m/s. Uten denne står
                                             # GPS-radene i rapporten tomme:
                                             # postprocess bygger elevasjons-
                                             # spekteret sitt av vUp alene
    "hacc_mm": ("hAccuracy", 1e-3),          # mm   -> m  (mapplot: "hAcc ... m")
    "head_e5": ("head", 1e-5),
}


def kopier_csv(src, dst, time_col):
    """Kopier en strømmefil til arbeidskatalogen, skalert til det dekodede
    formatet om den trenger det. Returnerer (CsvKutt, skalert).

    Hale-regelen ligger IKKE her, men i postprocess.read_csv_stream, sammen med
    leserne som bruker den videre. Denne funksjonen gjør bare det som er unikt
    for kopien: kolonnenavn og enheter."""
    idx, rader, kutt = read_csv_stream(src, time_col)
    navn = list(idx)
    skalert = any(c in GPS_SKALA for c in navn)
    ut_navn = [GPS_SKALA[c][0] if c in GPS_SKALA else c for c in navn]
    skala = [GPS_SKALA[c][1] if c in GPS_SKALA else None for c in navn]

    with open(dst, "w") as o:
        o.write(",".join(ut_navn) + "\n")
        for r in rader:
            o.write(",".join(
                f"{int(v) * s:.7f}".rstrip("0").rstrip(".") if s else v
                for v, s in zip(r, skala)) + "\n")
    return kutt, skalert


def kjør(cmd, logg):
    """Kjør og vis utskriften mens den kommer - kjeden tar minutter på en stor
    rålogg, og en stum terminal er ikke til å skille fra en hengt prosess.
    Kopien i loggfila er der fordi rawlog.summary() sier hvor mange blokker som
    gikk tapt, og det svaret vil man ha igjen senere."""
    print(f"$ {' '.join(cmd)}", flush=True)
    with open(logg, "w") as f:
        f.write(f"$ {' '.join(cmd)}\n")
        f.flush()
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True,
                             env=dict(os.environ, MPLBACKEND="Agg"))
        for line in p.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            f.write(line)
        rc = p.wait()
    if rc != 0:
        steg = os.path.basename(next(a for a in cmd if a.endswith(".py")))
        sys.exit(f"\n{steg} feilet (exit {rc}) - se {logg}")


def main():
    # allow_abbrev=False: uten den spiser --out-dir mapplots egen --out som
    # prefiks, og figuren havner et helt annet sted enn brukeren ba om.
    ap = argparse.ArgumentParser(
        allow_abbrev=False,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="raw.bin -> imu.csv -> postprocess -> mapplot, i én kommando.",
        epilog="Ukjente argumenter sendes videre til mapplot.py "
               "(--taper-f1, --taper-f2, --no-map, --init-notat, ...).")
    ap.add_argument("path", help="øktkatalog eller en av øktas filer")
    ap.add_argument("--source", choices=["auto", "raw", "device"], default="auto",
                    help="raw: rekonstruer imu.csv fra <stamp>_raw.bin. "
                         "device: bruk enhetens egen <stamp>_imu.csv slik den "
                         "ligger. auto (default): raw hvis råloggen finnes")
    ap.add_argument("--out-dir",
                    help="arbeidskatalog (default <øktkatalog>/<stamp>_<kilde>)")
    ap.add_argument("--ahrs", choices=["madgwick", "kalman"],
                    help="overstyr filteret i rekonstruksjonen "
                         "(default: orientation_name fra _cfg.csv)")
    ap.add_argument("--allow-damaged", action="store_true",
                    help="fortsett selv om råloggen mistet blokker. Alt etter "
                         "første tap er feiltolket - se raw_to_csv.py")
    ap.add_argument("--reuse", action="store_true",
                    help="hopp over rekonstruksjonen hvis arbeidskatalogen "
                         "alt har en imu.csv (rask ny figur med annen taper)")
    ap.add_argument("--force", action="store_true",
                    help="tillat overskriving INNE i arbeidskatalogen")
    ap.add_argument("--dry-run", action="store_true",
                    help="vis hva som ville skjedd, ikke gjør det")
    args, videre = ap.parse_known_args()

    katalog, stamp = finn_okt(args.path)
    kilde = velg_kilde(katalog, stamp, args.source)
    ut = os.path.abspath(args.out_dir or os.path.join(katalog, f"{stamp}_{kilde}"))
    i_økta = os.path.normpath(ut) == os.path.normpath(katalog)

    print(f"økt:            {katalog}")
    print(f"stamp:          {stamp}")
    print(f"kilde:          {kilde}"
          + (f"  ({stamp}_raw.bin, AHRS "
             f"{args.ahrs or 'fra cfg.csv'})" if kilde == "raw"
             else f"  ({stamp}_imu.csv fra enheten)"))
    print(f"arbeidskatalog: {ut}")

    for suf in MAA_FINNES:
        if not os.path.isfile(os.path.join(katalog, stamp + suf)):
            sys.exit(f"mangler {stamp}{suf} i økta - mapplot.py trenger den")

    # Gjenbruk av imu.csv avgjøres før kollisjonssjekken: fila er da et ferdig
    # mellomresultat og ikke lenger noe vi står i fare for å skrive over.
    imu_ut = os.path.join(ut, f"{stamp}_imu.csv")
    gjenbruk = args.reuse and os.path.isfile(imu_ut)
    if args.reuse and not gjenbruk:
        print("  --reuse: ingen imu.csv i arbeidskatalogen ennå - lager den")

    kopier = [suf for suf in KOPIER
              if os.path.isfile(os.path.join(katalog, stamp + suf))]
    if kilde == "device" and not gjenbruk:
        kopier.append("_imu.csv")

    # Filene kjeden LAGER overskrives ikke uten --force. Peker --out-dir på
    # selve øktkatalogen, hjelper ikke --force heller: da er hver kollisjon en
    # måling som forsvinner, ikke et mellomresultat som lages på nytt.
    lages = [suf for suf in LAGES if not (suf == "_imu.csv" and gjenbruk)]
    if kilde == "device" and not gjenbruk:
        lages.remove("_imu.csv")             # kopieres, ikke lages
    kollisjon = [stamp + suf for suf in lages
                 if os.path.isfile(os.path.join(ut, stamp + suf))]
    if kollisjon and not (args.force and not i_økta):
        hint = ("--out-dir peker på øktkatalogen, så --force gjelder ikke her - "
                "velg en annen katalog." if i_økta else
                "bruk --out-dir <ny katalog>, --reuse eller --force.")
        sys.exit(f"AVBRYTER: finnes fra før i {ut}:\n"
                 + "".join(f"  {n}\n" for n in kollisjon) + f"{hint}")

    # Kopier som ville skrevet over noe: samme fil (--out-dir = øktkatalogen),
    # eller et notat som alt ligger i arbeidskatalogen.
    kopier = [suf for suf in kopier
              if not (i_økta or (suf in BEVAR
                                 and os.path.isfile(os.path.join(ut, stamp + suf))))]
    print(f"kopierer:       {', '.join(stamp + s for s in kopier) or '(ingenting)'}")
    if args.dry_run:
        print("\n--dry-run, stopper her. Ville kjørt:")
        if not gjenbruk and kilde == "raw":
            print(f"  raw_to_csv.py {stamp}_raw.bin --mode imu -o {imu_ut}")
        print(f"  mapplot.py {ut} {' '.join(videre)}")
        return

    os.makedirs(ut, exist_ok=True)
    for suf in kopier:
        src, dst = os.path.join(katalog, stamp + suf), os.path.join(ut, stamp + suf)
        # De to strømmefilene er de eneste som kan være avbrutt midt i en rad -
        # resten skrives i én omgang, og har enten kommet eller ikke.
        if suf in ("_gps.csv", "_imu.csv"):
            kutt, skalert = kopier_csv(
                src, dst, "rel_ms" if suf == "_gps.csv" else "win_start_ms")
            if skalert:
                print(f"  {stamp}{suf}: heltallsformat fra firmware skalert "
                      f"(lat_e7 -> lat, gspeed_mms -> m/s, hacc_mm -> m)")
            print(f"  {stamp}{suf}: {kutt.rader} hele rader"
                  + (f", {kutt.total - kutt.lest} B hale forkastet: {kutt.grunn} "
                     f"(avbrutt fangst - fila ligger på preallokert lengde)"
                     if kutt.grunn else ""))
        else:
            shutil.copy2(src, dst)

    if kilde == "raw" and not gjenbruk:
        print(f"\n--- rekonstruerer {stamp}_imu.csv fra råloggen ---", flush=True)
        cmd = [sys.executable, "-u", RAW_TO_CSV,
               os.path.join(katalog, f"{stamp}_raw.bin"),
               "--mode", "imu", "-o", imu_ut]
        if args.ahrs:
            cmd += ["--ahrs", args.ahrs]
        if args.allow_damaged:
            cmd.append("--allow-damaged")
        kjør(cmd, os.path.join(ut, f"{stamp}_raw_to_csv.log"))
    elif gjenbruk:
        print(f"\n--- bruker {os.path.basename(imu_ut)} som den ligger (--reuse) ---")

    # --source device betyr "vis det enheten selv regnet ut". Da skal ikke
    # alternativfiltrene snike inn bakveien fra råloggen som ligger i
    # øktkatalogen ved siden av: postprocess leter ett nivå opp, og uten dette
    # ville device- og raw-katalogen fått IDENTISKE Kalman/NXP/MEKF-rader og
    # bare skilt seg i den ene raden enheten faktisk skrev - stikk i strid med
    # hva de to katalogene er til for.
    ekstra = ["--raw", "off"] if kilde == "device" else []
    if kilde == "device" and any(a == "--raw" for a in videre):
        ekstra = []                            # brukeren har bedt om noe annet
    print(f"\n--- postprocess + kart ---", flush=True)
    kjør([sys.executable, "-u", MAPPLOT, ut] + ekstra + videre,
         os.path.join(ut, f"{stamp}_mapplot.log"))

    print(f"\n=== ferdig, alt ligger i {ut} ===")
    for navn in sorted(os.listdir(ut)):
        print(f"  {navn}")


if __name__ == "__main__":
    main()
