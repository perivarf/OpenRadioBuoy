#!/usr/bin/env bash
#
# Begge kildene til samme økt, plottet hver for seg - og satt opp mot hverandre
# til slutt.
#
#   <stamp>_device/   enhetens egen imu.csv, altså tallene ombord-kjeden faktisk
#                     regnet på (FIR + AHRS i firmware, float32, i sanntid)
#   <stamp>_raw/      imu.csv rekonstruert fra raw.bin av raw_to_csv.py (samme
#                     kjede om igjen offline, float64, AHRS på 480 Hz-strømmen)
#
# Hver kjøring er en egen rawplot.py, i sin egen katalog, med samme analysevalg.
# Det er hele poenget: to uavhengige veier fra de samme samplene til Hs/Tz/Tc,
# som ikke kan blande seg med hverandre. Spriker de, ligger feilen i kjeden
# mellom dem - ikke i sjøen.
#
# Økta røres ikke: rawplot.py skriver bare i katalogene over.
#
#   ./plot_session.sh <øktkatalog>...
#   ./plot_session.sh <øktkatalog> --taper-f1 0.15 --taper-f2 0.3
#   ./plot_session.sh <øktkatalog> --reuse --force   # ny figur, behold imu.csv
#   ./plot_session.sh <økt1> <økt2> -- --no-map      # flere økter, felles valg
#
# Første argument som begynner med '-' avslutter lista over økter; resten
# sendes videre til rawplot.py (som igjen sender det den ikke kjenner til
# mapplot.py). '--' gjør det samme skillet eksplisitt.
#
set -uo pipefail

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAWPLOT="$TOOLS/rawplot.py"

# Hjelpeteksten ER kommentarblokka over: én tekst å holde oppdatert, ikke to.
usage() {
  awk 'NR > 2 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
  exit "${1:-0}"
}

SESSIONS=()
EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage 0 ;;
    --) shift; EXTRA+=("$@"); break ;;
    -*) EXTRA+=("$@"); break ;;
    *)  SESSIONS+=("$1"); shift ;;
  esac
done
[ ${#SESSIONS[@]} -gt 0 ] || usage 1

# --out-dir ville sendt begge kildene til SAMME katalog, og da er ikke lenger
# figurene til å skille fra hverandre - det er nettopp det dette skriptet er til
# for. Katalognavnene er derfor ikke til forhandling her; kjør rawplot.py
# direkte hvis du vil styre dem.
for a in ${EXTRA[@]+"${EXTRA[@]}"}; do
  case "$a" in
    --out-dir|--out-dir=*)
      echo "--out-dir kan ikke brukes her: begge kildene ville havnet i samme" >&2
      echo "katalog. Kjør rawplot.py --source raw/device direkte i stedet." >&2
      exit 2 ;;
  esac
done

# Ett tall ut av en key,value-CSV. Tåler at fila eller nøkkelen mangler - da er
# svaret en tankestrek, ikke en tom kolonne man kan lese som null.
kv() {
  [ -f "$1" ] || { printf -- '-'; return; }
  awk -F, -v k="$2" '$1 == k { printf "%s", $2; f = 1 } END { if (!f) printf "-" }' "$1"
}

rad() {   # rad <merkelapp> <ana-fil>
  printf '  %-22s %8s %8s %8s %8s %8s\n' "$1" \
    "$(kv "$2" Hs_madgwick)" "$(kv "$2" Tz_madgwick)" "$(kv "$2" Tc_madgwick)" \
    "$(kv "$2" Hs_gps)" "$(kv "$2" Tz_gps)"
}

status=0
for sess in "${SESSIONS[@]}"; do
  sess="${sess%/}"
  [ -d "$sess" ] || { echo "hopper over $sess: ikke en katalog" >&2; status=1; continue; }
  stamp="$(basename "$sess")"
  echo
  echo "############################################################"
  echo "# $stamp"
  echo "############################################################"

  kjort=()
  for kilde in device raw; do
    case "$kilde" in
      device) [ -f "$sess/${stamp}_imu.csv" ] || {
                echo; echo ">>> hopper over device: ingen ${stamp}_imu.csv "\
"(økta er logget uten WaveLogMode::Both)"; continue; } ;;
      raw)    [ -f "$sess/${stamp}_raw.bin" ] || {
                echo; echo ">>> hopper over raw: ingen ${stamp}_raw.bin"; continue; } ;;
    esac
    echo
    echo ">>> $kilde"
    if python3 -u "$RAWPLOT" "$sess" --source "$kilde" ${EXTRA[@]+"${EXTRA[@]}"}; then
      kjort+=("$kilde")
    else
      echo ">>> $kilde FEILET" >&2
      status=1
    fi
  done

  # Sammenligningen er hele grunnen til at begge kjøres i samme kommando.
  # Firmwarens egen _ana.csv står nederst uten GPS-kolonner - den regner ikke
  # GPS-referansen ombord, og seglengden er som regel en annen enn python-
  # kjedens, så den raden er en pekepinn og ikke en fasit.
  if [ ${#kjort[@]} -gt 0 ]; then
    echo
    echo "=== $stamp: Hs/Tz/Tc ==="
    printf '  %-22s %8s %8s %8s %8s %8s\n' "kilde" "Hs" "Tz" "Tc" "Hs_gps" "Tz_gps"
    for kilde in "${kjort[@]}"; do
      rad "$kilde" "$sess/${stamp}_${kilde}/${stamp}_ana_python.csv"
    done
    ana="$sess/${stamp}_ana.csv"
    if [ -f "$ana" ]; then
      # ASCII i merkelappen: printf %-22s teller bytes, og en 'ø' ville skjøvet
      # hele raden ut av kolonnene.
      printf '  %-22s %8s %8s %8s %8s %8s\n' "firmware (_ana.csv)" \
        "$(kv "$ana" Hs)" "$(kv "$ana" Tz)" "$(kv "$ana" Tc)" "-" "-"
    fi
    for kilde in "${kjort[@]}"; do
      echo "  kart: $sess/${stamp}_${kilde}/${stamp}_kart.png"
    done
  fi
done

exit "$status"
