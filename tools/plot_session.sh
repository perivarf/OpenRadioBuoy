#!/usr/bin/env bash
#
# Both sources for the same session, plotted separately - and put up against each
# other at the end.
#
#   <stamp>_device/   the device's own imu.csv, that is the numbers the onboard
#                     chain actually computed on (FIR + AHRS in firmware, float32,
#                     in real time)
#   <stamp>_raw/      imu.csv reconstructed from raw.bin by raw_to_csv.py (the same
#                     chain over again offline, float64, AHRS on the 480 Hz stream)
#
# Each run is its own rawplot.py, in its own directory, with the same analysis
# options. That is the whole point: two independent paths from the same samples to
# Hs/Tz/Tc, which cannot mix with each other. If they diverge, the error is in the
# chain between them - not in the sea.
#
# The session itself is untouched: rawplot.py only writes in the directories above.
#
#   ./plot_session.sh <session dir>...
#   ./plot_session.sh <session dir> --taper-f1 0.15 --taper-f2 0.3
#   ./plot_session.sh <session dir> --reuse --force   # new figure, keep imu.csv
#   ./plot_session.sh <sess1> <sess2> -- --no-map     # several sessions, same options
#
# The first argument beginning with '-' ends the list of sessions; the rest is passed
# on to rawplot.py (which in turn passes on what it does not recognise to
# mapplot.py). '--' makes the same split explicit.
#
set -uo pipefail

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAWPLOT="$TOOLS/rawplot.py"

# The help text IS the comment block above: one text to keep up to date, not two.
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

# --out-dir would send both sources to the SAME directory, and then the figures can no
# longer be told apart - which is precisely what this script exists for.
for a in ${EXTRA[@]+"${EXTRA[@]}"}; do
  case "$a" in
    --out-dir|--out-dir=*)
      echo "--out-dir cannot be used here: both sources would end up in the same" >&2
      echo "directory. Run rawplot.py --source raw/device directly instead." >&2
      exit 2 ;;
  esac
done

# One number out of a key,value CSV. Tolerates a missing file or key - the answer is
# then a dash, not an empty column that could be read as zero.
kv() {
  [ -f "$1" ] || { printf -- '-'; return; }
  awk -F, -v k="$2" '$1 == k { printf "%s", $2; f = 1 } END { if (!f) printf "-" }' "$1"
}

row() {   # row <label> <ana file>
  printf '  %-22s %8s %8s %8s %8s %8s\n' "$1" \
    "$(kv "$2" Hs_madgwick)" "$(kv "$2" Tz_madgwick)" "$(kv "$2" Tc_madgwick)" \
    "$(kv "$2" Hs_gps)" "$(kv "$2" Tz_gps)"
}

status=0
for sess in "${SESSIONS[@]}"; do
  sess="${sess%/}"
  [ -d "$sess" ] || { echo "skipping $sess: not a directory" >&2; status=1; continue; }
  stamp="$(basename "$sess")"
  echo
  echo "############################################################"
  echo "# $stamp"
  echo "############################################################"

  ran=()
  for src in device raw; do
    case "$src" in
      device) [ -f "$sess/${stamp}_imu.csv" ] || {
                echo; echo ">>> skipping device: no ${stamp}_imu.csv "\
"(the session was logged without WaveLogMode::Both)"; continue; } ;;
      raw)    [ -f "$sess/${stamp}_raw.bin" ] || {
                echo; echo ">>> skipping raw: no ${stamp}_raw.bin"; continue; } ;;
    esac
    echo
    echo ">>> $src"
    if python3 -u "$RAWPLOT" "$sess" --source "$src" ${EXTRA[@]+"${EXTRA[@]}"}; then
      ran+=("$src")
    else
      echo ">>> $src FAILED" >&2
      status=1
    fi
  done

  # The firmware's own _ana.csv sits at the bottom without GPS columns - it does not
  # compute the GPS reference onboard, and its segment length is usually a different
  # one from the python chain's, so that row is an indication and not ground truth.
  if [ ${#ran[@]} -gt 0 ]; then
    echo
    echo "=== $stamp: Hs/Tz/Tc ==="
    printf '  %-22s %8s %8s %8s %8s %8s\n' "source" "Hs" "Tz" "Tc" "Hs_gps" "Tz_gps"
    for src in "${ran[@]}"; do
      row "$src" "$sess/${stamp}_${src}/${stamp}_ana_python.csv"
    done
    ana="$sess/${stamp}_ana.csv"
    if [ -f "$ana" ]; then
      # Labels stay ASCII: printf %-22s counts bytes, not characters.
      printf '  %-22s %8s %8s %8s %8s %8s\n' "firmware (_ana.csv)" \
        "$(kv "$ana" Hs)" "$(kv "$ana" Tz)" "$(kv "$ana" Tc)" "-" "-"
    fi
    for src in "${ran[@]}"; do
      echo "  map: $sess/${stamp}_${src}/${stamp}_kart.png"   # mapplot.py's own name
    done
  fi
done

exit "$status"
