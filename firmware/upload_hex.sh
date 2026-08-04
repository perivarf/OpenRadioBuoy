#!/usr/bin/env bash
#
# Last opp en HEX-fil til STM32 (STM32WLE5 / lora_e5_mini) via ST-Link.
#
# Bruker STM32CubeProgrammer i stedet for openocd, fordi openocd-veien viste seg
# skjør på denne brikken. Kobler til UNDER RESET (mode=UR) slik at vi fanger
# kjernen ut av reset -- da slipper du å holde reset-knappen for hånd, og gammel
# firmware (SubGHz/klokker) rekker aldri å gjøre SWD utilgjengelig.
#
# Bruk:
#   ./upload_hex.sh                      # laster opp .pio-bygget automatisk
#   ./upload_hex.sh sti/til/firmware.hex # laster opp en spesifikk fil
#
# Miljøvariabler (valgfrie):
#   FREQ=240   ./upload_hex.sh   # senk SWD-klokka (kHz) hvis "communication failure"
#   MODE=HOTPLUG ./upload_hex.sh # hvis NRST ikke er wiret (må da holde reset selv)
#   NOERASE=1  ./upload_hex.sh   # hopp over full-chip erase (raskere, men risikabelt)
#
set -euo pipefail

# --- finn STM32_Programmer_CLI -------------------------------------------------
CLI="${STM32_CLI:-}"
if [[ -z "$CLI" ]]; then
    for c in \
        "$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI" \
        "$(command -v STM32_Programmer_CLI 2>/dev/null || true)"; do
        if [[ -n "$c" && -x "$c" ]]; then CLI="$c"; break; fi
    done
fi
if [[ -z "$CLI" || ! -x "$CLI" ]]; then
    echo "FEIL: fant ikke STM32_Programmer_CLI. Sett STM32_CLI=/sti/til/CLI." >&2
    exit 1
fi

# --- finn HEX-fila -------------------------------------------------------------
HEX="${1:-}"
if [[ -z "$HEX" ]]; then
    # standard PlatformIO-byggeutgang (compile_to_hex.py lager .hex fra .elf)
    HEX="$(ls -t .pio/build/*/firmware.hex 2>/dev/null | head -1 || true)"
fi
if [[ -z "$HEX" || ! -f "$HEX" ]]; then
    echo "FEIL: fant ingen HEX-fil. Bygg med 'pio run' eller oppgi sti:" >&2
    echo "      ./upload_hex.sh sti/til/firmware.hex" >&2
    exit 1
fi

# --- parametere ----------------------------------------------------------------
FREQ="${FREQ:-4000}"        # SWD-klokke i kHz; senk hvis marginal wiring
MODE="${MODE:-UR}"         # UR = connect under reset (bruker NRST-ledningen)

CONNECT=(port=SWD "mode=$MODE" freq="$FREQ")
[[ "$MODE" == "UR" ]] && CONNECT+=(reset=HWrst)

echo "==> CLI  : $CLI"
echo "==> HEX  : $HEX"
echo "==> Koble: ${CONNECT[*]}"
echo

# --- last opp ------------------------------------------------------------------
# -e all      : full-chip erase (unngår "block write failed" fra gammel firmware)
# -d ... --verify : skriv og les-tilbake-verifiser
# -rst        : reset og start den nye firmwaren etterpå

# Sjekk: HOTPLUG + NOERASE er ikke kompatibelt
# (STM32CubeProgrammer gjør alltid sektor-erase i HOTPLUG-modus)
if [[ -n "${NOERASE:-}" ]] && [[ "$MODE" == "HOTPLUG" ]]; then
  echo "ADVARSEL: NOERASE + HOTPLUG er ikke kompatibelt." >&2
  echo "  HOTPLUG-modus gjør alltid sektor-erase." >&2
  echo "  Velg: MODE=UR (med NRST) eller hold reset-knappen manuelt." >&2
  exit 1
fi

ERASE=(-e all)
[[ -n "${NOERASE:-}" ]] && ERASE=()

exec "$CLI" -c "${CONNECT[@]}" \
    "${ERASE[@]}" \
    -d "$HEX" --verify \
    -rst
