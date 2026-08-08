#!/usr/bin/env python3
"""Generer fir_coeffs.h til firmware - FIR-koeffisienttabeller som constexpr-arrays.

Firmware kan ikke regne ut koeffisientene selv (ingen sinc/cos i constexpr, og et
runtime-oppsett ville brent RAM pa noe som er konstant), sa de legges i flash som
genererte tabeller. Poenget med a generere dem herfra er PARITET: samme
firwin_lowpass som postprocess.py bruker offline, sa device og offline filtrerer
med nayaktig de samme tallene i stedet for to uavhengige implementasjoner som
driver fra hverandre.

TABELLENE ER INDEKSERT PA DESIMERINGSFAKTOR, IKKE PA RATEPAR. Det er ikke en
forenkling, det falger av formelen:

    h[n] = sinc(2*(fc/fs)*(n - (N-1)/2)) * hamming[n],  normalisert sa sum(h) = 1

fc og fs opptrer bare som forholdet fc/fs, og begge desimeringstrinnene i firmware
har fc = fs_ut/2. Altsa er fc/fs = 1/(2*D) der D = fs_inn/fs_ut, og tabellen
avhenger utelukkende av D. 960->100 og 480->50 gir derfor BIT-IDENTISKE tapper -
verifisert, ikke antatt.

Konsekvensen er at én generert fil dekker alle rateparene som gir en D i lista
under, og firmware velger tabell med et constexpr-oppslag. Ratene kan endres i
wave_config.h uten a regenerere noe, sa lenge forholdet finnes her.

Naklene er heltall = round(10*D), siden D kan vaere 9,6:

    D = 1,2 / 2,4 / 4,8   -> trinn 1 ved lav ODR
    D = 9,6               -> trinn 1 @ 960->100 OG 480->50
    D = 19,2              -> trinn 1 @ 960->50
    D = 5 / 10            -> trinn 2 @ 50 / 100 Hz rader

Bruk:
  python gen_fir_table.py \\
      -o ../firmware/common_libraries/wave_manager/src/fir_coeffs.h

Legg til en ny rate-kombinasjon ved a utvide --decim; firmwarens static_assert
sier fra med hvilken D som mangler.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fir  # noqa: E402

# Desimeringsfaktorene firmware kan tenkes a bruke. kImuOdrHz i {120,240,480,960}
# mot kOutputRateHz i {50,100} gir trinn 1; kOutputRateHz mot kVaccFsHz = 10 gir
# trinn 2. Ubrukte tabeller kastes av lenkeren (-fdata-sections + --gc-sections),
# sa lista koster ingenting a holde raus.
DEFAULT_DECIM = [1.2, 2.4, 4.8, 5.0, 9.6, 10.0, 19.2]


def decim_key(d):
    """Heltallsnakkel = 10*D. Kun eksakte tideler er lovlige, ellers ville to ulike
    D-er kunnet kollidere i samme navn uten at noen la merke til det."""
    k = int(round(10.0 * d))
    if abs(10.0 * d - k) > 1e-9:
        sys.exit(f"desimeringsfaktor {d!r} er ikke et helt antall tideler")
    if k < 10:
        sys.exit(f"desimeringsfaktor {d!r} < 1 gir cutoff over Nyquist")
    return k


def fmt_table(name, d, coeffs, per_line=4):
    """En float32-repr per koeffisient. %.9g er kort nok til a lese og langt nok
    til a runde tilbake til nayaktig samme float32."""
    out = [f"// D = {d:g}: firwin_lowpass({len(coeffs)}, 0.5, {d:g}) => fc/fs = {0.5 / d:.9g}",
           f"inline constexpr float {name}[kFirNtap] = {{"]
    for i in range(0, len(coeffs), per_line):
        chunk = coeffs[i:i + per_line]
        out.append("    " + " ".join(f"{np.float32(v):.9g}f," for v in chunk))
    out.append("};")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--decim", type=float, nargs="+", default=DEFAULT_DECIM,
                    help="desimeringsfaktorer det skal lages tabell for "
                         f"(default {DEFAULT_DECIM})")
    ap.add_argument("--ntap", type=int, default=fir.NTAP, help=f"antall tap (default {fir.NTAP})")
    ap.add_argument("-o", "--out", default="-", help="utfil, '-' for stdout")
    args = ap.parse_args()

    keys = sorted({decim_key(d) for d in args.decim})

    tables = []
    for k in keys:
        d = k / 10.0
        # fc = 0.5 og fs = D gir fc/fs = 1/(2D) - nayaktig det (fs_ut/2)/fs_inn var
        # da tabellene ble generert per ratepar.
        h = fir.firwin_lowpass(args.ntap, 0.5, d)
        # firwin_lowpass normaliserer til sum(h) = 1 i float64, men vi lagrer float32.
        # Avviket ma vaere sa lite at DC-forsterkningen fortsatt er 1 innenfor stayen -
        # ellers ville et konstant accel-bias blitt skalert av filteret.
        s = float(np.sum(np.float32(h).astype(np.float64)))
        if abs(s - 1.0) > 1e-5:
            sys.exit(f"D = {d:g}: float32-sum {s!r} avviker for mye fra 1.0")
        tables.append((k, d, h))

    # En kjede av ternaeroperatorer: constexpr-funksjonen ma vaere ett uttrykk for a
    # kunne evalueres i en static_assert.
    selector = "\n".join(
        f"  {'return' if i == 0 else '     :'} k == {k:3d} ? kFirTapsD{k}"
        for i, (k, _, _) in enumerate(tables))
    selector += "\n       :            nullptr;"

    body = f"""#ifndef FIR_COEFFS_H
#define FIR_COEFFS_H

#include "fir.h"   // kFirNtap - the tables are sized by it and generated for it

/*
  GENERERT av tools/gen_fir_table.py - IKKE rediger for hand.

  Vindusbasert sinc-lavpass (Hamming, normalisert sa sum(h) = 1), samme formel og
  samme funksjon som postprocess.py bruker offline: fir.firwin_lowpass. Sum = 1 gir
  nayaktig enhetsforsterkning ved DC, sa et gjenvaerende accel-bias ikke skaleres.

  EN TABELL PER DESIMERINGSFAKTOR, ikke per ratepar. Koeffisientene avhenger bare av
  fc/fs, og siden begge trinn har fc = fs_ut/2 er det 1/(2*D). 960->100 og 480->50
  er derfor SAMME tabell - bit for bit. Det er grunnen til at ratene kan endres i
  wave_config.h uten at noe her ma regenereres.

  Navnesuffikset er 10x desimeringsfaktoren, siden D kan vaere 9,6:
  kFirTapsD96 = D 9,6, kFirTapsD100 = D 10,0.

  Denne filen kjenner ingen rater og inkluderer med vilje IKKE wave_config.h - det
  er wave_config.h som inkluderer denne og gjar oppslaget. Motsatt vei blir sirkulaert.

  Tabellene ligger i flash ({len(tables)} x {args.ntap * 4} B i kilde); lenkeren kaster de
  ubrukte (-fdata-sections + --gc-sections), sa bare de to som velges koster noe.
  Bare forsinkelseslinjene i FirDecimator koster RAM.
*/

// Tapsantallet er bakt inn i tabellene. Endres det uten a regenerere, blir filteret
// stille feil - derfor et gjerde, ikke en kommentar.
static_assert(kFirNtap == {args.ntap},
              "kFirNtap er endret - regenerer fir_coeffs.h med "
              "tools/gen_fir_table.py --ntap <n>");

"""

    for k, d, h in tables:
        body += fmt_table(f"kFirTapsD{k}", d, h) + "\n\n"

    body += f"""// Velg tabell fra desimeringsfaktoren x 10. Returnerer nullptr for en faktor det
// ikke finnes tabell for, slik at wave_config.h kan stoppe byggingen med en melding
// som sier hva som mangler - i stedet for a filtrere med feil koeffisienter.
constexpr const float *firTapsForDecimX10(uint16_t k) {{
{selector}
}}

#endif  // FIR_COEFFS_H
"""

    if args.out == "-":
        sys.stdout.write(body)
    else:
        with open(args.out, "w") as f:
            f.write(body)
        print(f"skrev {args.out}: {len(tables)} tabeller a {args.ntap} tap, "
              f"D = {', '.join(f'{d:g}' for _, d, _ in tables)}")


if __name__ == "__main__":
    main()
