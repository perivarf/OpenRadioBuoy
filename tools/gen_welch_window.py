#!/usr/bin/env python3
"""Generer welch_window.h til firmware - Welch-vindusvektene som constexpr-array.

Vektene avhenger bare av kWelchSegLen og kWelchWindow, og begge er constexpr i
wave_config.h. De er altsa de samme for hvert eneste segment i hver eneste fangst -
men firmware kan ikke regne dem ut selv (ingen sin/cos i constexpr), sa fram til na
har accumSegment kalt sinf 1024 ganger per segment. Pa en M4 uten FPU er det den
dyreste enkeltoperasjonen i FFT-en, og maling 2026-08-15 satte hele accumSegment til
117 ms - lenge nok til at FIFO-en samler opp 136 av sine 256 ord mens den star pa.

Samme paritetspoeng som gen_fir_table.py: tabellen genereres fra welch.window_weights,
som ER funksjonen offline-kjeden bruker. Da kan device og offline ikke drive fra
hverandre, fordi det bare finnes én formel.

Et RAM-oppsett i begin() var alternativet og er utelukket: 4 kB til, og bss ligger
allerede pa 84 % av de 64 kB. Som constexpr havner tabellen i .rodata (flash, 52 %
brukt) og koster ingen RAM i det hele tatt.

Bruk:
  python gen_welch_window.py \\
      -o ../firmware/common_libraries/wave_manager/src/welch_window.h

Endres kWelchSegLen eller kWelchWindow i wave_config.h, ma denne kjares pa nytt.
Firmwarens static_assert stopper byggingen og sier hvilken av dem som ikke stemmer.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import welch  # noqa: E402

# Ma falge enum class WindowType { Hann, Hamming } i wave_config.h. Naklene her er
# strengene welch.window_weights forstar; verdiene er enumets tallverdier, og det er
# de som havner i gjerdet firmware sjekker mot.
WINDOW_KIND = {"hann": 0, "hamming": 1}

# kWelchSegLen i wave_config.h. Speilet her framfor a leses ut av headeren: en parser
# for C++ ville vaert en ny ting som kunne ryke stille, og static_assert-en pa den
# andre siden fanger uansett et avvik ved byggetid.
DEFAULT_SEGLEN = 1024
DEFAULT_WINDOW = "hann"


def lit(v):
    """En float32-repr. %.9g er kort nok til a lese og langt nok til a runde tilbake
    til nayaktig samme float32 - samme konvensjon som fir_coeffs.h.

    Punktumet ma tvinges inn. Hann-vinduet starter i nayaktig 0, og %.9g gir da "0",
    som med suffikset blir "0f" - ikke et float-literal i det hele tatt, men et
    heltall fulgt av en identifikator. fir_coeffs.h slipper unna fordi ingen av
    tappene der er eksakt null."""
    s = f"{np.float32(v):.9g}"
    if not any(c in s for c in ".eE"):
        s += ".0"
    return s + "f"


def fmt_table(name, weights, per_line=4):
    out = [f"inline constexpr float {name}[{len(weights)}] = {{"]
    for i in range(0, len(weights), per_line):
        chunk = weights[i:i + per_line]
        out.append("    " + " ".join(lit(v) + "," for v in chunk))
    out.append("};")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seglen", type=int, default=DEFAULT_SEGLEN,
                    help=f"kWelchSegLen (default {DEFAULT_SEGLEN})")
    ap.add_argument("--window", choices=sorted(WINDOW_KIND), default=DEFAULT_WINDOW,
                    help=f"kWelchWindow (default {DEFAULT_WINDOW})")
    ap.add_argument("-o", "--out", default="-", help="utfil, '-' for stdout")
    args = ap.parse_args()

    n = args.seglen
    if n < 2 or (n & (n - 1)) != 0:
        sys.exit(f"seglen {n} er ikke en toerpotens - FFT-en er radix-2")

    w = welch.window_weights(n, args.window)

    # Gjerdet mot en tabell som ser rimelig ut men er feil vindu: begge vinduene er
    # symmetriske om N/2 og ligger i [0,1], og Hann starter i nayaktig 0.
    if w.min() < -1e-12 or w.max() > 1.0 + 1e-12:
        sys.exit(f"vindusvekter utenfor [0,1]: {w.min()!r}..{w.max()!r}")
    if not np.allclose(w[1:], w[1:][::-1], atol=1e-12):
        sys.exit("vindusvektene er ikke symmetriske om N/2 - feil formel")

    # Samme sjekk som DC-forsterkningen i gen_fir_table.py, og av samme grunn: sum(w^2)
    # er normaliseringen accumSegment deler pa, sa den er det float32-avrundingen kunne
    # flyttet nivaet pa. Firmware summerer float32-vektene i double, sa det er den
    # summen som ma stemme - ikke float64-vektenes.
    s2_f64 = float(np.sum(w * w))
    s2_f32 = float(np.sum(np.float32(w).astype(np.float64) ** 2))
    if abs(s2_f32 - s2_f64) > 1e-6 * s2_f64:
        sys.exit(f"sum(w^2) flyttet seg for mye i float32: {s2_f64!r} -> {s2_f32!r}")

    kind = WINDOW_KIND[args.window]
    body = f"""#ifndef WELCH_WINDOW_H
#define WELCH_WINDOW_H

#include "wave_config.h"   // kWelchSegLen, kWelchWindow - the fence below checks both

/*
  GENERERT av tools/gen_welch_window.py - IKKE rediger for hand.

  Vindusvektene til Welch-segmentene, samme funksjon som postprocess.py bruker
  offline: welch.window_weights. Det er hele grunnen til at de genereres herfra og
  ikke skrives to steder - device og offline vindusvekter kan ikke drive fra hverandre
  nar det bare finnes én formel.

  HVORFOR EN TABELL. Vektene avhenger bare av kWelchSegLen og kWelchWindow, som begge
  er constexpr - de er identiske for hvert segment i hver fangst. Fram til 2026-08-15
  regnet accumSegment dem ut pa nytt hver gang: {n} kall til sinf per segment, og
  segmentene kommer hvert 25.6 s. Uten FPU var det ~30 av de 117 ms hele FFT-en tok,
  og de 117 ms er den lengste stripen i fangstlakka der FIFO-en ikke leses.

  HELE VINDUET, IKKE HALVE. Bade Hann og Hamming er symmetriske om N/2, sa {n // 2 + 1}
  verdier hadde holdt - men da matte hvert oppslag i den varme lakka gjare
  i <= N/2 ? t[i] : t[N-i], og symmetrien blitt én ting til som kunne vaere feil.
  Tabellen ligger i .rodata (flash, ~52 % brukt), ikke i RAM (~84 % brukt), sa
  {n * 4} B mot {(n // 2 + 1) * 4} B er ingen avveining.

  IKKE BIT-IDENTISK med sinf. Vektene er regnet i float64 og rundet til float32 én
  gang; newlibs sinf regner hele veien i float32 og kan avvike med 1 ULP. psdAcc_ blir
  altsa lik til ~1e-7 relativt, ikke bit for bit. Fysisk er det uten betydning - hs og
  tz er integraler over {n // 2 + 1} bins - og tabellen ligger naermere referansen enn
  koden gjorde far.

  Normaliseringen sum(w^2) genereres MED VILJE IKKE herfra. accumSegment summerer
  tabellen selv (ensureS2), sa den kan ikke komme i utakt med vektene den narmaliserer
  - og det er ett sted mindre der Python og C kunne ha regnet ut litt forskjellig ting.
*/

// Lengden og vindustypen er bakt inn i verdiene. Endres en av dem uten a regenerere,
// blir spekteret stille feil - derfor et gjerde, ikke en kommentar.
inline constexpr uint16_t kWelchWindowTableLen  = {n};
inline constexpr uint8_t  kWelchWindowTableKind = {kind};   // WindowType::{args.window.capitalize()}

static_assert(kWelchWindowTableLen == kWelchSegLen,
              "kWelchSegLen er endret - regenerer welch_window.h med "
              "tools/gen_welch_window.py --seglen <n>");
static_assert(kWelchWindowTableKind == (uint8_t)kWelchWindow,
              "kWelchWindow er endret - regenerer welch_window.h med "
              "tools/gen_welch_window.py --window <hann|hamming>");

{fmt_table("kWelchWindowTable", w)}

#endif  // WELCH_WINDOW_H
"""

    if args.out == "-":
        sys.stdout.write(body)
    else:
        with open(args.out, "w") as f:
            f.write(body)
        print(f"skrev {args.out}: {n} vekter, vindu {args.window}, "
              f"sum(w^2) = {s2_f32:.9g}")


if __name__ == "__main__":
    main()
