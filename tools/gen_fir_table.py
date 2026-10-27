#!/usr/bin/env python3
"""Generate fir_coeffs.h for the firmware - FIR coefficient tables as constexpr arrays.

The firmware cannot compute the coefficients itself (no sinc/cos in constexpr, and a
runtime setup would burn RAM on something constant), so they go into flash as generated
tables. The point of generating them from here is PARITY: the same firwin_lowpass that
postprocess.py uses offline, so the device and the offline chain filter with exactly the
same numbers.

THE TABLES ARE INDEXED BY DECIMATION FACTOR, NOT BY RATE PAIR. That follows from the
formula:

    h[n] = sinc(2*(fc/fs)*(n - (N-1)/2)) * hamming[n],  normalised so sum(h) = 1

fc and fs appear only as the ratio fc/fs, and both decimation stages in the firmware
have fc = fs_out/2. So fc/fs = 1/(2*D) where D = fs_in/fs_out, and the table depends on
D alone: 960->100 and 480->50 give BIT-IDENTICAL taps. One generated file therefore
covers every rate pair whose D is in the list below, and the rates can be changed in
wave_config.h without regenerating anything as long as the ratio is in here.

The keys are integers = round(10*D), since D can be 9.6:

    D = 1.2 / 2.4 / 4.8   -> stage 1 at a low ODR
    D = 9.6               -> stage 1 at 960->100 AND 480->50
    D = 19.2              -> stage 1 at 960->50
    D = 5 / 10            -> stage 2 at 50 / 100 Hz rows

Usage:
  python gen_fir_table.py \\
      -o ../firmware/common_libraries/wave_manager/src/fir_coeffs.h

Add a new rate combination by extending --decim, a tap count with --ntap; the
firmware's static_assert says which combination is missing.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fir

# kImuOdrHz in {120,240,480,960} against kOutputRateHz in {50,100} gives stage 1;
# kOutputRateHz against kVaccFsHz = 10 gives stage 2. Unused tables are dropped by the
# linker (-fdata-sections + --gc-sections), so the list costs nothing to keep generous.
DEFAULT_DECIM = [1.2, 2.4, 4.0, 4.8, 5.0, 6.0, 8.0, 9.6, 10.0, 12.0, 16.0, 19.2]

# Sinc tap counts. The mean has its own (fir.mean_ntap).
DEFAULT_NTAP = [31, 63, 129]

# --kind value -> FirKind enumerator in fir.h. Must follow that enum.
KINDS = {"hamming": "Hamming", "hann": "Hann", "blackman": "Blackman", "mean": "Mean"}


def decim_key(d):
    """Integer key = 10*D. Only exact tenths are legal, otherwise two different Ds
    could collide in the same name without anyone noticing."""
    k = int(round(10.0 * d))
    if abs(10.0 * d - k) > 1e-9:
        sys.exit(f"decimation factor {d!r} is not a whole number of tenths")
    if k < 10:
        sys.exit(f"decimation factor {d!r} < 1 puts the cutoff above Nyquist")
    return k


def lit(v):
    """One float32 literal, with the decimal point forced in, because 0f is not valid C++."""
    t = f"{np.float32(v):.9g}"
    if not any(c in t for c in ".en"):
        t += ".0"
    return t + "f"


def fmt_table(name, comment, coeffs, per_line=4):
    """%.9g is short enough to read and long enough to round back to exactly the same
    float32."""
    out = [f"// {comment}",
           f"inline constexpr float {name}[{len(coeffs)}] = {{"]
    for i in range(0, len(coeffs), per_line):
        chunk = coeffs[i:i + per_line]
        out.append("    " + " ".join(f"{lit(v)}," for v in chunk))
    out.append("};")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--decim", type=float, nargs="+", default=DEFAULT_DECIM,
                    help="the decimation factors to build tables for "
                         f"(default {DEFAULT_DECIM})")
    ap.add_argument("--ntap", type=int, nargs="+", default=DEFAULT_NTAP,
                    help=f"sinc tap counts (default {DEFAULT_NTAP}); the mean sets its own")
    ap.add_argument("--kind", nargs="+", default=list(KINDS), choices=list(KINDS),
                    help=f"filter kinds (default {list(KINDS)})")
    ap.add_argument("-o", "--out", default="-", help="output file, '-' for stdout")
    args = ap.parse_args()

    keys = sorted({decim_key(d) for d in args.decim})

    tables = []   # (kind, ntap, key, comment, taps)
    for kind in args.kind:
        for k in keys:
            d = k / 10.0
            if kind == "mean":
                n = fir.mean_ntap(d)
                if n is None:
                    continue   # no whole-sample mean for a fractional D
                variants = [(n, f"D = {d:g}: mean_taps({d:g}), {n} taps", fir.mean_taps(d))]
            else:
                # fc = 0.5, fs = D => fc/fs = 1/(2D)
                variants = [(n, f"D = {d:g}: firwin_lowpass({n}, 0.5, {d:g}, {kind!r}) "
                                f"=> fc/fs = {0.5 / d:.9g}",
                             fir.firwin_lowpass(n, 0.5, d, kind))
                            for n in sorted(set(args.ntap))]
            for n, comment, h in variants:
                # firwin_lowpass normalises to sum(h) = 1 in float64, but we store float32.
                # The deviation has to be small enough that the DC gain is still 1 within
                # the noise - otherwise a constant accel bias would be scaled by the filter.
                s = float(np.sum(np.float32(h).astype(np.float64)))
                if abs(s - 1.0) > 1e-5:
                    sys.exit(f"{kind} N = {n} D = {d:g}: the float32 sum {s!r} deviates too far from 1.0")
                tables.append((kind, n, k, comment, h))

    def tname(kind, n, k):
        return f"kFirTaps_{KINDS[kind]}_N{n}_D{k}"

    # A chain of ternary operators: the constexpr function has to be a single
    # expression to be evaluable in a static_assert.
    selector = "\n".join(
        f"  {'return' if i == 0 else '     :'} kind == FirKind::{KINDS[kind]:<8} && ntap == {n:3d} && k == {k:3d} ? {tname(kind, n, k)}"
        for i, (kind, n, k, _, _) in enumerate(tables))
    selector += "\n       :            nullptr;"

    body = f"""#ifndef FIR_COEFFS_H
#define FIR_COEFFS_H

#include "fir.h"   // FirKind

/*
  GENERATED by tools/gen_fir_table.py - do not edit by hand.

  Windowed-sinc lowpass (Hamming / Hann / Blackman, normalised so sum(h) = 1), and the
  mean over one output period. Sum = 1 gives exactly unity gain at DC, so any residual
  accel bias is not scaled.

  ONE TABLE PER (KIND, TAP COUNT, DECIMATION FACTOR). The coefficients depend only on
  fc/fs, and since both stages use fc = fs_out/2 that is 1/(2*D) - so 960->100 and
  480->50 are the SAME table.

  The D suffix is 10x the decimation factor, since D can be 9.6:
  kFirTaps_Hamming_N31_D96 = D 9.6, kFirTaps_Hamming_N31_D100 = D 10.0.

  The tables live in flash; the linker drops the unused ones
*/

"""

    for kind, n, k, comment, h in tables:
        body += fmt_table(tname(kind, n, k), comment, h) + "\n\n"

    body += f"""// Select a table by kind, tap count and decimation factor x 10. Returns nullptr for a
// combination with no table, so analysis_config.h can stop the build with a message
// naming what is missing rather than filtering with the wrong coefficients.
constexpr const float *firTaps(FirKind kind, uint16_t ntap, uint16_t k) {{
{selector}
}}

#endif  // FIR_COEFFS_H
"""

    if args.out == "-":
        sys.stdout.write(body)
    else:
        with open(args.out, "w") as f:
            f.write(body)
        print(f"wrote {args.out}: {len(tables)} tables, kinds {', '.join(args.kind)}, "
              f"sinc N = {', '.join(str(n) for n in sorted(set(args.ntap)))}, "
              f"D = {', '.join(f'{k / 10:g}' for k in keys)}")


if __name__ == "__main__":
    main()
