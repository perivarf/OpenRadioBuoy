"""FIR lowpass + decimation - mirrors the sfy buoy's src/fir.rs.

The alternative to the firmware's bucket mean. A mean over D samples IS a FIR filter,
but a poor one: the amplitude response |sin(pi*f*D/fs)/(D*sin(pi*f/fs))| has nulls at
multiples of the output rate and leaks heavily between them, so everything above
Nyquist folds down into the analysis band. A windowed sinc filter attenuates what is
meant to go away.
"""

import numpy as np

NTAP = 31                   # number of taps, as in fir.rs. MUST be odd - see below
CUTOFF = None                # None => half the output rate (fs_out/2), as in fir.rs
COMPENSATE_DELAY = True      # compensate the group delay (the firmware does not)


def firwin_lowpass(ntap, cutoff, fs):
    """Windowed sinc lowpass - identical to scipy.signal.firwin(ntap, cutoff, fs=fs)
    with the default choices (Hamming window, scale=True), but written out by hand
    because scipy is an OPTIONAL dependency here:

      h[n] = sinc(2*(fc/fs)*(n - (N-1)/2)) * w_hamming[n],  normalised so sum(h) = 1

    The normalisation gives exactly unity gain at DC, which is what keeps a filtered
    mean level (a residual accel bias, say) from being scaled.

    Note the limiting case cutoff = fs/2: sinc(n - M) then becomes a pure delta, and
    the filter is exactly the identity. That is precisely the right behaviour when
    nothing is to be decimated (D = 1)."""
    if ntap < 3:
        raise ValueError(f"FIR: ntap must be >= 3 (got {ntap})")
    if ntap % 2 == 0:
        # Odd length => the group delay (N-1)/2 is a WHOLE number of samples and can be
        # compensated exactly by indexing.
        raise ValueError(f"FIR: ntap must be odd for exact phase compensation (got {ntap})")
    if not (0.0 < cutoff <= 0.5 * fs):
        raise ValueError(f"FIR: cutoff must lie in (0, fs/2) = (0, {0.5 * fs:g}) Hz (got {cutoff})")
    n = np.arange(ntap, dtype=np.float64)
    h = np.sinc(2.0 * (cutoff / fs) * (n - 0.5 * (ntap - 1)))
    h *= 0.54 - 0.46 * np.cos(2.0 * np.pi * n / (ntap - 1))   # Hamming (symmetric)
    return h / float(np.sum(h))


def fir_filter_centered(x, coeffs, compensate=True):
    """Run the FIR over x and put the result back on x's OWN time axis.

    Returns (y, first_valid, last_valid), where y[n] is the filter value CENTRED on
    x[n] and [first_valid, last_valid] is the range where no zero padding has
    contributed. Linear phase gives a delay of h = (N-1)/2 samples, so the centred
    value at n is the causal value at n+h - hence just an index shift.

    compensate=False gives the firmware behaviour (causal, lagging by h samples)."""
    c = np.asarray(coeffs, dtype=np.float64)
    x = np.asarray(x, dtype=np.float64)
    h = (len(c) - 1) // 2 if compensate else 0
    full = np.convolve(x, c)                 # len = len(x) + len(c) - 1
    y = full[h:h + len(x)]
    # The causal index n+h is clean only once the whole window lies inside x:
    #   n + h >= len(c) - 1  and  n + h <= len(x) - 1
    return y, max(0, len(c) - 1 - h), len(x) - 1 - h


def raw_uniform_grid(t_ms, raw_dt_ms=None):
    """Place the rows on an EXACTLY uniform raw grid.

    The FIR assumes a constant sample rate; rows can be missing (FIFO overflow, a
    blocking SD flush). We find the grid from the row spacing, insert the missing
    indices and flag them.

    raw_dt_ms = kWindowMs from cfg.csv when it is there, and it is the ground truth:
    the median dt is only an estimate and misses if enough rows are gone. If the two
    diverge a lot, the directory is mispaired or the timestamps are broken - and then
    we trust what the rows show.

    Returns (n_idx, nfull, dt_ms, gap_raw), where n_idx is each row's place in
    nfull."""
    t = np.asarray(t_ms, dtype=np.float64)
    if len(t) < 2:
        raise ValueError("FIR: needs at least two rows")
    d = np.diff(t)
    dt = float(np.median(d))
    if dt <= 0.0:
        raise ValueError(f"FIR: invalid raw dt ({dt} ms)")
    # A row period that is not a whole number of ms (120 Hz is 8.333) reaches the CSV as
    # win_start_ms labels stepping 8,8,9: the MEDIAN picks 8 and puts the rate 4 % out,
    # while the MEAN recovers 8.333 exactly. Diffs far above the median are FIFO gaps,
    # not the pattern, and are what the mean has to be protected from - hence the
    # median first, then the mean over what it says is a normal step.
    near = d[d <= 1.5 * dt]
    if len(near) > 0:
        dt = float(np.mean(near))
    if raw_dt_ms:
        if abs(float(raw_dt_ms) - dt) <= 0.25 * dt:
            dt = float(raw_dt_ms)
        else:
            print(f"  WARNING: cfg.csv window_ms={float(raw_dt_ms):g} ms, but "
                  f"the rows show {dt:g} ms - using {dt:g} ms")
    n_idx = np.rint((t - t[0]) / dt).astype(np.int64)
    nfull = np.arange(n_idx[-1] + 1, dtype=np.int64)
    gap_raw = np.ones(len(nfull), dtype=np.float64)
    gap_raw[n_idx] = 0.0
    return n_idx, nfull, dt, gap_raw


def fir_decimate_series(t_ms, series, bucket_idx, bucket_ms, fs_out,
                        ntap=NTAP, cutoff=CUTOFF, unwrap=(),
                        compensate=COMPENSATE_DELAY, raw_dt_ms=None):
    """Replace the bucket mean with a FIR lowpass + decimation, on ALL the series.

    t_ms       = win_start_ms per row (raw rate)
    series     = {name: raw-rate array}
    bucket_idx = the bucket indices the series is to be delivered on (after gap filling)
    bucket_ms  = the bucket length in ms (the output period)
    fs_out     = the output rate in Hz, used as the default cutoff (fs_out/2)
    unwrap     = names that are ANGLES and must be np.unwrap-ed before filtering (roll
                 and pitch from atan2 jump by 2*pi, and a jump would smear itself
                 across the whole filter support)

    All the series get EXACTLY the same treatment - a requirement when series are to
    be compared bin by bin afterwards. The output for bucket b is taken at the MIDDLE
    of the bucket, so that it sits at the same instant as the bucket mean it replaces.

    Returns (out, gap, stats). gap is 1 for buckets where the filter support
    (+/-(N-1)/2 samples) touches filled-in rows or reaches outside the series - those
    are not real data and are ORed into the gap mask, so welch_psd can discard the
    segments."""
    n_idx, nfull, dt_ms, gap_raw = raw_uniform_grid(t_ms, raw_dt_ms)
    fs_raw = 1000.0 / dt_ms
    dec = int(round(bucket_ms / dt_ms))
    if dec < 1:
        raise ValueError(f"FIR: the raw rate ({fs_raw:.1f} Hz) is lower than the output "
                         f"({fs_out:g} Hz) - lower --decimate-hz")
    # The bucket boundaries sit on whole bucket_ms. If that does not divide the row
    # period, they fall in the middle of a row and the round() above gives a decimation
    # that slides relative to the time axis. The firmware makes this a static_assert
    # (kRowOdrHz % kWelchInputOdrHz == 0 - a whole number of ROWS per bucket, which is
    # the same condition stated in rates rather than periods, and stays checkable when
    # the row period is not a whole number of ms); here we cannot abort, since the
    # logging rate varies from session to session.
    if abs(bucket_ms - dec * dt_ms) > 1e-6:
        print(f"  WARNING: the bucket {bucket_ms} ms does not divide the row period "
              f"{dt_ms:g} ms - the decimation (D={dec}) slides against the time axis")
    fc = float(cutoff) if cutoff is not None else 0.5 * fs_out
    coeffs = firwin_lowpass(ntap, fc, fs_raw)
    half = (ntap - 1) // 2

    xf = nfull.astype(np.float64)
    xb = n_idx.astype(np.float64)
    out = {}
    for name, v in series.items():
        v = np.asarray(v, dtype=np.float64)
        if name in unwrap:
            v = np.unwrap(v)
        # Interpolate up onto the uniform grid (the identity when no rows are missing).
        vu = v if len(nfull) == len(v) else np.interp(xf, xb, v)
        out[name], first_ok, last_ok = fir_filter_centered(vu, coeffs, compensate)

    # The bucket centres in raw indices: bucket b starts at time b*bucket_ms.
    b = np.asarray(bucket_idx, dtype=np.float64)
    m = np.rint((b * bucket_ms - float(t_ms[0])) / dt_ms).astype(np.int64) + dec // 2
    inside = (m >= first_ok) & (m <= last_ok)
    mc = np.clip(m, 0, len(nfull) - 1)

    # A gap smears out over the whole filter support, so the flag is dilated to match.
    gap_dil = (np.convolve(gap_raw, np.ones(ntap), mode="same") > 0.0).astype(np.float64)
    gap = np.maximum(gap_dil[mc], (~inside).astype(np.float64))

    out = {k: v[mc] for k, v in out.items()}
    stats = dict(ntap=ntap, cutoff=fc, fs_raw=fs_raw, dec=dec, delay_s=half / fs_raw,
                 compensate=bool(compensate), n_raw=len(nfull),
                 n_raw_filled=int(np.count_nonzero(gap_raw)),
                 n_edge=int(np.count_nonzero(~inside)),
                 n_flagged=int(np.count_nonzero(gap > 0.0)), n_out=len(b))
    return out, gap, stats


def fir_response(f, coeffs, fs_raw):
    """|H(f)| for the FIR (the DTFT of the coefficients)."""
    f = np.atleast_1d(np.asarray(f, dtype=np.float64))
    n = np.arange(len(coeffs))
    return np.abs(np.exp(-2j * np.pi * np.outer(f / fs_raw, n)) @ np.asarray(coeffs))
