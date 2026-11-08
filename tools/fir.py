"""FIR lowpass + decimation, mirroring the firmware's FIR stages (and the sfy buoy's src/fir.rs)."""

import numpy as np

NTAP = 31                   # number of taps, as in fir.rs. MUST be odd - see below
CUTOFF = None                # None => half the output rate (fs_out/2), as in fir.rs
COMPENSATE_DELAY = True      # compensate the group delay (the firmware does not)


WINDOWS = ("hamming", "hann", "blackman")


def sinc_window(ntap, window):
    """Symmetric window weights w[0..N-1], as scipy.signal.get_window(window, N, fftbins=False)."""
    n = np.arange(ntap, dtype=np.float64)
    x = 2.0 * np.pi * n / (ntap - 1)
    if window == "hamming":
        return 0.54 - 0.46 * np.cos(x)
    if window == "hann":
        return 0.5 - 0.5 * np.cos(x)
    if window == "blackman":
        return 0.42 - 0.5 * np.cos(x) + 0.08 * np.cos(2.0 * x)
    raise ValueError(f"FIR: unknown window {window!r} (one of {WINDOWS})")


def mean_ntap(decim):
    """Tap count of the mean over one output period: D taps for an odd D, D+1 for an
    even D (half weight at the ends) so the length stays odd. None for a fractional D."""
    d = int(round(decim))
    if abs(decim - d) > 1e-9 or d < 1:
        return None
    return d if d % 2 else d + 1


def mean_taps(decim):
    """Moving mean over one output period, as a symmetric odd-length FIR (see mean_ntap)."""
    n = mean_ntap(decim)
    if n is None:
        raise ValueError(f"FIR: the mean needs a whole decimation factor (got {decim})")
    h = np.ones(n, dtype=np.float64)
    if n != int(round(decim)):
        h[0] = h[-1] = 0.5
    return h / float(np.sum(h))


def firwin_lowpass(ntap, cutoff, fs, window="hamming"):
    """Windowed sinc lowpass, equal to scipy.signal.firwin(ntap, cutoff, fs=fs, window=window)
    with scale=True (scipy is optional here):

      h[n] = sinc(2*(fc/fs)*(n - (N-1)/2)) * w[n],  normalised so sum(h) = 1

    cutoff = fs/2 gives the identity filter (D = 1)."""
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
    h *= sinc_window(ntap, window)
    return h / float(np.sum(h))


def fir_filter_centered(x, coeffs, compensate=True):
    """Run the FIR over x and centre the result on x's own time axis.

    Returns (y, first_valid, last_valid); [first_valid, last_valid] is the range with no
    zero padding. compensate=False gives the firmware behaviour (causal, lagging (N-1)/2)."""
    c = np.asarray(coeffs, dtype=np.float64)
    x = np.asarray(x, dtype=np.float64)
    h = (len(c) - 1) // 2 if compensate else 0
    full = np.convolve(x, c)                 # len = len(x) + len(c) - 1
    y = full[h:h + len(x)]
    # The causal index n+h is clean only once the whole window lies inside x:
    #   n + h >= len(c) - 1  and  n + h <= len(x) - 1
    return y, max(0, len(c) - 1 - h), len(x) - 1 - h


def raw_uniform_grid(t_ms, raw_dt_ms=None):
    """Place the rows on a uniform raw grid, inserting and flagging missing rows.

    raw_dt_ms (kWindowMs from cfg.csv) is preferred over the median row spacing unless
    the two disagree a lot.

    Returns (n_idx, nfull, dt_ms, gap_raw), where n_idx is each row's place in nfull."""
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
    """FIR lowpass + decimation of all series, evaluated at the middle of each bucket.

    t_ms       = win_start_ms per row (raw rate)
    series     = {name: raw-rate array}
    bucket_idx = bucket indices to deliver (after gap filling)
    bucket_ms  = bucket length in ms (the output period)
    fs_out     = output rate in Hz; default cutoff is fs_out/2
    unwrap     = names of angle series to np.unwrap before filtering

    Returns (out, gap, stats). gap is 1 where the filter support touches filled-in rows
    or the series edges, so welch_psd can discard those segments."""
    n_idx, nfull, dt_ms, gap_raw = raw_uniform_grid(t_ms, raw_dt_ms)
    fs_raw = 1000.0 / dt_ms
    dec = int(round(bucket_ms / dt_ms))
    if dec < 1:
        raise ValueError(f"FIR: the raw rate ({fs_raw:.1f} Hz) is lower than the output "
                         f"({fs_out:g} Hz) - lower --decimate-hz")
    # Bucket boundaries must fall on whole rows (firmware: static_assert
    # kRowOdrHz % kWelchInputOdrHz == 0). Only warn here, the logging rate varies.
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
