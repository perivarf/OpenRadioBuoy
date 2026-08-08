"""Welch-PSD med segmentforkasting - samme segmentering og normalisering som
accumSegment i analysis.cpp.

Én-sidig PSD = Σ_seg scale·|X_k|² / (fs·Σw²), snittet over segmentene.
Σw²-normaliseringen (ikke N²) er det som gjør at et vindusvektet estimat får
riktig absoluttnivå: uten den ville Hann-vinduet alene senket m0 med faktor ~2.67.

Samplingsraten sendes inn som argument. Den lå tidligere som en modulglobal i
postprocess.py, noe som betydde at --decimate-hz måtte skrive om en global før
noen kalte hit. Nå er koblingen eksplisitt, og fila kan testes uten å importere
resten av analysen.
"""

import numpy as np


def window_weights(n, kind):
    """Vindusvekter w[0..n-1] (i/N-indeksering, som firmware)."""
    i = np.arange(n, dtype=np.float64)
    if kind == "hamming":
        return 0.54 - 0.46 * np.cos(2.0 * np.pi * i / n)
    s = np.sin(np.pi * i / n)          # Hann = sin²(πi/N)
    return s * s


def detrend_segment(seg, mode):
    """Fjern middel eller lineær trend fra ett segment før vindusvekting.

    Firmware bruker "none". "linear" er default i offline-analysen fordi ÷ω⁴ gjør
    en udetrendet drift til en falsk lavfrekvenstopp: Hann håndterer ren DC, men
    ikke en trend, og det er nettopp trender ÷ω⁴ straffer hardest."""
    if mode == "none":
        return seg
    if mode == "mean":
        return seg - float(np.mean(seg))
    x = np.arange(len(seg), dtype=np.float64)
    dx = x - x.mean()
    ym = float(np.mean(seg))
    slope = float(np.dot(dx, seg - ym) / np.dot(dx, dx))
    return seg - (ym + slope * dx)


def welch_psd(series, seglen, overlap_div, win, fs, rejects=None, detrend="linear"):
    """Streaming-Welch over series, med fs = seriens samplingsrate [Hz].

    detrend = "none" | "mean" | "linear". Detrendingen skjer FØR vindusvekting,
    som er standard.

    rejects = liste av (navn, maske, terskel), der maske er 0/1 per sample med
    samme lengde som series. Et segment hoppes over hvis andelen flaggede samples
    i skiven > terskel (terskel 0.0 => enhver forekomst forkaster). Serien røres
    ALDRI - vi lar bare være å akkumulere det forurensede segmentet, så timingen
    på alle andre skiver er intakt (de er posisjons-indekserte biter av samme
    array). Et segment som bryter flere kriterier telles på det FØRSTE i lista,
    så rekkefølgen der bestemmer hvilken årsak som rapporteres.

    Returnerer (psd[0..N/2], antall segmenter, dict {navn: antall forkastet})."""
    n = seglen
    nbins = n // 2 + 1
    psd = np.zeros(nbins, dtype=np.float64)
    crit = [(nm, np.asarray(mk, dtype=np.float64), th)
            for nm, mk, th in (rejects or []) if mk is not None and th is not None]
    nrej = {nm: 0 for nm, _, _ in crit}
    if len(series) < n:
        return psd, 0, nrej
    gs2 = float(np.sum(win * win))
    step = n // overlap_div
    scale = np.full(nbins, 2.0)
    scale[0] = 1.0
    scale[-1] = 1.0            # k=0 og Nyquist uten faktor 2
    nseg = 0
    start = 0
    x = np.asarray(series, dtype=np.float64)
    while start + n <= len(x):
        hit = None
        for nm, mk, th in crit:
            if float(np.mean(mk[start:start + n])) > th:
                hit = nm
                break
        if hit is not None:
            nrej[hit] += 1
            start += step
            continue                # forurenset segment -> ikke med i snittet
        seg = detrend_segment(x[start:start + n], detrend) * win
        mag2 = np.abs(np.fft.rfft(seg)) ** 2
        psd += scale * mag2 / (fs * gs2)
        nseg += 1
        start += step
    if nseg > 0:
        psd /= nseg
    return psd, nseg, nrej
