"""FIR-lavpass + desimering - speiler sfy-bøyas src/fir.rs.

Alternativet til firmwares bøtte-middel. Et middel over D samples ER et FIR-filter,
men et dårlig et: amplituderesponsen |sin(πfD/fs)/(D·sin(πf/fs))| har null i
multipla av utgangsraten og lekker kraftig mellom dem, så alt over Nyquist brettes
ned i analysebåndet. Et vindusbasert sinc-filter demper det som skal bort.

Ratene sendes inn som argumenter (bucket_ms, fs_out). De lå tidligere som
modulglobaler i postprocess.py, slik at --decimate-hz måtte skrive om en global
før noen kalte hit. Nå er koblingen eksplisitt.
"""

import numpy as np

NTAP = 129                   # antall tap, som fir.rs. MÅ være oddetall - se under
CUTOFF = None                # None => halve utgangsraten (fs_out/2), som fir.rs
COMPENSATE_DELAY = True      # kompenser gruppeforsinkelsen (firmware gjør ikke det)


def firwin_lowpass(ntap, cutoff, fs):
    """Vindusbasert sinc-lavpass - identisk med scipy.signal.firwin(ntap, cutoff,
    fs=fs) med standardvalgene (Hamming-vindu, scale=True).

    Skrevet ut for hånd fordi scipy er en VALGFRI avhengighet her (kun
    --cutoff auto bruker den), og fordi den eksplisitte formelen viser nøyaktig
    hva firmware har liggende ferdig utregnet:

      h[n] = sinc(2·(fc/fs)·(n − (N−1)/2)) · w_hamming[n],  normalisert så Σh = 1

    Normaliseringen gir nøyaktig enhetsforsterkning ved DC, som er det som gjør at
    et filtrert middelnivå (f.eks. en gjenværende accel-bias) ikke skaleres.

    Merk grensetilfellet cutoff = fs/2: da blir sinc(n − M) en ren delta, og
    filteret er eksakt identitet. Det er nettopp riktig oppførsel når det ikke
    skal desimeres (D = 1)."""
    if ntap < 3:
        raise ValueError(f"FIR: ntap må være >= 3 (fikk {ntap})")
    if ntap % 2 == 0:
        # Odde lengde => gruppeforsinkelsen (N−1)/2 er et HELT antall samples og kan
        # kompenseres eksakt ved indeksering. Med like lengde blir den et halvt
        # sample, og da måtte serien reinterpoleres for å ligge på samme tidsakse.
        raise ValueError(f"FIR: ntap må være oddetall for eksakt fasekompensasjon (fikk {ntap})")
    if not (0.0 < cutoff <= 0.5 * fs):
        raise ValueError(f"FIR: cutoff må ligge i (0, fs/2) = (0, {0.5 * fs:g}) Hz (fikk {cutoff})")
    n = np.arange(ntap, dtype=np.float64)
    h = np.sinc(2.0 * (cutoff / fs) * (n - 0.5 * (ntap - 1)))
    h *= 0.54 - 0.46 * np.cos(2.0 * np.pi * n / (ntap - 1))   # Hamming (symmetrisk)
    return h / float(np.sum(h))


def fir_filter_centered(x, coeffs, compensate=True):
    """Kjør FIR-en over x og legg resultatet tilbake på x' EGEN tidsakse.

    Returnerer (y, first_valid, last_valid), der y[n] er filterverdien SENTRERT på
    x[n] og [first_valid, last_valid] er området der ingen null-utfylling har
    bidratt. Lineærfase gir forsinkelsen h = (N−1)/2 samples, så den sentrerte
    verdien i n er den kausale verdien i n+h - derfor bare en indeksforskyvning.

    compensate=False gir firmware-oppførselen (kausal, etterslep på h samples)."""
    c = np.asarray(coeffs, dtype=np.float64)
    x = np.asarray(x, dtype=np.float64)
    h = (len(c) - 1) // 2 if compensate else 0
    full = np.convolve(x, c)                 # len = len(x) + len(c) − 1
    y = full[h:h + len(x)]
    # Kausal indeks n+h er ren først når hele vinduet ligger inne i x:
    #   n + h >= len(c) − 1  og  n + h <= len(x) − 1
    return y, max(0, len(c) - 1 - h), len(x) - 1 - h


def raw_uniform_grid(t_ms, raw_dt_ms=None):
    """Legg radene på et EKSAKT uniformt rå-rutenett.

    FIR-en antar konstant samplerate; radene kan mangle (FIFO-overflow, blokkerende
    SD-flush). Vi finner rutenettet fra median-dt, setter inn de manglende
    indeksene og flagger dem.

    raw_dt_ms = kWindowMs fra cfg.csv når den finnes. Den er fasit: median-dt er
    bare et estimat og bommer hvis nok rader mangler. Spriker de mye, er mappa
    feilparet eller tidsstemplene ødelagte - da stoler vi på det radene viser.

    Returnerer (n_idx, nfull, dt_ms, gap_raw), der n_idx er hver rads plass i
    nfull."""
    t = np.asarray(t_ms, dtype=np.float64)
    if len(t) < 2:
        raise ValueError("FIR: trenger minst to rader")
    dt = float(np.median(np.diff(t)))
    if dt <= 0.0:
        raise ValueError(f"FIR: ugyldig rå-dt ({dt} ms)")
    if raw_dt_ms:
        if abs(float(raw_dt_ms) - dt) <= 0.25 * dt:
            dt = float(raw_dt_ms)
        else:
            print(f"  ADVARSEL: cfg.csv window_ms={float(raw_dt_ms):g} ms, men "
                  f"radene viser {dt:g} ms - bruker {dt:g} ms")
    n_idx = np.rint((t - t[0]) / dt).astype(np.int64)
    nfull = np.arange(n_idx[-1] + 1, dtype=np.int64)
    gap_raw = np.ones(len(nfull), dtype=np.float64)
    gap_raw[n_idx] = 0.0
    return n_idx, nfull, dt, gap_raw


def fir_decimate_series(t_ms, series, bucket_idx, bucket_ms, fs_out,
                        ntap=NTAP, cutoff=CUTOFF, unwrap=(),
                        compensate=COMPENSATE_DELAY, raw_dt_ms=None):
    """Bytt ut bøtte-midlet med FIR-lavpass + desimering, på ALLE seriene.

    t_ms       = win_start_ms per rad (rå rate)
    series     = {navn: rå-rate array}
    bucket_idx = bøtte-indeksene serien skal leveres på (etter hull-fylling)
    bucket_ms  = bøttelengden i ms (utgangsperioden)
    fs_out     = utgangsraten i Hz, brukt som default cutoff (fs_out/2)
    unwrap     = navn som er VINKLER og må np.unwrap-es før filtrering (roll/pitch
                 fra atan2 hopper 2π, og et hopp ville smurt seg utover hele
                 filterstøtten)

    Alle seriene får NØYAKTIG samme behandling - det er et krav når serier skal
    sammenlignes bin for bin etterpå.

    Utgangen for bøtte b hentes i MIDTEN av bøtta, slik at den ligger på samme
    tidspunkt som bøtte-midlet den erstatter - se bucket_ms/2-forskyvningen.

    Returnerer (out, gap, stats). gap er 1 for bøtter der filterstøtten (±(N−1)/2
    samples) berører utfylte rader eller stikker utenfor serien - de er ikke ekte
    data og ORes inn i hull-masken, så welch_psd kan forkaste segmentene."""
    n_idx, nfull, dt_ms, gap_raw = raw_uniform_grid(t_ms, raw_dt_ms)
    fs_raw = 1000.0 / dt_ms
    dec = int(round(bucket_ms / dt_ms))
    if dec < 1:
        raise ValueError(f"FIR: rå-raten ({fs_raw:.1f} Hz) er lavere enn utgangen "
                         f"({fs_out:g} Hz) - senk --decimate-hz")
    # Bøttegrensene ligger på hele bucket_ms. Går ikke det opp i radperioden, faller
    # de midt i en rad, og round() over gir en desimering som glir i forhold til
    # tidsaksen. Firmware har samme krav som static_assert (kVacc10HzBucketMs %
    # kWindowMs == 0); her kan vi ikke avbryte, siden loggeraten varierer per økt.
    if abs(bucket_ms - dec * dt_ms) > 1e-6:
        print(f"  ADVARSEL: bøtte {bucket_ms} ms går ikke opp i radperioden "
              f"{dt_ms:g} ms - desimeringen (D={dec}) glir mot tidsaksen")
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
        # Interpolér opp på det uniforme rutenettet (identitet når ingen rader mangler).
        vu = v if len(nfull) == len(v) else np.interp(xf, xb, v)
        out[name], first_ok, last_ok = fir_filter_centered(vu, coeffs, compensate)

    # Bøttesentrene i rå-indekser: bøtte b starter ved tid b·bucket_ms.
    b = np.asarray(bucket_idx, dtype=np.float64)
    m = np.rint((b * bucket_ms - float(t_ms[0])) / dt_ms).astype(np.int64) + dec // 2
    inside = (m >= first_ok) & (m <= last_ok)
    mc = np.clip(m, 0, len(nfull) - 1)

    # Hull smøres utover hele filterstøtten, så flagget må dilateres tilsvarende.
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
    """|H(f)| for FIR-en (DTFT av koeffisientene).

    Ingen kaller denne i dag - den står igjen fordi den er den naturlige måten å
    dokumentere antialias-filteret på i en figur."""
    f = np.atleast_1d(np.asarray(f, dtype=np.float64))
    n = np.arange(len(coeffs))
    return np.abs(np.exp(-2j * np.pi * np.outer(f / fs_raw, n)) @ np.asarray(coeffs))
