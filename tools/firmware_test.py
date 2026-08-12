#!/usr/bin/env python3
"""Speil av drifterens bølgekjede - kjør firmwarens algoritme mot en opptaksmappe.

HVORFOR DENNE FINNES, ved siden av postprocess.py: postprocess er forskningsverktøyet.
Den kjører fire AHRS-er, analyserer til 5 Hz, bruker seglen 2048, taper 0,15-0,30 og
fs 20 Hz. Firmware kjører ETT filter, stopper på 1,0 Hz, bruker seglen 1024, taper
0,03-0,05 og fs 10 Hz. Tallene fra de to er derfor ikke sammenlignbare, og det har
ikke finnes noe som svarer på "regner brettet riktig?". Det gjør denne: hver konstant
under er firmwarens, så utgangen kan holdes rett mot device sin egen ana.csv.

HVA DEN FAKTISK TESTER. imu.csv ligger på radraten (100 Hz), altså NEDENFOR trinn 1 i
firmwarekjeden. Det som kan replayes er derfor:

    vacc per rad -> FIR-desimering -> Welch -> taper -> momenter -> Hs/Tz/Tc/Tp
                                                     -> baandmidlet spektrum-slice

Det som IKKE kan replayes er alt over radgrensen: FIFO-drift, vindusdriften, AHRS på
960 Hz, QuatDelay og trinn-1-FIR-en. De krever råsamples som ikke finnes i fila.
FirDecimator og QuatDelay er dekket av host-testen på C++-koden i stedet.

    Med en fangst tatt i WaveLogMode::Raw/Both finnes de råsamplene likevel, i
    <stamp>_raw.bin. Bruk firmware_test_raw.py: den rekonstruerer radene fra
    råloggen (AHRS på råstrømmen) og mater dem inn i NØYAKTIG denne fila.

Merk også at dette speiler ALGORITMEN, ikke C++-koden. En indeksfeil i firmware kan
ikke fanges her - bare i at tallene spriker.

DESIMERINGSKJEDEN er konfigurerbar (--rates). Firmware kjører i praksis 100 -> 10 på
denne siden av radgrensen, men en kaskade som 100 -> 50 -> 10 kan settes opp for å se
hva et ekstra trinn gjør. Hvert trinn er et Hamming-sinc-lavpass med cutoff = fs_ut/2,
KAUSALT (som firmware, compensate=0), evaluert i bøttesenteret (fir.py sin dec//2).

Bruk:
  python firmware_test.py <mappe>
  python firmware_test.py <mappe> --ahrs kalman --rates 100 50 10
  python firmware_test.py <mappe> --ahrs sflp --rates 100 10 --seglen 512
"""

import argparse
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fir                                    # noqa: E402
import welch as welchmod                      # noqa: E402
import rotation                               # noqa: E402
from madgwick import Madgwick                 # noqa: E402
from kalman import Kalman                     # noqa: E402
from mekf import Mekf                         # noqa: E402
import postprocess as pp                      # noqa: E402

# ---------------------------------------------------------------------------
# Firmwarekonstanter. Speiler wave_manager/src/wave_config.h - endres de der, må de
# endres her, og det er hele poenget med fila: den skal ikke ha egne meninger.
# ---------------------------------------------------------------------------
GRAVITY        = 9.80665        # kGravity
MG2MS2         = GRAVITY / 1000.0
MDPS2RADS      = 1.0e-3 * math.pi / 180.0
MADGWICK_BETA  = 0.05           # kMadgwickBeta
FIR_NTAP       = 129            # kFirNtap
OUTPUT_RATE_HZ = 100            # kOutputRateHz - radraten i imu.csv
VACC_FS_HZ     = 10.0           # kVaccFsHz
SEGLEN         = 1024           # kWelchSegLen
OVERLAP_DIV    = 4              # kWelchOverlapDiv
WINDOW         = "hann"         # kWelchWindow
DETREND        = "none"         # firmware detrender ikke
WAVE_FMAX      = 1.0            # kWaveFMax
TAPER_F1       = 0.03           # kTaperF1
TAPER_F2       = 0.05           # kTaperF2
WARMUP_MS      = 30000          # wave_measurement_filter_warm_up
PSD_MIN_FREQ   = 0.03           # kPsdMinFreq - nedre båndkant; resten utledes
PSD_MAX_FREQ   = 1.0            # kPsdMaxFreq - øvre båndkant; resten utledes
WELCH_BINS     = 102            # welch_bins (common_config.h) - wire-format KAPASITET
# welch_bin_min / kSpecBinGroup / kSpecNBins / welch_bin_max / kSpecBandMaxHz er UTLEDEDE
# i firmware og skal aldri stå som literaler her. resolve_spec_bins() nedenfor leser dem
# fra opptakets cfg.csv når de finnes, og faller tilbake på å speile utledningen ellers.
SPEC_BIN_MIN   = None           # welch_bin_min (inklusiv)
SPEC_BIN_GROUP = None           # kSpecBinGroup
SPEC_N_BINS    = None           # kSpecNBins - faktisk antall sendte bins, <= WELCH_BINS
SPEC_BIN_MAX   = None           # welch_bin_max (eksklusiv)
SPEC_BAND_MIN  = None           # kSpecBandMinHz - nedre KANT, ikke bin-senter
SPEC_BAND_MAX  = None           # kSpecBandMaxHz


def derive_spec_bins(seglen):
    """Speil utledningen i wave_config.h: (bin_min, group, nbins, bin_max, band_max).

    Speiler utledningen, ikke resultatet. WELCH_BINS er wire-formatets KAPASITET, ikke
    antallet som må sendes - num_bins ligger i meldinga og spekteret er siste felt, så
    færre bins gir bare en kortere melding. Derfor: nedre kant rundet OPP til hel bin
    (ingen sendt bin stikker under PSD_MIN_FREQ), så minste gruppe som presser resten av
    båndet inn i kapasiteten, og deretter så mange hele grupper som får plass UNDER
    PSD_MAX_FREQ (gulv, aldri over). Int-trunkeringen matcher (size_t)-casten."""
    df = VACC_FS_HZ / seglen                                  # kPsdDfHz
    psd_max_bin = int(PSD_MAX_FREQ / df)                      # kPsdMaxBin
    bin_min = int(PSD_MIN_FREQ / df)                          # kPsdMinBin, avrundet opp
    if bin_min * df < PSD_MIN_FREQ:
        bin_min += 1
    group   = max(1, -(-(psd_max_bin - bin_min) // WELCH_BINS))
    nbins   = (psd_max_bin - bin_min) // group
    bin_max = bin_min + nbins * group
    return bin_min, group, nbins, bin_max, bin_max * df


def resolve_spec_bins(cfg, seglen, quiet=False):
    """Sett SPEC_BIN_*-globalene for dette opptaket.

    cfg.csv er fasit når nøklene finnes: den beskriver spekteret device FAKTISK sendte,
    og å utlede tallene på nytt her ville bare vaere et andre sted å regne feil (og et
    sted som ikke vet hvilken welch_bins den builden hadde). Utledningen er fallback for
    gamle opptak uten nøklene, og samtidig kontrollen som sier fra naar firmware har
    flyttet seg siden opptaket."""
    global SPEC_BIN_MIN, SPEC_BIN_GROUP, SPEC_N_BINS, SPEC_BIN_MAX
    global SPEC_BAND_MIN, SPEC_BAND_MAX
    mine = derive_spec_bins(seglen)
    if "spec_bin_group" in cfg and "welch_bin_max" in cfg:
        group   = int(float(cfg["spec_bin_group"]))
        bin_max = int(float(cfg["welch_bin_max"]))
        # welch_bin_min har vært logget hele veien, men var 0 fram til den nedre
        # båndkanten kom - default 0 er derfor riktig for et opptak uten nøkkelen.
        bin_min = int(float(cfg.get("welch_bin_min", 0)))
        # spec_n_bins kom sammen med det variable antallet; før det var welch_bin_max
        # alltid nøyaktig welch_bins grupper, så antallet kan utledes for gamle opptak.
        nbins   = int(float(cfg["spec_n_bins"])) if "spec_n_bins" in cfg \
                  else (bin_max - bin_min) // group
        SPEC_BIN_MIN, SPEC_BIN_GROUP, SPEC_N_BINS, SPEC_BIN_MAX = bin_min, group, nbins, bin_max
        SPEC_BAND_MIN = bin_min * (VACC_FS_HZ / seglen)
        SPEC_BAND_MAX = bin_max * (VACC_FS_HZ / seglen)
        if not quiet and (bin_min, group, nbins, bin_max) != mine[:4]:
            print(f"    [!] spec-bins: device bin_min {bin_min}/group {group}/{nbins} bins/"
                  f"bin_max {bin_max}, dagens PSD_MIN_FREQ={PSD_MIN_FREQ} "
                  f"PSD_MAX_FREQ={PSD_MAX_FREQ} gir {mine[0]}/{mine[1]}/{mine[2]}/{mine[3]} - "
                  f"bruker device sine, det er dem spekteret ble laget med")
    else:
        SPEC_BIN_MIN, SPEC_BIN_GROUP, SPEC_N_BINS, SPEC_BIN_MAX, SPEC_BAND_MAX = mine
        SPEC_BAND_MIN = SPEC_BIN_MIN * (VACC_FS_HZ / seglen)


def low_freq_taper(f):
    """Halvcosinus-taper, identisk med lowFreqTaper() i wave_analysis.cpp."""
    f = np.asarray(f, dtype=np.float64)
    t = 0.5 * (1.0 - np.cos(math.pi * (f - TAPER_F1) / (TAPER_F2 - TAPER_F1)))
    return np.where(f <= TAPER_F1, 0.0, np.where(f >= TAPER_F2, 1.0, t))


def uniform_grid(t_ms, vals, dt_ms):
    """Legg radene på et eksakt uniformt rutenett. np.convolve forutsetter jevn
    sampling; firmware slipper unna fordi den ser hver rad når den kommer. Hull
    (vindu uten accel-samples) interpoleres lineært og telles."""
    t = np.asarray(t_ms, dtype=np.float64)
    idx = np.rint((t - t[0]) / dt_ms).astype(np.int64)
    n = int(idx[-1]) + 1
    out = np.full(n, np.nan)
    out[idx] = vals
    miss = int(np.count_nonzero(np.isnan(out)))
    if miss:
        good = ~np.isnan(out)
        out = np.interp(np.arange(n), np.flatnonzero(good), out[good])
    return out, miss


def mean_stage(x, fs_in, fs_out):
    """Boxcar-bøttemiddel - det build 1/64 gjorde før FIR-en. Ligger her for at
    verktøyet skal kunne reprodusere et GAMMELT opptak eksakt, og for å måle hva
    antialias-byttet faktisk flyttet."""
    dec = int(round(fs_in / fs_out))
    n = (len(x) // dec) * dec
    return np.asarray(x[:n], dtype=np.float64).reshape(-1, dec).mean(axis=1), dec, 0.0


def fir_stage(x, fs_in, fs_out, ntap):
    """Ett desimeringstrinn, nøyaktig som FirDecimator + evalueringen i firmware.

    push() på hvert inn-sample og eval() i bøttesenteret blir offline til: kausal
    konvolusjon over hele serien, så plukk indeksene dec//2, dec//2+dec, ...
    Kausal (ikke fasekompensert) fordi firmware er det - utgangen ligger derfor
    (ntap-1)/2 inn-samples etter."""
    dec = int(round(fs_in / fs_out))
    if abs(fs_in / fs_out - dec) > 1e-9:
        sys.exit(f"FIR: {fs_in} -> {fs_out} Hz er ikke heltallsdesimering")
    h = fir.firwin_lowpass(ntap, 0.5 * fs_out, fs_in)
    y = np.convolve(np.asarray(x, dtype=np.float64), h)[:len(x)]   # kausal
    idx = np.arange(dec // 2, len(x), dec)
    delay_s = ((ntap - 1) // 2) / fs_in
    return y[idx], dec, delay_s


def wave_params(psd, fs, seglen):
    """Momenter og bølgeparametre, linje for linje som finalize() i
    wave_analysis.cpp: start på k=1, stopp over kWaveFMax, hopp over taper<=0.

    max_value er IKKE elevasjonstoppen: det er toppen i akselerasjons-PSD-en over de
    sendte binsene, altså skalaen wire-spekteret normaliseres mot. Elevasjonstoppen
    ligger igjen som peak_eta, siden det er den som gir Tp."""
    df = fs / seglen
    m0 = m2 = m4 = 0.0
    peak_eta = 0.0
    peak_f = 0.0
    eta = np.zeros(len(psd))
    for k in range(1, seglen // 2 + 1):
        f = k * df
        if f > WAVE_FMAX:
            break
        taper = float(low_freq_taper(f))
        if taper <= 0.0:
            continue
        w = 2.0 * math.pi * f
        psd_eta = psd[k] / (w ** 4) * (taper ** 2)
        eta[k] = psd_eta
        m0 += psd_eta * df
        m2 += psd_eta * f * f * df
        m4 += psd_eta * f ** 4 * df
        if psd_eta > peak_eta:
            peak_eta, peak_f = psd_eta, f

    # Normaliseringstoppen: største UMIDLEDE akselerasjonsbin innenfor det sendte
    # båndet, ikke over hele analysebåndet. Toppen må ligge blant binsene som faktisk
    # sendes, ellers skaleres hele wire-spekteret ned av noe mottakeren ikke ser.
    peak_acc = 0.0
    for k in range(max(SPEC_BIN_MIN, 1), SPEC_BIN_MAX):
        if psd[k] > peak_acc:
            peak_acc = float(psd[k])

    return {
        "Hs": 4.0 * math.sqrt(m0) if m0 > 0 else -1.0,
        "Tz": math.sqrt(m0 / m2) if m0 > 0 and m2 > 0 else -1.0,
        "Tc": math.sqrt(m2 / m4) if m2 > 0 and m4 > 0 else -1.0,
        "Tp": 1.0 / peak_f if peak_f > 0 else -1.0,
        "max_value": peak_acc, "peak_eta": peak_eta,
        "m0": m0, "m2": m2, "m4": m4, "eta": eta,
    }


def spectrum_slice(psd, peak_acc, fs, seglen):
    """Det sendte spekteret: AKSELERASJONS-PSD-en (ingen omega^4, ingen taper),
    båndmidlet SPEC_BIN_GROUP PSD-bins per wire-bin og normalisert mot den UMIDLEDE
    toppen, som finalize() gjør."""
    out = np.zeros(SPEC_N_BINS)
    if peak_acc <= 0:
        return out
    for j in range(SPEC_N_BINS):
        acc = 0.0
        for g in range(SPEC_BIN_GROUP):
            k = SPEC_BIN_MIN + j * SPEC_BIN_GROUP + g
            if k == 0:
                continue                       # DC: middelet er trukket fra, bærer ingenting
            acc += max(float(psd[k]), 0.0)
        out[j] = min((acc / SPEC_BIN_GROUP) / peak_acc, 1.0)
    return out


def attach_device_vacc(imu_path, rows):
    """Hent device sin egen vacc-kolonne inn paa radene. read_imu_rows leser den
    ikke - postprocess har aldri brukt den - men den er fasiten naar spoersmaalet
    er om ANALYSEKJEDEN regner riktig, uavhengig av AHRS-replayet."""
    with open(imu_path) as f:
        hdr = f.readline().rstrip("\n").split(",")
        idx = {n: i for i, n in enumerate(hdr)}
        # vacc_fir (den FILTRERTE serien) beholdt navnet sitt gjennom omdøpingen ved
        # build_seq 3, så den er førstevalget uansett filversjon. Fallbacken er den
        # UFILTRERTE serien, og den heter ulikt før/etter - derav oppslaget.
        col = idx.get("vacc_fir", idx.get(pp.imu_col_names(idx)["vacc"]))
        if col is None:
            sys.exit("--ahrs device: fant verken vacc_fir eller den ufiltrerte "
                     "vacc-kolonnen (vacc / vacc_madgwick) i imu.csv")
        for r in rows:
            line = f.readline().rstrip("\n").split(",")
            if len(line) < len(hdr):
                break
            r["vacc_dev"] = float(line[col])
    return rows


def vacc_series(rows, method):
    """Vertikal lineær akselerasjon per rad (m/s^2).

    AHRS-en kjøres på RADRATEN her - det er alt CSV-en gir. Firmware kjører den på
    råstrømmen ved kAhrsRateHz, så attityden vil avvike noe; det er en kjent og
    uunngåelig forskjell, ikke en feil i noen av dem."""
    if method == "device":
        # Device sin EGEN vacc-kolonne, ikke gjenberegnet. Isolerer analysekjeden
        # fra AHRS-replayet: stemmer Hs her, er alt nedenfor vacc verifisert, og en
        # gjenvaerende forskjell er attityden alene.
        return np.array([r.get("vacc_dev", 0.0) for r in rows], dtype=np.float64)

    if method == "sflp":
        # On-chip-fusjonen er allerede gravitasjonskompensert i az_ned.
        return np.array([r["azn"] * MG2MS2 for r in rows], dtype=np.float64)

    filt = {"madgwick": lambda: Madgwick(beta=MADGWICK_BETA),
            "kalman":   Kalman,
            "mekf":     Mekf}[method]()
    out = np.empty(len(rows), dtype=np.float64)
    prev_t = None
    for i, r in enumerate(rows):
        ax, ay, az = r["ax"] * MG2MS2, r["ay"] * MG2MS2, r["az"] * MG2MS2
        if prev_t is None:
            filt.init_from_accel(ax, ay, az)   # samme seeding som ImuSampler
        else:
            dt = (r["t"] - prev_t) / 1000.0
            if dt > 0:
                filt.update(r["gx"] * MDPS2RADS, r["gy"] * MDPS2RADS,
                            r["gz"] * MDPS2RADS, ax, ay, az, dt)
        prev_t = r["t"]
        out[i] = rotation.world_z(filt.q, ax, ay, az) - GRAVITY
    return np.nan_to_num(out, nan=0.0, posinf=0.0, neginf=0.0)


def apply_device_cfg(cfg):
    """Adopter opptakets EGNE analysekonstanter fra cfg.csv.

    Uten dette sammenlignes eplene i denne fila (dagens firmware) med device sine
    pærer (den builden opptaket ble gjort med). De gamle Skjaerhalden-opptakene
    kjørte wave_fmax 0,5 Hz; å holde dagens 1,0 Hz mot deres Hs sier ingenting."""
    global WAVE_FMAX, TAPER_F1, TAPER_F2, SEGLEN, PSD_MIN_FREQ, PSD_MAX_FREQ
    got = {}
    for key, name in (("wave_fmax_hz", "WAVE_FMAX"), ("taper_f1_hz", "TAPER_F1"),
                      ("taper_f2_hz", "TAPER_F2"), ("psd_min_freq_hz", "PSD_MIN_FREQ"),
                      ("psd_max_freq_hz", "PSD_MAX_FREQ")):
        if key in cfg:
            globals()[name] = float(cfg[key])
            got[key] = cfg[key]
    if "welch_seglen" in cfg:
        SEGLEN = int(float(cfg["welch_seglen"]))
        got["welch_seglen"] = cfg["welch_seglen"]
    # SPEC_BIN_* trengs ikke her: resolve_spec_bins() leser dem fra cfg.csv uansett,
    # ikke bare under --match-device. PSD_MIN/MAX_FREQ over betyr bare noe for fallbacken.
    return got


def row_rate_from_cfg(cfg):
    """Radraten opptaket faktisk ble logget med. Den er IKKE alltid 100 Hz - de
    gamle 240 Hz-ODR-opptakene ligger på 50 - og en feil antagelse her gir en
    desimeringsfaktor som er dobbelt så stor som den skal, uten at noe klager."""
    if "output_rate_hz" in cfg:
        return float(cfg["output_rate_hz"])
    if "window_ms" in cfg:
        return 1000.0 / float(cfg["window_ms"])
    return None


def run(directory, method, rates, seglen, ntap, match_device=False,
        detrend=DETREND, decimate='fir', quiet=False):
    stamps = sorted(glob.glob(os.path.join(directory, "*_imu.csv")))
    if not stamps:
        sys.exit(f"fant ingen *_imu.csv i {directory}")
    imu_path = stamps[0]
    stamp = os.path.basename(imu_path)[:-len("_imu.csv")]
    cfg = pp.read_kv(os.path.join(directory, f"{stamp}_cfg.csv"))

    from_device_cfg = apply_device_cfg(cfg) if match_device else {}
    if match_device and seglen is None:
        seglen = SEGLEN
    if seglen is None:
        seglen = SEGLEN

    # Må skje etter at seglen er avgjort: bin-indeksene betyr ingenting uten den df-en.
    resolve_spec_bins(cfg, seglen, quiet=quiet)

    # Radraten kommer fra opptaket, ikke fra en antagelse.
    fs_cfg = row_rate_from_cfg(cfg)
    if rates is None:
        out_hz = VACC_FS_HZ
        if match_device and "vacc_bucket_ms" in cfg:
            out_hz = 1000.0 / float(cfg["vacc_bucket_ms"])
        rates = [fs_cfg if fs_cfg else OUTPUT_RATE_HZ, out_hz]
    elif fs_cfg and abs(rates[0] - fs_cfg) > 1e-6:
        print(f"  [!] --rates starter paa {rates[0]} Hz, men cfg.csv sier {fs_cfg} Hz")

    rows = pp.read_imu_rows(imu_path)
    if not rows:
        sys.exit(f"ingen rader i {imu_path}")
    if method == "device":
        attach_device_vacc(imu_path, rows)

    dt_ms = 1000.0 / rates[0]
    vacc = vacc_series(rows, method)
    t_ms = np.array([r["t"] for r in rows], dtype=np.float64)
    series, nfilled = uniform_grid(t_ms, vacc, dt_ms)

    # --- desimeringskjeden ---
    stages = []
    fs = float(rates[0])
    total_delay = 0.0
    for fs_out in rates[1:]:
        if decimate == 'mean':
            series, dec, delay = mean_stage(series, fs, fs_out)
        else:
            series, dec, delay = fir_stage(series, fs, fs_out, ntap)
        total_delay += delay
        stages.append((fs, fs_out, dec, delay, len(series)))
        fs = float(fs_out)

    # --- warm-up: firmware mater filteret på hver rad, men holder rader under
    # warm-up ute av Welch. Offline er det ekvivalent med å kutte i den desimerte
    # serien, siden hele serien har vært gjennom filteret.
    n_skip = int(math.ceil(WARMUP_MS / 1000.0 * fs))
    analysed = series[n_skip:]

    win = welchmod.window_weights(seglen, WINDOW)
    psd, nseg, _ = welchmod.welch_psd(analysed, seglen, OVERLAP_DIV, win, fs,
                                      detrend=detrend)
    if nseg == 0:
        sys.exit(f"for kort serie: {len(analysed)} samples < seglen {seglen}")

    p = wave_params(psd, fs, seglen)
    spec = spectrum_slice(psd, p["max_value"], fs, seglen)

    if quiet:
        return p

    df = fs / seglen
    print(f"\n=== {stamp} ===")
    print(f"  {len(rows)} rader fra {imu_path.split(os.sep)[-1]}"
          + (f"   ({nfilled} hull interpolert)" if nfilled else ""))
    print(f"  AHRS: {method}   (paa radraten {rates[0]} Hz - firmware kjoerer den paa raastroemmen)")

    print("\n  desimeringskjede:")
    for fs_in, fs_out, dec, delay, n in stages:
        if decimate == "mean":
            print(f"    {fs_in:>6.1f} -> {fs_out:>5.1f} Hz   dec {dec:>3}   BOXCAR-middel"
                  f" (som build 1/64)   -> {n} samples")
        else:
            h = fir.firwin_lowpass(ntap, 0.5 * fs_out, fs_in)
            g1 = abs(np.sum(h * np.exp(-2j * np.pi * 1.0 * np.arange(ntap) / fs_in)))
            print(f"    {fs_in:>6.1f} -> {fs_out:>5.1f} Hz   dec {dec:>3}   cutoff {0.5*fs_out:>5.2f} Hz"
                  f"   {ntap} tap   lag {delay*1000:>6.1f} ms   |H(1Hz)| = {g1:.4f}   -> {n} samples")
    print(f"    samlet kausalt etterslep: {total_delay*1000:.1f} ms")

    print(f"\n  Welch: seglen {seglen}, df {df:.6f} Hz, T_seg {seglen/fs:.1f} s, "
          f"{OVERLAP_DIV-1}/{OVERLAP_DIV} overlapp, {WINDOW}, detrend {detrend}")
    print(f"    {nseg} segmenter   ({n_skip} samples droppet i warm-up)")

    print("\n  BOELGEPARAMETRE")
    print(f"    Hs (SWH) = {p['Hs']:8.3f} m")
    print(f"    Tz       = {p['Tz']:8.2f} s")
    print(f"    Tc       = {p['Tc']:8.2f} s")
    print(f"    Tp       = {p['Tp']:8.2f} s")
    print(f"    m0/m2/m4 = {p['m0']:.6e} / {p['m2']:.6e} / {p['m4']:.6e}")
    print(f"    maxValue = {p['max_value']:.6e}  (topp psd_acc i sendt bånd, (m/s^2)^2/Hz)")
    print(f"    peak_eta = {p['peak_eta']:.6e}  (topp psd_eta, gir Tp)")

    # --- device sin egen ana.csv, hvis den finnes ---
    ana = pp.read_kv(os.path.join(directory, f"{stamp}_ana.csv"))
    if ana:
        print("\n  MOT DEVICE ana.csv")
        if from_device_cfg:
            print(f"    [konstanter hentet fra opptakets cfg.csv: {from_device_cfg}]")
        mism = [(k, cfg[k], v) for k, v in
                (("wave_fmax_hz", WAVE_FMAX), ("taper_f1_hz", TAPER_F1),
                 ("taper_f2_hz", TAPER_F2), ("welch_seglen", float(seglen)))
                if k in cfg and abs(float(cfg[k]) - v) > 1e-6]
        for k, dev, mine in mism:
            print(f"    [!] {k}: device {dev}, her {mine} - Hs/Tz/Tc er ikke "
                  f"sammenlignbare (bruk --match-device)")
        for key, mine in (("Hs", p["Hs"]), ("Tz", p["Tz"]), ("Tc", p["Tc"]), ("Tp", p["Tp"])):
            dev = ana.get(key, ana.get(f"{key}_madgwick"))
            if dev is None:
                continue
            dev = float(dev)
            d = 100.0 * (mine / dev - 1.0) if dev else float("nan")
            print(f"    {key:<3} device {dev:8.3f}   her {mine:8.3f}   avvik {d:+7.2f} %")

    print(f"\n  sendt spektrum: {len(spec)} bins a {SPEC_BIN_GROUP*df:.6f} Hz, "
          f"{SPEC_BAND_MIN:.4f}-{SPEC_BAND_MAX:.4f} Hz, topp i bin {int(np.argmax(spec))}")
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("directory", help="opptaksmappe med <stamp>_imu.csv")
    ap.add_argument("--ahrs", default="madgwick",
                    choices=["madgwick", "kalman", "mekf", "sflp", "device"],
                    help="orienteringsfilter (default madgwick, som firmware)")
    ap.add_argument("--rates", type=float, nargs="+", default=None,
                    help="desimeringskjede i Hz, f.eks. '100 10' eller '100 50 10'. "
                         "Utelatt: radraten fra cfg.csv -> 10 Hz")
    ap.add_argument("--seglen", type=int, default=None)
    ap.add_argument("--ntap", type=int, default=FIR_NTAP)
    ap.add_argument("--detrend", default=DETREND, choices=["none", "mean", "linear"],
                    help="firmware bruker 'none'. 'linear' viser hvor mye av Hs som "
                         "er drift forsterket av 1/omega^4, ikke boelger")
    ap.add_argument("--decimate", default="fir", choices=["fir", "mean"],
                    help="'fir' = dagens firmware. 'mean' = boxcar-boettemiddel, som "
                         "build 1/64 - bruk den for aa reprodusere et gammelt opptak")
    ap.add_argument("--match-device", action="store_true",
                    help="adopter opptakets egne wave_fmax/taper/seglen/bucket fra "
                         "cfg.csv, saa tallene kan holdes mot device sin ana.csv")
    args = ap.parse_args()

    if args.rates is not None and len(args.rates) < 2:
        sys.exit("--rates trenger minst inn- og utrate, f.eks. '100 10'")
    run(args.directory, args.ahrs, args.rates, args.seglen, args.ntap,
        match_device=args.match_device, detrend=args.detrend,
        decimate=args.decimate)


if __name__ == "__main__":
    main()
