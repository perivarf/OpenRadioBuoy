#include "wave_analysis.h"

#include <math.h>
#include <string.h>

#include "welch_window.h"  // kWelchWindowTable - generated, see the header for why

/*
  Contains the wave chain (FFT / Welch / spectral moments) plus
  the second decimation stage for imu (ingest). 
*/

// -----------------------------------------------------------------------------
// FFT + Welch
// -----------------------------------------------------------------------------
static float  gRe[kWelchSegLen], gIm[kWelchSegLen];
static double gS2 = 0.0;
static bool   gS2Ready = false;

// Iterative radix-2 Cooley-Tukey FFT (in-place), forward. n = power of two.
static void fft(float *re, float *im, int n) {
  for (int i = 1, j = 0; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float tr = re[i]; re[i] = re[j]; re[j] = tr;
      float ti = im[i]; im[i] = im[j]; im[j] = ti;
    }
  }
  for (int len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * (float)M_PI / (float)len;
    float wr = cosf(ang), wi = sinf(ang);
    for (int i = 0; i < n; i += len) {
      float cwr = 1.0f, cwi = 0.0f;
      for (int k = 0; k < len / 2; k++) {
        int a = i + k, b = i + k + len / 2;
        float vr = re[b] * cwr - im[b] * cwi;
        float vi = re[b] * cwi + im[b] * cwr;
        re[b] = re[a] - vr; im[b] = im[a] - vi;
        re[a] += vr;        im[a] += vi;
        float nwr = cwr * wr - cwi * wi;
        cwi = cwr * wi + cwi * wr;
        cwr = nwr;
      }
    }
  }
}

/*
  Sum of the squared window weights - the normalisation that gives a windowed estimate
  the right absolute level (without it a Hann window alone would drop m0 by ~2.67).

  Summed from kWelchWindowTable rather than generated alongside it, deliberately: it is
  then consistent with the very weights it normalises by construction, and there is no
  second place where Python and C could have computed the scale slightly differently.
  One pass of kWelchSegLen multiply-adds, once per capture, and no transcendentals.
*/
static void ensureS2() {
  if (gS2Ready) return;
  double s2 = 0.0;
  for (int i = 0; i < kWelchSegLen; i++) {
    const float w = kWelchWindowTable[i];
    s2 += (double)w * w;
  }
  gS2 = s2;
  gS2Ready = true;
}

/*
  Window one segment, FFT, accumulate one-sided PSD into psdAcc[0..N/2].

  Runs once every kWelchSegLen / kWelchOverlapDiv samples - 25.6 s at 10 Hz - and is the
  longest uninterruptible stretch in the capture loop at 88 ms. Its cost is therefore a
  FIFO question and not only a CPU one, which is why it is reached from
  processPendingSegment() in the capture loop rather than from pushWelch inside the pop
  loop: it starts with the whole FIFO depth in front of it (220 ms) instead of with up
  to kFifoWatermark - 1 words already standing (165 ms). See the ring-slack section in
  analysis_config.h.

  The window is a table lookup, not sinf per sample - see welch_window.h, including why
  the result is not bit-identical to what sinf gave.

  seg is the RING, not a flat segment, and start is where this segment begins in it. The
  wrap is a compare-and-subtract rather than a modulo: kWelchRingLen is not a power of
  two, so % would be a division per sample inside the hottest loop in the analyzer.
*/
static void accumSegment(const float *seg, uint16_t start, float *psdAcc) {
  ensureS2();
  const int N = kWelchSegLen;
  uint16_t j = start;
  for (int i = 0; i < N; i++) {
    gRe[i] = seg[j] * kWelchWindowTable[i];
    gIm[i] = 0.0f;
    if (++j == kWelchRingLen) j = 0;
  }
  fft(gRe, gIm, N);
  for (int k = 0; k <= N / 2; k++) {
    float mag2 = gRe[k] * gRe[k] + gIm[k] * gIm[k];
    float scale = (k == 0 || k == N / 2) ? 1.0f : 2.0f;
    psdAcc[k] += scale * mag2 / ((float)kWelchInputOdrHz * (float)gS2);
  }
}

// Low-frequency half-cosine taper (Kohout / Tucker & Pitt 2001).
static inline float lowFreqTaper(float f) {
  if (f <= kTaperF1) return 0.0f;
  if (f >= kTaperF2) return 1.0f;
  return 0.5f * (1.0f - cosf((float)M_PI * (f - kTaperF1) / (kTaperF2 - kTaperF1)));
}

// -----------------------------------------------------------------------------
// StreamAnalyzer
// -----------------------------------------------------------------------------
void StreamAnalyzer::begin(void) {
  fir2_.reset();
  curBucket_ = -1; bucketDone_ = false;
  n10_ = nData_ = nBrake_ = nWarm_ = 0;
  head_ = tail_ = fill_ = 0; segPending_ = false;
  nSeg_ = 0; nRingFull_ = 0;
  for (int k = 0; k <= kWelchSegLen / 2; k++) psdAcc_[k] = 0.0f;
  ensureS2();
}

// Push one sample into the ring. NO FFT from here - this runs inside the FIFO pop loop,
// and getting the 88 ms of accumSegment out of that loop is the whole point of the ring.
// A full segment only raises the flag; processPendingSegment() below does the work.
void StreamAnalyzer::pushWelch(float sample) {
  // Safety valve, not the normal path. The ring has kWelchRingSlack free slots beyond a
  // segment and a drain delivers at most 3 samples, so reaching this means the deferred
  // call never ran at all - not merely that it ran late. Running the FFT here is bad
  // (it meets a half-drained FIFO), but losing a sample is worse: the segments would no
  // longer sit 256 samples apart and the PSD would go quietly wrong. The static_assert
  // by kWelchRingSlack should catch the cause long before it gets here.
  if (fill_ == kWelchRingLen) {
    nRingFull_++;
    processPendingSegment();
  }

  ring_[head_] = sample;
  if (++head_ == kWelchRingLen) head_ = 0;
  fill_++;
  if (fill_ >= kWelchSegLen) segPending_ = true;
}

// The deferred half of pushWelch: FFT + accumulate PSD, then release one step of the
// ring (75% overlap keeps the rest). Called from the capture loop with the FIFO just
// drained - the same reasoning that moved the raw-log write and the imu.csv sync out of
// the pop loop. Nothing here is timing-sensitive on its own; what matters is only WHERE
// in the drain cycle it runs.
//
// Advancing tail_ is also what replaces a memmove of (kWelchSegLen - step) floats per
// segment: the read index moves instead of the data, so 3 kB of copying never happens.
bool StreamAnalyzer::processPendingSegment(void) {
  if (!segPending_) return false;
  segPending_ = false;

  accumSegment(ring_, tail_, psdAcc_);
  nSeg_++;

  const uint16_t step = kWelchSegLen / kWelchOverlapDiv;
  tail_ += step;
  if (tail_ >= kWelchRingLen) tail_ -= kWelchRingLen;
  fill_ -= step;
  return true;
}

void StreamAnalyzer::ingest(const ImuRow &r) {
  nData_++;
  if (r.braking) nBrake_++;

  long t = (long)r.winStartMs;

  
  const float v = isfinite(r.vaccFir) ? r.vaccFir : 0.0f;

  // Stage 2 is fed on EVERY row, warm-up included. The delay line has to be full by
  // the time the first Welch sample is taken, otherwise the spectrum opens with a
  // filter transient - so the warm-up gate below sits between the filter and Welch,
  // not in front of the filter.
  fir2_.push(v);

  // Warm-up: the AHRS has not converged yet (and the FIR is still filling), so the
  // vertical accel is biased. The rows are still logged by the caller; they are just
  // kept out of the Welch chain -> no effect on the PSD or on Hs/Tz/Tc/Tp.
  if (t < (long)wave_measurement_filter_warm_up) {
    nWarm_++;
    return;
  }

  // Decimate to kWelchInputOdrHz. Same convention as stage 1 and as fir.py's dec//2: the
  // output for a bucket is evaluated at the bucket's CENTRE, on the first row to
  // reach it, so the value sits in the middle of the interval it represents. Rows
  // can be missing (a window with no accel samples emits nothing), hence "first row
  // at or past the centre" rather than an exact timestamp match.
  long bucket = t / kWelchInputPeriodMs;
  if (bucket != curBucket_) {
    curBucket_ = bucket;
    bucketDone_ = false;
  }
  if (!bucketDone_ && t >= bucket * (long)kWelchInputPeriodMs + (long)kFirS2CenterMs) {
    float s = fir2_.eval();
    pushWelch(isfinite(s) ? s : 0.0f);
    n10_++;
    bucketDone_ = true;
  }
}

bool StreamAnalyzer::finalize(WaveParams &params, uint16_t *spectrumOut) {
  // A segment that filled on the capture's LAST drain has no capture loop left to run
  // it, so it is picked up here. This is the one choke point: psd() and segments() are
  // only read after finalize() in processReading, so no caller can see a stale nSeg_.
  // Same role as RawLogWriter::flush(true) in the stop sequence - a deferral must not
  // eat the tail. Costs 88 ms outside the capture window, where no FIFO is at risk.
  processPendingSegment();

  // No partial bucket to flush any more: a decimated sample is either evaluated at
  // its bucket centre or not at all. At worst the final bucket is dropped - one
  // sample in ~18000, far below a Welch segment.
  params = {-1.0f, -1.0f, -1.0f, -1.0f, 0.0f, 0.0, 0.0, 0.0};
  for (size_t j = 0; j < welch_bins; j++) spectrumOut[j] = 0;
  if (nSeg_ == 0) return false;

  const int N = kWelchSegLen;
  const float df = (float)kWelchInputOdrHz / N;
  const float invSeg = 1.0f / (float)nSeg_;

  // Spectral moments + peak, over the elevation PSD (acc PSD / omega^4 * taper^2).
  float peakEta = 0.0f, peakF = 0.0f;
  for (int k = 1; k <= N / 2; k++) {
    float f = k * df;
    if (f > kWaveFMax) break;
    float taper = lowFreqTaper(f);
    if (taper <= 0.0f) continue;
    float w = 2.0f * (float)M_PI * f;
    float psdEta = (psdAcc_[k] * invSeg) / (w * w * w * w) * (taper * taper);
    params.m0 += (double)psdEta * df;
    params.m2 += (double)psdEta * f * f * df;
    params.m4 += (double)psdEta * f * f * f * f * df;
    if (psdEta > peakEta) { peakEta = psdEta; peakF = f; }
  }

  if (params.m0 > 0) params.hs = 4.0f * sqrtf((float)params.m0);
  if (params.m0 > 0 && params.m2 > 0) params.tz = sqrtf((float)(params.m0 / params.m2));
  if (params.m2 > 0 && params.m4 > 0) params.tc = sqrtf((float)(params.m2 / params.m4));
  if (peakF > 0) params.tp = 1.0f / peakF;   // peak period from the spectral peak

  /*
    The spectrum sent on wire is acceleration with no
    no taper. 
  */

  // Normalisation peak: the largest unaveraged acceleration PSD bin inside the
  // transmitted range, not across the whole analysed band. 
  float peakAcc = 0.0f;
  for (size_t k = welch_bin_min == 0 ? 1 : welch_bin_min; k < welch_bin_max; k++) {
    const float p = psdAcc_[k] * invSeg;
    if (p > peakAcc) peakAcc = p;
  }
  params.maxValue = peakAcc;

  // Quantise: acceleration PSD over bins welch_bin_min..max, normalised to peakAcc
  // (shape) so it fits uint16; the absolute peak rides in params.maxValue.
  //
  // Each wire bin is the BAND AVERAGE of kSpecBinGroup consecutive PSD bins, not a
  // 1:1 copy - that is what lets kSpecNBins span the whole wave band without the
  // payload growing to match (see wave_config.h). Averaging keeps the integral:
  // sum_j Shat_j * (G*df) == sum_k S_k * df. Normalising against the UNAVERAGED
  // peakAcc is deliberate and keeps the absolute scale reconstructible on the far
  // side as (value/65535)^2 * maxValue; the cost is that no wire bin quite reaches
  // 65535, since averaging the peak bin with its neighbour lowers it.
  //
  // SQRT COMPANDING, and why it is not optional. peakAcc is the largest bin in the
  // band - but the vertical acceleration PSD of a buoy keeps climbing past 0.5 Hz on
  // chop, so with kPsdMaxFreq at 1.0 Hz the peak is set by something that is not a
  // wave, three to four decades above the wave band. Storing norm linearly spends the
  // uint16 on that peak: replaying seven Skjaerhalden captures through the wire format
  // (tools/firmware_test.py --psd-quant) put the weakest wave-band bin at 3-202 counts
  // and the worst bin error at 0.2-14 %. sqrt puts the resolution on the RELATIVE
  // value instead - the same captures give 457-3642 counts and 0.02-0.20 %, 10-72x
  // better - for one sqrtf here and one multiply on the receiver. The peak bin still
  // lands on 65535 either way, so only the flanks move.
  // kSpecNBins, not welch_bins: the array is sized to the wire-format capacity, but
  // only the first kSpecNBins entries are filled and sent. The rest stay at the zero
  // set above, so a stale tail cannot leak out if the count ever grows again.
  if (peakAcc > 0.0f) {
    for (size_t j = 0; j < kSpecNBins; j++) {
      float acc = 0.0f;
      for (size_t g = 0; g < kSpecBinGroup; g++) {
        const int k = (int)welch_bin_min + (int)(j * kSpecBinGroup + g);
        // k == 0 is DC. The segment mean is removed before the FFT, so that bin holds
        // no wave information - only whatever offset survived detrending, which must
        // not be allowed to sit in the spectrum or (via peakAcc) set its scale. Any
        // kPsdMinFreq above zero already puts welch_bin_min past it; this stays as the
        // guard for a build that sets kPsdMinFreq to 0 and asks for the whole band.
        if (k == 0) continue;
        const float psd = psdAcc_[k] * invSeg;
        if (psd > 0.0f) acc += psd;
      }
      float norm = (acc / (float)kSpecBinGroup) / peakAcc;
      if (norm > 1.0f) norm = 1.0f;
      spectrumOut[j] = (uint16_t)lroundf(sqrtf(norm) * 65535.0f);
    }
  }
  return true;
}
