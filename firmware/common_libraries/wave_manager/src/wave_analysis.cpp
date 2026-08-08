#include "wave_analysis.h"

#include <math.h>
#include <string.h>

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

static inline float windowWeight(int i) {
  const float N = (float)kWelchSegLen;
  if (kWelchWindow == WindowType::Hamming) {
    return 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i / N);
  } else {
    float s = sinf((float)M_PI * i / N);  // Hann = sin^2(pi i / N)
    return s * s;
  }
}

static void ensureS2() {
  if (gS2Ready) return;
  double s2 = 0.0;
  for (int i = 0; i < kWelchSegLen; i++) { float w = windowWeight(i); s2 += (double)w * w; }
  gS2 = s2;
  gS2Ready = true;
}

// Window one segment, FFT, accumulate one-sided PSD into psdAcc[0..N/2].
static void accumSegment(const float *seg, float *psdAcc) {
  ensureS2();
  const int N = kWelchSegLen;
  for (int i = 0; i < N; i++) { gRe[i] = seg[i] * windowWeight(i); gIm[i] = 0.0f; }
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
  segFill_ = 0; nSeg_ = 0;
  for (int k = 0; k <= kWelchSegLen / 2; k++) psdAcc_[k] = 0.0f;
  ensureS2();
}

// Push one sample into the segment; on a full segment FFT + accumulate PSD
// and keep the last (N - step) samples (75% overlap).
void StreamAnalyzer::pushWelch(float sample) {
  segBuf_[segFill_++] = sample;
  if (segFill_ >= kWelchSegLen) {
    accumSegment(segBuf_, psdAcc_);
    nSeg_++;
    const int step = kWelchSegLen / kWelchOverlapDiv;
    const int keep = kWelchSegLen - step;
    memmove(segBuf_, segBuf_ + step, keep * sizeof(float));
    segFill_ = keep;
  }
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
  params.maxValue = peakEta;

  // Quantise the transmitted spectrum: elevation PSD over bins welch_bin_min..max,
  // normalised to the peak (shape) so it fits uint16; the absolute peak rides in
  // params.maxValue.
  //
  // Each wire bin is the BAND AVERAGE of kSpecBinGroup consecutive PSD bins, not a
  // 1:1 copy - that is what lets kSpecNBins span the whole wave band without the
  // payload growing to match (see wave_config.h). Averaging keeps the integral:
  // sum_j Shat_j * (G*df) == sum_k S_k * df. Normalising against the UNAVERAGED
  // peakEta is deliberate and keeps the absolute scale reconstructible on the far
  // side as value/65535 * maxValue; the cost is that no wire bin quite reaches
  // 65535, since averaging the peak bin with its neighbour lowers it.
  // kSpecNBins, not welch_bins: the array is sized to the wire-format capacity, but
  // only the first kSpecNBins entries are filled and sent. The rest stay at the zero
  // set above, so a stale tail cannot leak out if the count ever grows again.
  if (peakEta > 0.0f) {
    for (size_t j = 0; j < kSpecNBins; j++) {
      float acc = 0.0f;
      for (size_t g = 0; g < kSpecBinGroup; g++) {
        const int k = (int)welch_bin_min + (int)(j * kSpecBinGroup + g);
        // k == 0 is DC: f = 0 makes omega^4 zero and the elevation PSD undefined.
        // The moment loop above sidesteps it by starting at k = 1; this loop has to
        // do the same now that welch_bin_min reaches down to 0. The bin is zero by
        // construction anyway - lowFreqTaper kills everything below kTaperF1.
        if (k == 0) continue;
        const float f = k * df;
        const float taper = lowFreqTaper(f);
        const float w = 2.0f * (float)M_PI * f;
        const float psdEta = (psdAcc_[k] * invSeg) / (w * w * w * w) * (taper * taper);
        if (psdEta > 0.0f) acc += psdEta;
      }
      float norm = (acc / (float)kSpecBinGroup) / peakEta;
      if (norm > 1.0f) norm = 1.0f;
      spectrumOut[j] = (uint16_t)lroundf(norm * 65535.0f);
    }
  }
  return true;
}
