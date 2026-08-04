#include "wave_analysis.h"

#include <math.h>
#include <string.h>

/*
  Ported from ORB_test/src/analysis.cpp, reduced to a single on-device orientation
  method to fit RAM - selected at compile time at the bottom of wave_config.h. The
  offline postprocess still recomputes all three methods from the raw imu.csv.

  What is left here is the wave chain itself (FFT / Welch / spectral moments) plus
  the few lines in ingest() that turn a row into a vertical acceleration. The
  filters themselves live next door and stay free of Arduino and of this file's
  units: madgwick.{h,cpp}, kalman.{h,cpp} and rotation.{h,cpp} (the quaternion
  primitives they share).
*/

// -----------------------------------------------------------------------------
// FFT + Welch (file-local, ported from analysis.cpp). Single shared scratch.
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
    psdAcc[k] += scale * mag2 / (kVaccFsHz * (float)gS2);
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
  ahrs_.reset();
  haveT_ = false; prevT_ = 0;
  curBucket_ = -1; bSum_ = 0.0; bN_ = 0;
  n10_ = nData_ = nBrake_ = nWarm_ = 0;
  segFill_ = 0; nSeg_ = 0;
  for (int k = 0; k <= kWelchSegLen / 2; k++) psdAcc_[k] = 0.0f;
  ensureS2();
}

// Push one 10 Hz sample into the segment; on a full segment FFT + accumulate PSD
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

void StreamAnalyzer::ingest(ImuRow &r) {
  nData_++;
  if (r.braking) nBrake_++;

  // Interval since the previous row; 0 on the first row of the capture, which is
  // the cue to seed the attitude from gravity instead of integrating.
  long t = (long)r.winStartMs;
  float dt = haveT_ ? (float)(t - prevT_) / 1000.0f : 0.0f;
  const bool firstRow = !haveT_;
  prevT_ = t;
  haveT_ = true;

  // Orientation -> vertical accel (m/s^2). ImuRow carries accel in mg and gyro in
  // mdps, so the unit conversions live here; the AHRS takes SI (m/s^2 and rad/s).
  // The accel unit is not cosmetic: KalmanAhrs' adaptive R weighs |a| against
  // gravity, so in mg every sample would look like a 100 g slam and R would stay
  // pinned high. Madgwick normalises and is scale-free, so it sees no difference -
  // which is why this stays filter-agnostic. Only the selected branch is compiled.
  const float ax = r.ax * kMg2Ms2;
  const float ay = r.ay * kMg2Ms2;
  const float az = r.az * kMg2Ms2;

  float qSel[4];
  float vSel;
  if constexpr (wave_use_sflp) {
    qSel[0] = r.qw; qSel[1] = r.qx; qSel[2] = r.qy; qSel[3] = r.qz;
    vSel = r.azn * kMg2Ms2;          // already gravity-compensated (world/NED Z)
  } else {
    if (firstRow) {
      ahrs_.initFromAccel(ax, ay, az);
    } else if (dt > 0.0f) {
      ahrs_.update(r.gx * kMdps2Rads, r.gy * kMdps2Rads, r.gz * kMdps2Rads,
                   ax, ay, az, dt);
    }
    const float *q = ahrs_.quaternion();
    qSel[0] = q[0]; qSel[1] = q[1]; qSel[2] = q[2]; qSel[3] = q[3];
    vSel = verticalAccel(qSel, ax, ay, az, kGravity);
  }

  // SFLP is logged alongside the selected method regardless of which one runs, so
  // the offline postprocess has the on-chip reference in every imu.csv.
  float vSflp = r.azn * kMg2Ms2;

  if (!isfinite(vSel)) vSel = 0.0f;
  if (!isfinite(vSflp)) vSflp = 0.0f;

  // Write orientation result back into the row -> logged to imu.csv.
  r.mqw = qSel[0]; r.mqx = qSel[1]; r.mqy = qSel[2]; r.mqz = qSel[3];
  r.vaccMadgwick = vSel; r.vaccSflp = vSflp;

  // AHRS warm-up: the filter above has been updated (and the row is logged by the
  // caller), but the orientation has not converged yet, so the vertical accel is
  // biased. Keep those rows out of the bucketing/Welch chain -> no effect on the
  // PSD or on Hs/Tz/Tc/Tp.
  if (t < (long)wave_measurement_filter_warm_up) {
    nWarm_++;
    return;
  }

  // Average to 10 Hz buckets (100 ms) -> streaming Welch.
  long bucket = t / kVacc10HzBucketMs;
  if (bucket != curBucket_) {
    if (bN_ > 0) {
      pushWelch((float)(bSum_ / bN_));
      n10_++;
    }
    curBucket_ = bucket;
    bSum_ = 0.0;
    bN_ = 0;
  }
  bSum_ += vSel;
  bN_++;
}

bool StreamAnalyzer::finalize(WaveParams &params, uint16_t *spectrumOut) {
  // Flush the last partial bucket.
  if (bN_ > 0) {
    pushWelch((float)(bSum_ / bN_));
    n10_++;
    bN_ = 0;
  }

  params = {-1.0f, -1.0f, -1.0f, -1.0f, 0.0f, 0.0, 0.0, 0.0};
  for (size_t j = 0; j < welch_bins; j++) spectrumOut[j] = 0;
  if (nSeg_ == 0) return false;

  const int N = kWelchSegLen;
  const float df = kVaccFsHz / N;
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
  if (peakEta > 0.0f) {
    for (size_t j = 0; j < welch_bins; j++) {
      int k = (int)welch_bin_min + (int)j;
      float f = k * df;
      float taper = lowFreqTaper(f);
      float w = 2.0f * (float)M_PI * f;
      float psdEta = (psdAcc_[k] * invSeg) / (w * w * w * w) * (taper * taper);
      if (psdEta < 0.0f) psdEta = 0.0f;
      float norm = psdEta / peakEta;         // 0..1
      if (norm > 1.0f) norm = 1.0f;
      spectrumOut[j] = (uint16_t)lroundf(norm * 65535.0f);
    }
  }
  return true;
}
