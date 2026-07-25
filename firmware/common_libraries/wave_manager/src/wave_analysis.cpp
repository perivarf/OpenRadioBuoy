#include "wave_analysis.h"

#include <math.h>
#include <string.h>

/*
  Ported from ORB_test/src/analysis.cpp (Madgwick / FFT / Welch / wave parameters)
  and ORB_test/tools/postprocess.py (KalmanAngle). Reduced to a single on-device
  orientation method selected by wave_orientation_method to fit RAM. The offline
  postprocess still recomputes all three methods from the raw imu.csv.
*/

// -----------------------------------------------------------------------------
// Orientation helpers (exposed via the header for reuse/testing).
// -----------------------------------------------------------------------------

// Madgwick 6-axis AHRS (IMU variant): update q=[w,x,y,z] from gyro (rad/s) and
// accel (any unit; normalised internally) over dt (s).
void madgwickUpdateIMU(float q[4], float gx, float gy, float gz,
                       float ax, float ay, float az, float dt, float beta) {
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    float recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  float recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q[0] = q0 * recipNorm; q[1] = q1 * recipNorm; q[2] = q2 * recipNorm; q[3] = q3 * recipNorm;
}

// Init quaternion from one accel sample (gravity -> roll/pitch, yaw=0).
void initQuatFromAccel(float q[4], float ax, float ay, float az) {
  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  q[0] = cp * cr; q[1] = cp * sr; q[2] = sp * cr; q[3] = -sp * sr;
}

// Quaternion [w,x,y,z] from roll/pitch (yaw=0) - same convention as
// initQuatFromAccel, used for the Kalman orientation.
void quatFromRollPitch(float q[4], float roll, float pitch) {
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  q[0] = cp * cr; q[1] = cp * sr; q[2] = sp * cr; q[3] = -sp * sr;
}

// Vertical linear accel (m/s^2): rotate body accel to world Z, subtract gravity.
float verticalAccel(const float q[4], float ax, float ay, float az) {
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  float wZ = 2.0f * (qx * qz - qw * qy) * ax +
             2.0f * (qy * qz + qw * qx) * ay +
             (1.0f - 2.0f * (qx * qx + qy * qy)) * az;
  return wZ - kGravity;
}

// -----------------------------------------------------------------------------
// KalmanAngle (Lauszus 2-state), translated from postprocess.py.
// -----------------------------------------------------------------------------
void KalmanAngle::reset(float angle) {
  angle_ = angle;
  bias_ = 0.0f;
  P_[0][0] = P_[0][1] = P_[1][0] = P_[1][1] = 0.0f;
}

float KalmanAngle::update(float newAngle, float newRate, float dt) {
  // Prediction (integrate gyro, subtract estimated bias).
  float rate = newRate - bias_;
  angle_ += dt * rate;
  P_[0][0] += dt * (dt * P_[1][1] - P_[0][1] - P_[1][0] + kKalmanQAngle);
  P_[0][1] -= dt * P_[1][1];
  P_[1][0] -= dt * P_[1][1];
  P_[1][1] += kKalmanQBias * dt;
  // Correction from the accel measurement.
  float s = P_[0][0] + kKalmanR;
  float k0 = P_[0][0] / s;
  float k1 = P_[1][0] / s;
  float y = newAngle - angle_;
  angle_ += k0 * y;
  bias_ += k1 * y;
  float p00 = P_[0][0], p01 = P_[0][1];
  P_[0][0] -= k0 * p00;
  P_[0][1] -= k0 * p01;
  P_[1][0] -= k1 * p00;
  P_[1][1] -= k1 * p01;
  return angle_;
}

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
  q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
  kroll_.reset(0.0f); kpitch_.reset(0.0f);
  haveQ_ = false; prevT_ = 0;
  curBucket_ = -1; bSum_ = 0.0; bN_ = 0;
  n10_ = nData_ = nBrake_ = 0;
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

  long t = (long)r.winStartMs;
  float ax = r.ax, ay = r.ay, az = r.az;      // mg, body
  float gx = r.gx * kMdps2Rads;
  float gy = r.gy * kMdps2Rads;
  float gz = r.gz * kMdps2Rads;

  // Accel tilt (measurement for Kalman): roll about x, pitch about y.
  float rollAcc = atan2f(ay, az);
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az));

  float dt = 0.0f;
  if (!haveQ_) {
    initQuatFromAccel(q_, ax, ay, az);
    kroll_.reset(rollAcc);
    kpitch_.reset(pitchAcc);
    haveQ_ = true;
  } else {
    dt = (float)(t - prevT_) / 1000.0f;
    if (dt > 0.0f) {
      madgwickUpdateIMU(q_, gx, gy, gz, ax, ay, az, dt, kMadgwickBeta);
      kroll_.update(rollAcc, gx, dt);    // gyro rate about x
      kpitch_.update(pitchAcc, gy, dt);  // gyro rate about y
    }
  }
  prevT_ = t;

  // Vertical accel (m/s^2) for the SELECTED method, plus SFLP for logging.
  float axm = ax * kMg2Ms2, aym = ay * kMg2Ms2, azm = az * kMg2Ms2;
  float vSel;
  float qSel[4];
  if (wave_orientation_method == WaveOrientation::Sflp) {
    // SFLP azn is already gravity-compensated (world/NED Z), in mg.
    vSel = r.azn * kMg2Ms2;
    qSel[0] = r.qw; qSel[1] = r.qx; qSel[2] = r.qy; qSel[3] = r.qz;
  } else if (wave_orientation_method == WaveOrientation::Kalman) {
    quatFromRollPitch(qSel, kroll_.angle(), kpitch_.angle());
    vSel = verticalAccel(qSel, axm, aym, azm);
  } else {  // Madgwick
    qSel[0] = q_[0]; qSel[1] = q_[1]; qSel[2] = q_[2]; qSel[3] = q_[3];
    vSel = verticalAccel(qSel, axm, aym, azm);
  }
  float vSflp = r.azn * kMg2Ms2;

  if (!isfinite(vSel)) vSel = 0.0f;
  if (!isfinite(vSflp)) vSflp = 0.0f;

  // Write orientation result back into the row -> logged to imu.csv.
  r.mqw = qSel[0]; r.mqx = qSel[1]; r.mqy = qSel[2]; r.mqz = qSel[3];
  r.vaccMadgwick = vSel; r.vaccSflp = vSflp;

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
