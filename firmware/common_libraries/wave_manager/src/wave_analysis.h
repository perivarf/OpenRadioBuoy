#ifndef WAVE_ANALYSIS_H
#define WAVE_ANALYSIS_H

#include <Arduino.h>
#include "config.h"        // common_config.h: welch_bins, welch_bin_min/max
#include "wave_config.h"
#include "imu_sampler.h"   // ImuRow

/*
  Streaming wave analysis, ported from ORB_test/src/analysis.{h,cpp} and reduced to
  the single on-device orientation method selected by wave_orientation_method
  (Madgwick / Kalman / SFLP) to fit RAM. The chain is:

    orientation (AHRS) -> vertical linear accel -> 10 Hz bucketing ->
    streaming Welch PSD -> acc->elevation (/omega^4) with low-frequency taper ->
    spectral moments m0/m2/m4 -> Hs = 4*sqrt(m0), Tz = sqrt(m0/m2), Tc = sqrt(m2/m4),
    Tp = 1/f_peak.

  Welch keeps a single kWelchSegLen segment (75% overlap) and accumulates the PSD,
  so only ~one segment lives in RAM regardless of capture length.
*/

// Classic 2-state Kalman for one angle (Lauszus): fuses the gyro rate (prediction)
// with the accel tilt (measurement) and estimates the gyro bias. Translated from
// ORB_test/tools/postprocess.py KalmanAngle.
class KalmanAngle {
 public:
  void reset(float angle);
  float update(float newAngle, float newRate, float dt);
  float angle() const { return angle_; }
 private:
  float angle_ = 0.0f;
  float bias_ = 0.0f;
  float P_[2][2] = {{0, 0}, {0, 0}};
};

// One set of spectral moments + derived wave parameters.
struct WaveParams {
  float hs, tz, tc, tp, maxValue;
  double m0, m2, m4;
};

class StreamAnalyzer {
 public:
  void begin(void);           // reset all state for a new capture
  void ingest(ImuRow &r);     // per row: fill r.mq*/r.vacc*, accumulate Welch

  // Finalise: average the PSD, derive wave parameters, fill the quantised spectrum
  // bins (welch_bin_min..welch_bin_max). Returns false if no usable segment.
  bool finalize(WaveParams &params, uint16_t *spectrumOut);

  // Accessors for the CSV logger (spec.csv / ana.csv).
  const float *psd() const { return psdAcc_; }
  uint16_t     psdBins() const { return kWelchSegLen / 2 + 1; }
  uint32_t     segments() const { return nSeg_; }
  uint32_t     samples10Hz() const { return n10_; }
  uint32_t     rows() const { return nData_; }
  uint32_t     brakeRows() const { return nBrake_; }

 private:
  void pushWelch(float sample);  // push one 10 Hz sample into the segment

  // Orientation state.
  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  KalmanAngle kroll_, kpitch_;
  bool haveQ_ = false;
  long prevT_ = 0;

  // 10 Hz bucketing.
  long curBucket_ = -1;
  double bSum_ = 0.0;
  uint32_t bN_ = 0;

  // Counters.
  uint32_t n10_ = 0, nData_ = 0, nBrake_ = 0;

  // Streaming Welch: one segment buffer + PSD accumulator.
  float segBuf_[kWelchSegLen];
  int   segFill_ = 0;
  float psdAcc_[kWelchSegLen / 2 + 1];
  uint32_t nSeg_ = 0;
};

// Shared DSP helpers (ported from analysis.cpp), exposed for reuse/testing.
void  madgwickUpdateIMU(float q[4], float gx, float gy, float gz,
                        float ax, float ay, float az, float dt, float beta);
void  initQuatFromAccel(float q[4], float ax, float ay, float az);
void  quatFromRollPitch(float q[4], float roll, float pitch);
float verticalAccel(const float q[4], float ax, float ay, float az);

#endif  // WAVE_ANALYSIS_H
