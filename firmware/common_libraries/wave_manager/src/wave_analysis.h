#ifndef WAVE_ANALYSIS_H
#define WAVE_ANALYSIS_H

#include <Arduino.h>
#include "config.h"        // common_config.h: welch_bins (wire-format bin count)
#include "wave_config.h"
#include "imu_sampler.h"   // ImuRow
#include "fir.h"           // FirDecimator - the second decimation stage
#include "fir_coeffs.h"    // kFirCoeffsStage2

/*
  Streaming wave analysis, ported from ORB_test/src/analysis.{h,cpp} and reduced to
  a single on-device orientation method (Madgwick / Kalman / SFLP) to fit RAM. Which
  one is a compile-time choice made at the bottom of wave_config.h. The chain is:

    vertical linear accel (already computed per RAW sample by ImuSampler and
    FIR-decimated into the row) -> FIR decimation to kWelchInputOdrHz ->
    streaming Welch PSD -> acc->elevation (/omega^4) with low-frequency taper ->
    spectral moments m0/m2/m4 -> Hs = 4*sqrt(m0), Tz = sqrt(m0/m2), Tc = sqrt(m2/m4),
    Tp = 1/f_peak.

  The orientation filter runs on the raw FIFO stream

  Welch keeps a single kWelchSegLen segment (75% overlap) and accumulates the PSD,
  so only ~one segment lives in RAM regardless of capture length.
*/

// One set of spectral moments + derived wave parameters.
// maxValue is the peak acceleration PSD [(m/s^2)^2/Hz] over the transmitted bins
struct WaveParams {
  float hs, tz, tc, tp, maxValue;
  double m0, m2, m4;
};

class StreamAnalyzer {
 public:
  void begin(void);              // reset all state for a new capture
  void ingest(const ImuRow &r);  // per row: decimate to kWelchInputOdrHz, accumulate Welch

  // Finalise: average the PSD, derive wave parameters from the ELEVATION spectrum, and
  // fill the quantised spectrum bins (welch_bin_min..welch_bin_max) from the
  // ACCELERATION spectrum. Returns false if no usable segment.
  bool finalize(WaveParams &params, uint16_t *spectrumOut);

  // Which orientation filter produced the vacc column, for ses.csv / cfg.csv.
  const char *orientationName(void) const { return wave_orientation_name; }

  // Accessors for the CSV logger (spec.csv / ana.csv).
  const float *psd() const { return psdAcc_; }
  uint16_t     psdBins() const { return kWelchSegLen / 2 + 1; }
  uint32_t     segments() const { return nSeg_; }
  uint32_t     samples10Hz() const { return n10_; }
  uint32_t     rows() const { return nData_; }
  uint32_t     brakeRows() const { return nBrake_; }
  uint32_t     warmupRows() const { return nWarm_; }  // rows dropped before the AHRS settled

 private:
  void pushWelch(float sample);  // push one 10 Hz sample into the segment

  // Second decimation stage: kRowOdrHz -> kWelchInputOdrHz into Welch.
  // Applied on every reading including the warm-up ones 
  // - otherwise the delay line is still half full of zeros when the
  // first Welch sample is taken and the spectrum starts with a filter transient.
  FirDecimator fir2_{kFirCoeffsStage2};
  long curBucket_ = -1;
  bool bucketDone_ = false;   // this bucket's centre has been reached and evaluated

  // Counters.
  uint32_t n10_ = 0, nData_ = 0, nBrake_ = 0, nWarm_ = 0;

  // Streaming Welch: one segment buffer + PSD accumulator.
  float segBuf_[kWelchSegLen];
  int   segFill_ = 0;
  float psdAcc_[kWelchSegLen / 2 + 1];
  uint32_t nSeg_ = 0;
};

#endif  // WAVE_ANALYSIS_H
