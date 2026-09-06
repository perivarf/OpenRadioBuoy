#ifndef WAVE_ANALYSIS_H
#define WAVE_ANALYSIS_H

#include <Arduino.h>
#include "config.h"
#include "wave_config.h"
#include "imu_row.h"
#include "fir.h"
#include "fir_coeffs.h"

// PIF TODO

/*
  Streaming wave analysis. Orientation method (Madgwick, Kalman, or SFLP)
  is a compile-time choice made in wave_config.h. 
  
  The chain is:

    1) vertical linear accel 
    2) FIR decimation to kWelchInputOdrHz
    3) Welch PSD acc
    4) PSD acc -> PSD elevation (/omega^4) 
    5) Spectral moments m0/m2/m4 -> 
       Hs = 4*sqrt(m0), Tz = sqrt(m0/m2), Tc = sqrt(m2/m4), Tp = 1/f_peak.
       with low-frequency taper.

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

  // The Welch FFT. ingest() decimate to kWelchInputOdrHz, fills the ring and raises a flag
  // processPendingSegment() runs accumSegment. 
  // Should be called from the capture loop right after FIFO drain.
  // A no-op when no segment is pending, so calling it every iteration is free.
  //
  // Returns whether a segment was actually accumulated
  bool processPendingSegment(void);

  // Finalise: average the PSD-sums, derive wave parameters from the elevation spectrum, and
  // fill the quantised spectrum bins (welch_bin_min..welch_bin_max) from the
  // acceleration spectrum. Returns false if no usable segment.
  bool finalize(WaveParams &params, uint16_t *spectrumOut);

  // Which orientation filter produced the vacc column, for ses.csv / cfg.csv.
  const char *orientationName(void) const { return wave_orientation_name; }

  // Accessors for the CSV logger (spec.csv / ana.csv).
  const float *psd() const { return psdAcc_; }
  uint16_t     psdBins() const { return kWelchSegLen / 2 + 1; }
  uint32_t     segments() const { return nSeg_; }
  uint32_t     samplesWelch() const { return nWelch_; }
  uint32_t     rows() const { return nData_; }
  uint32_t     brakeRows() const { return nBrake_; }
  uint32_t     warmupRows() const { return nWarm_; } 

  // Times the ring had no free slot and the FFT had to run inside the pop loop after
  // all. Should be zero. Logged to ana.csv.
  uint32_t     ringFullCount() const { return nRingFull_; }

 private:
  void pushWelch(float sample);  // push one Welch-rate sample into the segment

  // Second decimation stage: kRowOdrHz -> kWelchInputOdrHz into Welch.
  // Applied on every reading including the warm-up ones
  // - otherwise the delay line is still half full of zeros when the
  // first Welch sample is taken
  FirDecimator fir2_{kFirCoeffsStage2};
  long curBucket_ = -1;
  bool bucketDone_ = false;   // this bucket's centre has been reached and evaluated

  // Counters.
  uint32_t nWelch_ = 0, nData_ = 0, nBrake_ = 0, nWarm_ = 0;

  // Streaming Welch: one segment ring + PSD accumulator.
  //
  // A ring buffer because accumSegment is deferred out of the pop loop:
  // the samples that arrive while a full segment waits have to land somewhere, and the
  // ring is kWelchRingSlack larger than a segment for exactly that
  float ring_[kWelchRingLen];
  uint16_t head_ = 0;      // where the next sample goes
  uint16_t tail_ = 0;      // oldest sample = start of the segment being accumulated
  uint16_t fill_ = 0;      // samples held, tail_ -> head_
  bool     segPending_ = false;   // a full segment is waiting for processPendingSegment
  float psdAcc_[kWelchSegLen / 2 + 1];
  uint32_t nSeg_ = 0;
  uint32_t nRingFull_ = 0;
};

#endif  // WAVE_ANALYSIS_H
