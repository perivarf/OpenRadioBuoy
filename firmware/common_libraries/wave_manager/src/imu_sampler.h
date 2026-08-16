#ifndef IMU_SAMPLER_H
#define IMU_SAMPLER_H

#include <Arduino.h>
#include "constants.h"
#include "wave_config.h"
#include "wave_timing.h"  // wave_timing buckets; compiles away with wave_timing_enabled
#include "imu_row.h"      // ImuRow, ImuRowSink - what a closed window produces
#include "fir_row_bank.h" // FirRowBank - the ten first-stage decimation filters
#include "fir_coeffs.h"   // kFirCoeffsStage1 - generated, see tools/gen_fir_table.py
#include "quat_delay.h"
#include "imu_device.h"   // ImuDevice, ImuFifoWord - the sensor, and the only place
                          // that knows which one it is
#include "raw_log.h"      // RawLogWriter - the <stamp>_raw.bin byte format

/*
  The sampling pipeline: drain the FIFO, run the AHRS on every raw sample, decimate
  through the FIR bank, and emit one ImuRow per window.

  Device-neutral - everything sensor-specific is behind ImuDevice (imu_device.h). The
  five lifecycle calls below forward to it and exist so callers have one object to talk
  to; the pipeline itself deals only in ImuFifoWord.

  Owns the AHRS: the raw samples exist nowhere else, and running the orientation filter
  on window means would integrate the gyro at a rate it was never measured at.
*/
class ImuSampler {
 public:
  // Bring the sensor up. Does NOT start the FIFO stream (see startStreaming).
  // Called at boot and again from WaveManager::wake() before each capture, so the
  // return value answers "is the IMU alive now", not "was it at boot".
  bool begin(Print &dbg) { return dev_.begin(dbg); }

  // Start the FIFO filling, and drop any pending watermark flag.
  void startStreaming() { dev_.startStreaming(); }

  // Flush the hardware FIFO and clear pending state, leaving it idle. The latched
  // overrun goes with the words being thrown away - carrying it across would pin a
  // previous capture's overrun on the first window of the next one.
  void resetFifo() { dev_.bypassFifo(); pendingOvrLatched_ = false; }

  // Park the sensor between captures; begin() is the other half, as with the GPS.
  void shutdownIMU() { dev_.shutdown(); }

  // Boot liveness check: begin() only proves the part ANSWERS, so a dead or stuck
  // converter passes it. False only if the sensor cannot be read.
  bool checkImu(Print &dbg) { return dev_.checkAlive(dbg); }

  // Drain all pending FIFO words once (call repeatedly during a capture).
  // captureLeftMs is passed straight to the debug line - the sampler does not time
  // the capture, it only reports what the caller already knows.
  void update(Print &dbg, uint32_t captureLeftMs);

  // Reset windowing for a new capture. captureStartMs is the capture t=0.
  void resetWindowing(uint32_t captureStartMs);

  void setRowSink(ImuRowSink sink) { rowSink_ = sink; }

  // The raw log to feed, or nullptr for none. Owned by the caller (WaveManager owns
  // the file), because opening, headering and closing it are all its business - the
  // sampler only emits into it, from inside the drain.
  void setRawLog(RawLogWriter *log) { rawLog_ = log; }

  // FIFO fills since the last debug print, which zeroes it on the way out.
  uint32_t overflowCount() const { return nOverflow_; }

  // FIFO fills for the WHOLE capture (ana.csv). nOverflow_ is the debug print's own
  // counter and is zeroed on every print, so it cannot answer "was this capture clean?".
  uint32_t overflowTotal() const { return nOverflowTotal_; }

  // Windows where no raw sample landed on the centre and the FIR had to be read at
  // the window edge instead. Non-zero means FIFO gaps; logged to ana.csv.
  uint32_t firLateEvalCount() const { return nFirLateEval_; }

 private:
  // When the last drain ENDED. The DRAIN gate in update() measures kDrainIntervalMs
  // from here; the WTM gate does not, but this is still reset on every drain because
  // the re-arm depends on it.
  uint32_t lastDrainMs_ = 0;

  // FIFO_OVR_LATCHED picked up by the re-arm read at the END of a drain, carried to the
  // next one. That read is the only place an overrun DURING the pop loop is still
  // visible: the loop keeps popping afterwards, so by the next drain's status read the
  // level is back under the brim and FIFO_OVR_IA reads zero again. Since the register is
  // reset by the very read that reports it, the bit has to be remembered here or lost.
  bool pendingOvrLatched_ = false;

  void closeWindow();

  // Evaluate the FIR bank + read the delayed quaternions into pendingRow_.
  void latchRowValues();

  // Print the effective accel/gyro rate + mean magnitudes at most every
  // imu_debug_print_period ms (the ex-reportOncePerSecond, now interval-driven),
  // led by how much of the capture is left.
  void debugPrintStatus(Print &dbg, uint32_t captureLeftMs);

  ImuDevice     dev_;
  ImuRowSink    rowSink_ = nullptr;
  RawLogWriter *rawLog_  = nullptr;

  uint32_t nOverflow_ = 0;
  uint32_t nOverflowTotal_ = 0;
  uint32_t nFirLateEval_ = 0;

  // Debug counters (accumulated per print interval, then reset). Only what bears on
  // LOST SAMPLES is kept - see debugPrintStatus for why the margin counters (peak
  // DIFF_FIFO, FIFO_FULL_IA) and the |a|/|g| sums went away.
  //   nAccDbg_/nGyrDbg_  samples decoded -> the rate, which is below kImuOdrHz
  //                      exactly when something went missing
  //   nUnknownDbg_       words popped but decoded by no branch (datasheet table 210)
  uint32_t dbgLastPrint_ = 0;
  uint32_t nAccDbg_ = 0, nGyrDbg_ = 0;
  uint32_t nUnknownDbg_ = 0;
  uint8_t  lastUnknownTag_ = 0;

  // Windowing state: kRowPeriodMs windows off a monotonic accel counter.
  uint32_t sessionStartMs_ = 0;
  bool     logStarted_ = false;
  uint32_t accelIdx_ = 0;
  double   sampleTms_ = 0.0;
  double   samplePeriodMs_ = 1000.0 / kAccelOdrHz;
  int32_t  curWinIdx_ = -1;
  uint16_t winNAcc_ = 0;
  bool     winBraking_ = false;
  bool     winSflpNan_ = false;
  bool     winFifoOvf_ = false;
  bool     winFirDone_ = false;
  uint16_t brakeRun_ = 0;
  float    latestQw_ = 1, latestQx_ = 0, latestQy_ = 0, latestQz_ = 0;

  // AHRS on the raw stream, one update per accel sample. latestG*_ pairs the gyro
  // with the accel sample that drives the update - they arrive as separate FIFO
  // tags, so the freshest gyro word is the best available match (one sample of skew).
  WaveAhrs ahrs_ = makeWaveAhrs();
  bool     ahrsSeeded_ = false;
  float    latestGx_ = 0, latestGy_ = 0, latestGz_ = 0;

  FirRowBank              fir_{kFirCoeffsStage1};
  QuatDelay<kQuatDelaySlots> qDelay_;
  ImuRow                  pendingRow_{};
};

#endif  // IMU_SAMPLER_H
