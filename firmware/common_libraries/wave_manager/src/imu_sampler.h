#ifndef IMU_SAMPLER_H
#define IMU_SAMPLER_H

#include <Arduino.h>
#include "constants.h"
#include "wave_config.h"
#include "wave_timing.h"
#include "imu_row.h"
#include "fir_row_bank.h"
#include "fir_coeffs.h"
#include "quat_delay.h"
#include "imu_device.h"   
#include "raw_log.h"

/*
  The sampling pipeline: drain the FIFO, run the AHRS on every raw sample, decimate
  through the FIR bank, and emit one ImuRow per window.

  Trying to be device-neutral - sensor-specific should behind ImuDevice (imu_device.h). 
*/
class ImuSampler {
 public:
  // Bring the sensor up. Does NOT start the FIFO stream (see startStreaming).
  // Called at boot and again from WaveManager::wake() before each capture
  // Returns "is the IMU alive now"
  bool begin(Print &dbg) { return dev_.begin(dbg); }

  // Start the FIFO filling, and drop any pending watermark flag.
  void startStreaming() { dev_.startStreaming(); }

  // Flush the hardware FIFO and clear pending state, leaving it idle
  void resetFifo() { dev_.bypassFifo(); pendingOvrLatched_ = false; }

  // Shutdown the sensor between captures
  void shutdownIMU() { dev_.shutdown(); }

  bool checkImu(Print &dbg) { return dev_.checkAlive(dbg); }

  // Drain all pending FIFO words once (call repeatedly during a capture).
  // captureLeftMs and gpsRows are passed straight to the debug output
  void update(Print &dbg, uint32_t captureLeftMs, uint32_t gpsRows = 0);

  // Reset windowing for a new capture. captureStartMs is the capture t=0.
  void resetWindowing(uint32_t captureStartMs);

  void setRowSink(ImuRowSink sink) { rowSink_ = sink; }

  // The raw log to feed, or nullptr for none
  void setRawLog(RawLogWriter *log) { rawLog_ = log; }

  // FIFO fills since the last debug print, which zeroes it on the way out.
  uint32_t overflowCount() const { return nOverflow_; }

  // FIFO fills for the whole capture (outputted later to ana.csv). 
  uint32_t overflowTotal() const { return nOverflowTotal_; }

  // Windows where no raw sample landed on the centre and the FIR had to be read at
  // the window edge instead. Non-zero means FIFO gaps; logged to ana.csv.
  uint32_t firLateEvalCount() const { return nFirLateEval_; }

 private:
  // When the last FIFO drain ended
  uint32_t lastDrainMs_ = 0;

  // FIFO_OVR_LATCHED picked up at the end of a drain, carried to the
  // next one. That read is the only place an overrun during the pop loop is still
  // visible: the loop keeps popping afterwards, so by the next drain's status read the
  // level is back under the brim and FIFO_OVR_IA reads zero again. Since the register is
  // reset by the very read that reports it, the bit has to be remembered here.
  bool pendingOvrLatched_ = false;

  void closeWindow();

  // Evaluate the FIR bank + read the delayed quaternions into pendingRow_.
  void latchRowValues();

  void debugPrintStatus(Print &dbg, uint32_t captureLeftMs, uint32_t gpsRows);

  ImuDevice     dev_;
  ImuRowSink    rowSink_ = nullptr;
  RawLogWriter *rawLog_  = nullptr;

  uint32_t nOverflow_ = 0;
  uint32_t nOverflowTotal_ = 0;
  uint32_t nFirLateEval_ = 0;

  // Debug counters (accumulated per print interval)
  uint32_t dbgLastPrint_ = 0;         // Last debug print (millis())
  uint32_t nAccDbg_ = 0, nGyrDbg_ = 0;// Samples decoded since the last debug print
  uint32_t dbgGpsPrev_ = 0;           // gpsRows at the last print
  uint32_t nUnknownDbg_ = 0;          // Unknown values from FIFO read, since last debug print
  uint8_t  lastUnknownTag_ = 0;       // The tag of the last unknown value, for the debug print

    // Windowing state
  uint32_t sessionStartMs_ = 0;         // Capture start (millis())
  bool     logStarted_ = false;         // First accel sample seen
  uint32_t accelIdx_ = 0;               // Accel samples this capture
  double   sampleTms_ = 0.0;            // Software clock (ms), advanced by samplePeriodMs_ per accel sample
  double   samplePeriodMs_ = 1000.0 / kImuOdrHz;  // ms/sample
  int32_t  curWinIdx_ = -1;             // Row window currently accumulating; -1 = none yet
  uint16_t winNAcc_ = 0;                // Accel samples in the current window
  bool     winBraking_ = false;         // Brake events in window
  bool     winSflpNan_ = false;         // Corrupt/NaN SFLP quaternion in window
  bool     winFifoOvf_ = false;         // A FIFO overrun happened while this window was open
  bool     winFirDone_ = false;         // This window's row has read the FIR bank
  uint16_t brakeRun_ = 0;               // Number of samples in a row with breaking values
  float    latestQw_ = 1, latestQx_ = 0, latestQy_ = 0, latestQz_ = 0;  // Latest SFLP quat (own FIFO tag)


  // AHRS on the raw stream, one update per accel sample. latestG*_ pairs the gyro
  // with the accel sample that drives the update - they arrive as separate FIFO
  // tags, so the freshest gyro word is the best available match (max one sample of skew).
  AhrsFilter ahrs_ = makeAhrsFilter();
  bool     ahrsSeeded_ = false;
  float    latestGx_ = 0, latestGy_ = 0, latestGz_ = 0;

  FirRowBank              fir_{kFirCoeffsStage1};
  QuatDelay<kQuatDelaySlots> qDelay_;
  ImuRow                  pendingRow_{};
};

#endif  // IMU_SAMPLER_H
