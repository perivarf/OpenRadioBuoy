#ifndef IMU_SAMPLER_H
#define IMU_SAMPLER_H

#include <Arduino.h>
#include <SPI.h>
#include <LSM6DSV16XSensor.h>
#include "wave_config.h"

/*
  One window-aggregated IMU row (mean over kWindowMs), ported from ORB_test Imu.h.
  Shared type: ImuSampler produces it, the analyzer and CSV logger consume it.
*/
struct ImuRow {
  uint32_t winStartMs;              // window start, relative ms from capture start
  uint16_t n;                       // accel samples in the window
  float ax, ay, az;                 // mean accel (mg, body frame)
  float axn, ayn, azn;              // mean linear accel (mg, world/NED, gravity removed)
  float gx, gy, gz;                 // mean gyro (mdps)
  float qw, qx, qy, qz;             // last quaternion in window (on-chip SFLP)
  uint8_t braking;                  // 1 if linear |a| > threshold long enough
  // Filled by the analyzer before logging:
  float mqw, mqx, mqy, mqz;         // Madgwick/Kalman quaternion (streaming AHRS)
  float vaccMadgwick, vaccSflp;     // vertical linear accel (m/s^2): selected method vs SFLP
  uint8_t sflpNan;                  // 1 if a NaN SFLP quaternion was rejected in the window
};

// Callback invoked when a window closes. The row is NOT const: the analyzer fills
// the Madgwick/Kalman fields in the sink before the logger writes it.
using ImuRowSink = void (*)(ImuRow &);

/*
  IMU driver for the LSM6DSV family. Ported from ORB_test/src/Imu, adapted for the
  buoy: shares the global Arduino SPI object (already brought up by sd_writer on
  SPI1) instead of owning its own bus, and drains the FIFO by polling its fill
  level - no INT1 pin, since the interrupt line is not routed on this PCB.
*/
class ImuSampler {
 public:
  ImuSampler();

  // Init sensor: ODR/FS/filter, FIFO + SFLP batching. Does NOT start the FIFO
  // stream (see startStreaming). Assumes the shared SPI bus is already begun.
  bool begin(Print &dbg);

  // Put the FIFO into STREAM/continuous mode so it starts filling.
  void startStreaming();

  // Flush the hardware FIFO (BYPASS -> STREAM) and clear pending state.
  void resetFifo();

  // Drain all pending FIFO words once (call repeatedly during a capture).
  void update(Print &dbg);

  // Reset windowing for a new capture. captureStartMs is the capture t=0.
  void resetWindowing(uint32_t captureStartMs);

  void setRowSink(ImuRowSink sink) { rowSink_ = sink; }

  uint32_t overflowCount() const { return nOverflow_; }

 private:
  void closeWindow();

  // Print the effective accel/gyro rate + mean magnitudes at most every
  // imu_debug_print_period ms (the ex-reportOncePerSecond, now interval-driven).
  void debugPrintStatus(Print &dbg);

  LSM6DSV16XSensor imu_;
  ImuRowSink rowSink_ = nullptr;

  uint32_t nOverflow_ = 0;

  // Debug-rate counters (accumulated per print interval, then reset).
  uint32_t dbgLastPrint_ = 0;
  uint32_t nAccDbg_ = 0, nGyrDbg_ = 0;
  double   sumAccMag_ = 0.0, sumGyrMag_ = 0.0;

  // Windowing state: 10 ms windows off a monotonic accel counter.
  uint32_t sessionStartMs_ = 0;
  bool     logStarted_ = false;
  uint32_t accelIdx_ = 0;
  double   sampleTms_ = 0.0;
  double   samplePeriodMs_ = 1000.0 / kAccelOdrHz;
  int32_t  curWinIdx_ = -1;
  uint16_t winNAcc_ = 0, winNGyr_ = 0;
  bool     winBraking_ = false;
  bool     winSflpNan_ = false;
  uint16_t brakeRun_ = 0;
  double   winSumAx_ = 0, winSumAy_ = 0, winSumAz_ = 0;
  double   winSumNx_ = 0, winSumNy_ = 0, winSumNz_ = 0;  // linear accel, world/NED frame
  double   winSumGx_ = 0, winSumGy_ = 0, winSumGz_ = 0;
  float    latestQw_ = 1, latestQx_ = 0, latestQy_ = 0, latestQz_ = 0;
};

#endif  // IMU_SAMPLER_H
