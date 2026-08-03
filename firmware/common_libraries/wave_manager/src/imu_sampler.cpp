#include "imu_sampler.h"
#include "config.h"   // SPI_CS_IMU_PIN and the shared SPI pin definitions
#include <math.h>

/*
  Ported from ORB_test/src/Imu.cpp. Differences for the buoy:
   - shares the global Arduino SPI object (sd_writer already called SPI.begin() on
     SPI1) instead of owning a private SPIClass;
   - no INT1 interrupt: update() polls and drains whatever the FIFO holds, so no
     dedicated interrupt pin is needed.
*/

ImuSampler::ImuSampler()
    : imu_(&SPI, (int)SPI_CS_IMU_PIN, kImuSpiHz) {}

bool ImuSampler::begin(Print &dbg) {
  // Keep CS high before any bus activity (sd_writer also drives it high on boot).
  pinMode(SPI_CS_IMU_PIN, OUTPUT);
  digitalWrite(SPI_CS_IMU_PIN, HIGH);

  if (imu_.begin() != LSM6DSV16X_OK) {
    dbg.println("LSM6DSV: begin() failed - check SPI wiring/CS");
    return false;
  }
  dbg.println("LSM6DSV: begin() OK");

  imu_.Enable_X();      // accelerometer
  imu_.Enable_G();      // gyroscope
  imu_.Set_X_FS((int32_t)kAccelFS);   // full-scale from wave_config (kAccelFS)
  imu_.Set_G_FS((int32_t)kGyroFS);    // full-scale from wave_config (kGyroFS)
  imu_.Set_X_ODR((float)kImuOdrHz, kImuAccMode);
  imu_.Set_G_ODR((float)kImuOdrHz, kImuGyrMode);

  if (kUseLpf2) {
    imu_.Set_X_Filter_Mode(0, kLpf2Bw);  // 0 => low-pass mode, arg2 = bandwidth
  }

  // Batch accel + gyro into the FIFO at the chosen ODR; SFLP game rotation vector
  // (quaternion) is batched alongside them. Stream mode is started separately.
  imu_.FIFO_Set_X_BDR((float)kImuOdrHz);
  imu_.FIFO_Set_G_BDR((float)kImuOdrHz);
  imu_.Enable_Rotation_Vector();
  imu_.Set_SFLP_ODR(kSflpOdrHz);
  imu_.Set_SFLP_Batch(true, false, false);  // (GameRotation, Gravity, gBias) -> FIFO

  return true;
}

void ImuSampler::startStreaming() {
  imu_.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
}

void ImuSampler::resetFifo() {
  imu_.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);  // flush hardware FIFO
  imu_.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);  // resume streaming
}

void ImuSampler::resetWindowing(uint32_t captureStartMs) {
  sessionStartMs_ = captureStartMs;
  logStarted_ = false;
  accelIdx_ = 0;
  sampleTms_ = 0.0;
  samplePeriodMs_ = 1000.0 / kAccelOdrHz;
  curWinIdx_ = -1;
  winNAcc_ = winNGyr_ = 0;
  winBraking_ = false;
  winSflpNan_ = false;
  winFifoOvf_ = false;
  brakeRun_ = 0;
  winSumAx_ = winSumAy_ = winSumAz_ = 0;
  winSumNx_ = winSumNy_ = winSumNz_ = 0;
  winSumGx_ = winSumGy_ = winSumGz_ = 0;
  dbgLastPrint_ = captureStartMs;
  nAccDbg_ = nGyrDbg_ = 0;
  sumAccMag_ = sumGyrMag_ = 0.0;
}

// Close the current window -> build one ImuRow (means) and hand it to the row sink.
void ImuSampler::closeWindow() {
  if (winNAcc_ == 0) return;
  ImuRow r;
  r.winStartMs = (uint32_t)curWinIdx_ * kWindowMs;
  r.n = winNAcc_;
  r.ax = (float)(winSumAx_ / winNAcc_);
  r.ay = (float)(winSumAy_ / winNAcc_);
  r.az = (float)(winSumAz_ / winNAcc_);
  r.axn = (float)(winSumNx_ / winNAcc_);
  r.ayn = (float)(winSumNy_ / winNAcc_);
  r.azn = (float)(winSumNz_ / winNAcc_);
  uint16_t ng = winNGyr_ ? winNGyr_ : 1;  // avoid /0 if gyro missing
  r.gx = (float)(winSumGx_ / ng);
  r.gy = (float)(winSumGy_ / ng);
  r.gz = (float)(winSumGz_ / ng);
  r.qw = latestQw_; r.qx = latestQx_; r.qy = latestQy_; r.qz = latestQz_;
  r.braking = winBraking_ ? 1 : 0;
  r.mqw = 1.0f; r.mqx = 0.0f; r.mqy = 0.0f; r.mqz = 0.0f;
  r.vaccMadgwick = 0.0f; r.vaccSflp = 0.0f;
  r.sflpNan = winSflpNan_ ? 1 : 0;
  r.fifoOvf = winFifoOvf_ ? 1 : 0;
  if (rowSink_) rowSink_(r);
}

// Drain all pending FIFO words. Three tags are batched together: accel (2), gyro
// (1) and SFLP game rotation / quaternion (0x13). Accel/gyro are window-aggregated
// (~100 Hz), the quaternion is taken as the latest in the window.
void ImuSampler::update(Print &dbg) {
  uint8_t full = 0;
  imu_.FIFO_Get_Full_Status(&full);
  if (full) {
    nOverflow_++;       // stream mode overwrote oldest samples -> draining too slowly
    winFifoOvf_ = true; // flag the window that was open when it happened (fifo_ovf in imu.csv)
  }

  uint16_t nSamples = 0;
  imu_.FIFO_Get_Num_Samples(&nSamples);

  for (uint16_t i = 0; i < nSamples; i++) {
    uint8_t tag = 0;
    imu_.FIFO_Get_Tag(&tag);
    if (tag == 2) {  // accel (mg)
      int32_t a[3];
      imu_.FIFO_Get_X_Axes(a);
      sumAccMag_ += sqrt((double)a[0] * a[0] + (double)a[1] * a[1] + (double)a[2] * a[2]);
      nAccDbg_++;
      // Windowing: samples arrive in FIFO bursts but represent evenly spaced points
      // in time. A running, monotonic clock (sampleTms_ += samplePeriodMs_) gives a
      // steady ~10 ms binning; samplePeriodMs_ self-calibrates below so the axis
      // tracks the wall clock without drift.
      uint32_t tms = (uint32_t)sampleTms_;
      int32_t widx = (int32_t)(tms / kWindowMs);
      if (!logStarted_) { logStarted_ = true; curWinIdx_ = widx; }
      if (widx != curWinIdx_) {
        closeWindow();
        curWinIdx_ = widx;
        winNAcc_ = winNGyr_ = 0;
        winBraking_ = false;
        winSflpNan_ = false;
        winFifoOvf_ = false;
        winSumAx_ = winSumAy_ = winSumAz_ = 0;
        winSumNx_ = winSumNy_ = winSumNz_ = 0;
        winSumGx_ = winSumGy_ = winSumGz_ = 0;
      }
      winSumAx_ += a[0]; winSumAy_ += a[1]; winSumAz_ += a[2]; winNAcc_++;
      // Brake flag with debounce: LINEAR accel (gravity removed) must stay over
      // threshold for kBrakeMinSamples in a row. Rotate body accel to world/NED
      // with the SFLP quaternion (world = R(q).a_body), then subtract gravity.
      float qw = latestQw_, qx = latestQx_, qy = latestQy_, qz = latestQz_;
      float ax = (float)a[0], ay = (float)a[1], az = (float)a[2];
      float wX = (1 - 2 * (qy * qy + qz * qz)) * ax + 2 * (qx * qy - qw * qz) * ay + 2 * (qx * qz + qw * qy) * az;
      float wY = 2 * (qx * qy + qw * qz) * ax + (1 - 2 * (qx * qx + qz * qz)) * ay + 2 * (qy * qz - qw * qx) * az;
      float wZ = 2 * (qx * qz - qw * qy) * ax + 2 * (qy * qz + qw * qx) * ay + (1 - 2 * (qx * qx + qy * qy)) * az;
      wZ -= 1000.0f;  // remove 1 g gravity
      winSumNx_ += wX; winSumNy_ += wY; winSumNz_ += wZ;
      double aMag2 = (double)wX * wX + (double)wY * wY + (double)wZ * wZ;
      if (aMag2 > kBrakeThresholdMg2) {
        if (brakeRun_ < 0xFFFF) brakeRun_++;
        if (brakeRun_ >= kBrakeMinSamples) winBraking_ = true;
      } else {
        brakeRun_ = 0;
      }
      sampleTms_ += samplePeriodMs_;
      accelIdx_++;
    } else if (tag == 1) {  // gyro (mdps)
      int32_t g[3];
      imu_.FIFO_Get_G_Axes(g);
      sumGyrMag_ += sqrt((double)g[0] * g[0] + (double)g[1] * g[1] + (double)g[2] * g[2]);
      nGyrDbg_++;
      winSumGx_ += g[0]; winSumGy_ += g[1]; winSumGz_ += g[2]; winNGyr_++;
    } else if (tag == kSflpGameRotationTag) {  // quaternion [x,y,z,w]
      float q[4];
      imu_.FIFO_Get_Rotation_Vector(q);
      // NaN guard: a corrupt FIFO word decodes to an invalid half-float and the
      // library's w = sqrt(1 - sumsq) yields NaN; one NaN poisons az_ned and the
      // whole SFLP spectrum. Keep the last valid quaternion instead.
      if (isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) && isfinite(q[3])) {
        latestQx_ = q[0]; latestQy_ = q[1]; latestQz_ = q[2]; latestQw_ = q[3];
      } else {
        winSflpNan_ = true;
      }
    } else {
      uint8_t dummy[6];
      imu_.FIFO_Get_Data(dummy);  // unknown tag: pop so the FIFO pointer advances
    }
  }

  // Calibrate the accel sample period from real elapsed time / total samples, so
  // the time axis tracks the wall clock and the last sample lands on real elapsed time.
  if (accelIdx_ > 0) {
    samplePeriodMs_ = (double)(millis() - sessionStartMs_) / (double)accelIdx_;
  }

  debugPrintStatus(dbg);
}

// Print the effective accel/gyro sample rate + mean magnitudes, at most every
// imu_debug_print_period ms. Generalises ORB_test's reportOncePerSecond to any
// interval (rate = count / elapsed, not assuming a 1 s window). Also surfaces any
// FIFO overflow since the last print (draining too slowly).
void ImuSampler::debugPrintStatus(Print &dbg) {
  if (!debug_serial || imu_debug_print_period == 0) return;
  uint32_t now = millis();
  if (now - dbgLastPrint_ < imu_debug_print_period) return;

  double elapsedS = (now - dbgLastPrint_) / 1000.0;
  double accHz = elapsedS > 0 ? nAccDbg_ / elapsedS : 0.0;
  double gyrHz = elapsedS > 0 ? nGyrDbg_ / elapsedS : 0.0;
  double avgAcc = nAccDbg_ ? sumAccMag_ / nAccDbg_ : 0.0;
  double avgGyr = nGyrDbg_ ? sumGyrMag_ / nGyrDbg_ : 0.0;

  dbg.print("[IMU] Accel: "); dbg.print(accHz, 0);
  dbg.print(" Hz, |a| = ");   dbg.print(avgAcc, 1);
  dbg.print(" mg  |  Gyro: "); dbg.print(gyrHz, 0);
  dbg.print(" Hz, |g| = ");   dbg.print(avgGyr, 1);
  dbg.print(" mdps");
  if (nOverflow_ > 0) {
    dbg.print("  [WARN] FIFO overflow x"); dbg.print(nOverflow_);
    nOverflow_ = 0;
  }
  dbg.println();

  nAccDbg_ = nGyrDbg_ = 0;
  sumAccMag_ = sumGyrMag_ = 0.0;
  dbgLastPrint_ = now;
}
