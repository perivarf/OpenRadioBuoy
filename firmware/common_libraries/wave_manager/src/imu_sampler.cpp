#include "imu_sampler.h"
#include "config.h"   // SPI_CS_IMU_PIN and the shared SPI pin definitions
#include "rotation.h" // verticalAccel
#include <math.h>

/*
  Ported from ORB_test/src/Imu.cpp. Differences for the buoy:
   - shares the global Arduino SPI object (sd_writer already called SPI.begin() on
     SPI1) instead of owning a private SPIClass;
   - no INT1 interrupt: update() polls and drains whatever the FIFO holds, so no
     dedicated interrupt pin is needed;
   - the AHRS and the decimation filter run HERE, on the raw FIFO stream, instead of
     downstream on window means. The raw samples exist nowhere else, so this is the
     only place the gyro can be integrated at the rate it was measured at and the
     only place an antialias filter can see what it is supposed to remove.
*/

// -----------------------------------------------------------------------------
// FirRowBank
// -----------------------------------------------------------------------------
void FirRowBank::reset(void) {
  ax_.reset(); ay_.reset(); az_.reset();
  nx_.reset(); ny_.reset(); nz_.reset();
  gx_.reset(); gy_.reset(); gz_.reset();
  vacc_.reset();
}

void FirRowBank::push(float ax, float ay, float az,
                      float nx, float ny, float nz,
                      float gx, float gy, float gz, float vacc) {
  ax_.push(ax); ay_.push(ay); az_.push(az);
  nx_.push(nx); ny_.push(ny); nz_.push(nz);
  gx_.push(gx); gy_.push(gy); gz_.push(gz);
  vacc_.push(vacc);
}

void FirRowBank::eval(ImuRow &r) const {
  r.ax = ax_.eval(); r.ay = ay_.eval(); r.az = az_.eval();
  r.axn = nx_.eval(); r.ayn = ny_.eval(); r.azn = nz_.eval();
  r.gx = gx_.eval(); r.gy = gy_.eval(); r.gz = gz_.eval();
  r.vaccFir = vacc_.eval();
  // The SFLP vertical accel is the world-Z channel in m/s^2; filtering azn and
  // scaling is identical to filtering the scaled series (the filter is linear), so
  // there is no eleventh delay line for it.
  r.vaccSflpFir = r.azn * kMg2Ms2;
  // Unfiltered counterparts, read straight off the delay lines' centre taps - the
  // same instant the filtered values are centred on, at no extra cost.
  r.vaccMadgwick = vacc_.center();
  r.vaccSflp = nz_.center() * kMg2Ms2;
}

// -----------------------------------------------------------------------------
// ImuSampler
// -----------------------------------------------------------------------------
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
  winNAcc_ = 0;
  winBraking_ = false;
  winSflpNan_ = false;
  winFifoOvf_ = false;
  winFirDone_ = false;
  brakeRun_ = 0;
  // The SFLP quaternion was NOT cleared here before, so the first rows of a capture
  // could carry the previous capture's attitude into q*, ax_ned..az_ned and the
  // brake flag until the first 0x13 word arrived. Identity is the honest start.
  latestQw_ = 1; latestQx_ = latestQy_ = latestQz_ = 0;
  latestGx_ = latestGy_ = latestGz_ = 0;
  // The AHRS re-seeds from gravity on the first accel sample (ahrsSeeded_), which is
  // also where the quaternion delay line gets filled.
  ahrs_.reset();
  ahrsSeeded_ = false;
  ahrsN_ = 0;
  fir_.reset();
  dbgLastPrint_ = captureStartMs;
  nAccDbg_ = nGyrDbg_ = 0;
  sumAccMag2_ = sumGyrMag2_ = 0.0;
}

// Evaluate the decimation filters and read the delay-matched quaternions into the
// row being assembled. Everything written here refers to one instant: the centre of
// the current window, carried back by the FIR group delay.
void ImuSampler::latchRowValues() {
  const uint32_t t0 = micros();
  fir_.eval(pendingRow_);
  usFir_ += micros() - t0;
  float mq[4], sq[4];
  qDelay_.read(mq, sq);
  pendingRow_.mqw = mq[0]; pendingRow_.mqx = mq[1];
  pendingRow_.mqy = mq[2]; pendingRow_.mqz = mq[3];
  pendingRow_.qw = sq[0]; pendingRow_.qx = sq[1];
  pendingRow_.qy = sq[2]; pendingRow_.qz = sq[3];
  if (!isfinite(pendingRow_.vaccFir)) pendingRow_.vaccFir = 0.0f;
  if (!isfinite(pendingRow_.vaccSflpFir)) pendingRow_.vaccSflpFir = 0.0f;
  if (!isfinite(pendingRow_.vaccMadgwick)) pendingRow_.vaccMadgwick = 0.0f;
  if (!isfinite(pendingRow_.vaccSflp)) pendingRow_.vaccSflp = 0.0f;
  winFirDone_ = true;
}

// Close the current window -> finish the ImuRow and hand it to the row sink.
void ImuSampler::closeWindow() {
  if (winNAcc_ == 0) return;
  // No raw sample reached the window centre - a FIFO gap. Read the filters at the
  // window edge instead of dropping the row: the filtered value is still valid, it
  // is just centred up to kWindowMs late. Counted so a capture can be judged.
  if (!winFirDone_) {
    latchRowValues();
    nFirLateEval_++;
  }
  pendingRow_.winStartMs = (uint32_t)curWinIdx_ * kWindowMs;
  pendingRow_.n = winNAcc_;
  pendingRow_.braking = winBraking_ ? 1 : 0;
  pendingRow_.sflpNan = winSflpNan_ ? 1 : 0;
  pendingRow_.fifoOvf = winFifoOvf_ ? 1 : 0;
  if (rowSink_) rowSink_(pendingRow_);
}

// Drain all pending FIFO words. Three tags are batched together: accel (2), gyro
// (1) and SFLP game rotation / quaternion (0x13). The accel tag is the clock: it
// drives windowing, the AHRS, and all ten decimation filters, so every delay line
// advances exactly once per accel sample and they can never drift apart. Gyro and
// SFLP words only latch their latest value for the accel branch to pair with.
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
      // Squared magnitude only; the sqrt is taken once per debug print. Rooting per
      // sample cost ~960 double sqrt/s here and as many again in the gyro branch,
      // for a line of serial output.
      sumAccMag2_ += (double)a[0] * a[0] + (double)a[1] * a[1] + (double)a[2] * a[2];
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
        winNAcc_ = 0;
        winBraking_ = false;
        winSflpNan_ = false;
        winFifoOvf_ = false;
        winFirDone_ = false;
      }
      winNAcc_++;
      // Brake flag with debounce: LINEAR accel (gravity removed) must stay over
      // threshold for kBrakeMinSamples in a row. Rotate body accel to world/NED
      // with the SFLP quaternion (world = R(q).a_body), then subtract gravity.
      float qw = latestQw_, qx = latestQx_, qy = latestQy_, qz = latestQz_;
      float ax = (float)a[0], ay = (float)a[1], az = (float)a[2];
      float wX = (1 - 2 * (qy * qy + qz * qz)) * ax + 2 * (qx * qy - qw * qz) * ay + 2 * (qx * qz + qw * qy) * az;
      float wY = 2 * (qx * qy + qw * qz) * ax + (1 - 2 * (qx * qx + qz * qz)) * ay + 2 * (qy * qz - qw * qx) * az;
      float wZ = 2 * (qx * qz - qw * qy) * ax + 2 * (qy * qz + qw * qx) * ay + (1 - 2 * (qx * qx + qy * qy)) * az;
      wZ -= 1000.0f;  // remove 1 g gravity
      double aMag2 = (double)wX * wX + (double)wY * wY + (double)wZ * wZ;
      if (aMag2 > kBrakeThresholdMg2) {
        if (brakeRun_ < 0xFFFF) brakeRun_++;
        if (brakeRun_ >= kBrakeMinSamples) winBraking_ = true;
      } else {
        brakeRun_ = 0;
      }

      // ---- AHRS on the raw stream ----
      // Accel and gyro arrive as separate FIFO tags, so the update is paired with
      // the freshest gyro word: at most one sample of skew (1.04 ms @ 960 Hz).
      // The AHRS takes SI units - KalmanAhrs' adaptive R weighs |a| against gravity,
      // so in mg every sample would look like a 100 g slam. Madgwick normalises and
      // does not care, which is why this stays filter-agnostic.
      const float axS = ax * kMg2Ms2, ayS = ay * kMg2Ms2, azS = az * kMg2Ms2;
      const float sflpQ[4] = {latestQw_, latestQx_, latestQy_, latestQz_};
      if (!ahrsSeeded_) {
        ahrs_.initFromAccel(axS, ayS, azS);
        ahrsSeeded_ = true;
        ahrsN_ = 0;
        qDelay_.reset(ahrs_.quaternion(), sflpQ);
      } else if (++ahrsN_ >= kAhrsDiv) {
        const uint32_t t0 = micros();
        ahrs_.update(latestGx_ * kMdps2Rads, latestGy_ * kMdps2Rads, latestGz_ * kMdps2Rads,
                     axS, ayS, azS, (float)(kAhrsDiv * samplePeriodMs_) * 1.0e-3f);
        usAhrs_ += micros() - t0;
        ahrsN_ = 0;
        // The attitude is not filtered, but it is carried back by the same group
        // delay the FIR imposes on ax..gz, so the whole row describes one instant.
        qDelay_.push(ahrs_.quaternion(), sflpQ);
      }

      // The vertical accel is recomputed on EVERY raw sample from the latest
      // quaternion: the attitude is slow, but the acceleration is the signal that
      // has to keep its full bandwidth going into the antialias filter.
      const float vacc = wave_use_sflp
                             ? (wZ * kMg2Ms2)
                             : verticalAccel(ahrs_.quaternion(), axS, ayS, azS, kGravity);
      fir_.push(ax, ay, az, wX, wY, wZ,
                latestGx_, latestGy_, latestGz_, vacc);

      // Output sample: the first raw sample to reach the window centre. Same
      // convention as fir.py's dec//2 - the value sits in the middle of the window
      // it represents. 960/100 = 9.6 is not an integer decimation, so the output
      // grid stays time-driven and the residual jitter is at most half a raw period.
      if (!winFirDone_ && tms >= (uint32_t)curWinIdx_ * kWindowMs + kFirS1CenterMs) {
        latchRowValues();
      }

      sampleTms_ += samplePeriodMs_;
      accelIdx_++;
    } else if (tag == 1) {  // gyro (mdps)
      int32_t g[3];
      imu_.FIFO_Get_G_Axes(g);
      sumGyrMag2_ += (double)g[0] * g[0] + (double)g[1] * g[1] + (double)g[2] * g[2];
      nGyrDbg_++;
      latestGx_ = (float)g[0]; latestGy_ = (float)g[1]; latestGz_ = (float)g[2];
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
  // RMS, not the mean of the magnitudes - the per-sample sqrt is what was removed.
  // For a near-constant |a| (which is the case here: ~1 g) the two agree closely,
  // and the number is only ever read as a sanity check.
  double avgAcc = nAccDbg_ ? sqrt(sumAccMag2_ / nAccDbg_) : 0.0;
  double avgGyr = nGyrDbg_ ? sqrt(sumGyrMag2_ / nGyrDbg_) : 0.0;

  dbg.print("[IMU] Accel: "); dbg.print(accHz, 0);
  dbg.print(" Hz, |a| = ");   dbg.print(avgAcc, 1);
  dbg.print(" mg  |  Gyro: "); dbg.print(gyrHz, 0);
  dbg.print(" Hz, |g| = ");   dbg.print(avgGyr, 1);
  dbg.print(" mdps  |  cpu: ahrs ");
  dbg.print(elapsedS > 0 ? usAhrs_ / (elapsedS * 1.0e4) : 0.0, 1);   // us/s -> %
  dbg.print("% fir ");
  dbg.print(elapsedS > 0 ? usFir_ / (elapsedS * 1.0e4) : 0.0, 1);
  dbg.print("%");
  if (nOverflow_ > 0) {
    dbg.print("  [WARN] FIFO overflow x"); dbg.print(nOverflow_);
    nOverflow_ = 0;
  }
  dbg.println();

  nAccDbg_ = nGyrDbg_ = 0;
  sumAccMag2_ = sumGyrMag2_ = 0.0;
  usAhrs_ = usFir_ = 0;
  dbgLastPrint_ = now;
}
