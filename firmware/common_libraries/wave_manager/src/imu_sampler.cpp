#include "imu_sampler.h"
#include "config.h"   // SPI_CS_IMU_PIN and the shared SPI pin definitions
#include "rotation.h" // verticalAccel
#include <math.h>

/*
  Ported from ORB_test/src/Imu.cpp. Differences for the buoy:
   - shares the global Arduino SPI object (sd_writer already called SPI.begin() on
     SPI1) instead of owning a private SPIClass;
   - INT1 is routed on the PCB but not used by the firmware: update() polls and
     drains whatever the FIFO holds, so the drain cadence is the main loop's rather
     than the sensor's. kFifoWatermark survived the port from ORB_test but nothing
     applies it, so it is written to cfg.csv while describing nothing. ORB_test's
     Imu.cpp has the working version behind kWakeMode == WakeMode::Interrupt
     (FIFO_Set_Watermark_Level + INT1_CTRL bit 3 on kLsmInt1 = PB12), including the
     re-arm guard the level-driven watermark interrupt needs after a blocking SD
     flush. Porting that is the lever if polling proves too coarse;
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
  r.axnSflp = nx_.eval(); r.aynSflp = ny_.eval(); r.aznSflp = nz_.eval();
  r.gx = gx_.eval(); r.gy = gy_.eval(); r.gz = gz_.eval();
  r.vaccFir = vacc_.eval();
  // The SFLP vertical accel is the world-Z channel in m/s^2; filtering aznSflp and
  // scaling is identical to filtering the scaled series (the filter is linear), so
  // there is no eleventh delay line for it.
  r.vaccSflpFir = r.aznSflp * kMg2Ms2;
  // Unfiltered counterparts, read straight off the delay lines' centre taps - the
  // same instant the filtered values are centred on, at no extra cost.
  r.vacc = vacc_.center();
  r.vaccSflp = nz_.center() * kMg2Ms2;
}

// -----------------------------------------------------------------------------
// ImuSampler
// -----------------------------------------------------------------------------
ImuSampler *ImuSampler::s_self = nullptr;

// Runs in interrupt context: set the flag and get out. Reading the FIFO from here
// would take the SPI bus away from whatever the main loop was doing with it.
void ImuSampler::isrTrampoline() {
  if (s_self) s_self->fifoFlag_ = true;
}

ImuSampler::ImuSampler()
    : imu_(&SPI, (int)SPI_CS_IMU_PIN, kImuSpiHz) { s_self = this; }

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

  if (kImuUseInt1) {
    // Let INT1 go high once the FIFO holds kFifoWatermark words, so the drain runs
    // on the sensor's cadence rather than the main loop's. INT1_CTRL has no setter
    // in the wrapper, hence the raw register write (see wave_config.h).
    imu_.FIFO_Set_Watermark_Level((uint8_t)kFifoWatermark);
    imu_.Write_Reg(kInt1CtrlReg, kInt1FifoTh);
    // The sensor drives INT1 push-pull, active high -> trigger on the rising edge.
    pinMode(INT1_IMU_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT1_IMU_PIN), isrTrampoline, RISING);
  }

  return true;
}

// STREAM_MODE. FIFO_MODE was tried on 2026-08-04 and had to be reverted - the reason
// is worth keeping, because FIFO_MODE looks like the safer choice and is not.
//
// The problem it was meant to solve: a full FIFO in STREAM mode discards its OLDEST
// word to make room, which is the word the drain is in the middle of reading. With the
// driver's split read (tag at 0x78, payload at 0x79..0x7E in a SECOND transaction) a
// discard between the two paired one sample's tag with another's payload; an accel
// word decoded with gyro sensitivity reads ~287000 mdps, which is the |g| that showed
// up whenever fifo_ovf was set.
//
// That is fixed at the source now: readFifoWord() takes tag and payload in one burst,
// so the word cannot be split at all. FIFO_MODE was defence against a bug that no
// longer exists - and it brought a failure mode of its own. FIFO_MODE STOPS collecting
// when full and only a trip through BYPASS restarts it, so recovery hinges on noticing
// that it stopped. The only signal for that is FIFO_FULL_IA, and the datasheet defines
// it as "FIFO will be full at the NEXT ODR" - predictive, not a state. Once collection
// has stopped there is no next write, the flag de-asserts, the restart never fires, and
// INT1 stays low forever. Measured symptom: the stream drops to 0 Hz and never returns,
// with no overflow warning at all, because the counter never incremented either.
//
// STREAM mode cannot get stuck: it never stops, so there is nothing to restart. An
// overrun costs samples and nothing else. This is also what ORB_test's Imu.cpp has
// always done - it counts overflows and never resets the FIFO.
//
// resetFifo() is therefore a CAPTURE-START operation only, not a recovery path.
void ImuSampler::startStreaming() {
  imu_.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
}

void ImuSampler::resetFifo() {
  imu_.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);  // flush hardware FIFO
  imu_.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);  // resume streaming
  // The buffer is empty now, so any pending flag refers to words that no longer
  // exist. Leaving it set would make the next update() drain nothing and clear it -
  // harmless, but it would also mask a genuinely dead interrupt line.
  fifoFlag_ = false;
}

void ImuSampler::resetWindowing(uint32_t captureStartMs) {
  sessionStartMs_ = captureStartMs;
  nOverflowTotal_ = 0;       // per-capture, reported in ana.csv
  lastDrainMs_ = captureStartMs;  // measure the INT1 deadline from t=0, not from boot
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
  // could carry the previous capture's attitude into q*_sflp, ax_ned_sflp..az_ned_sflp and the
  // brake flag until the first 0x13 word arrived. Identity is the honest start.
  latestQw_ = 1; latestQx_ = latestQy_ = latestQz_ = 0;
  latestGx_ = latestGy_ = latestGz_ = 0;
  // The AHRS re-seeds from gravity on the first accel sample (ahrsSeeded_), which is
  // also where the quaternion delay line gets filled.
  ahrs_.reset();
  ahrsSeeded_ = false;
  fir_.reset();
  dbgLastPrint_ = captureStartMs;
  nAccDbg_ = nGyrDbg_ = 0;
  sumAccMag2_ = sumGyrMag2_ = 0.0;
}

// Evaluate the decimation filters and read the delay-matched quaternions into the
// row being assembled. Everything written here refers to one instant: the centre of
// the current window, carried back by the FIR group delay.
void ImuSampler::latchRowValues() {
  fir_.eval(pendingRow_);
  float mq[4], sq[4];
  qDelay_.read(mq, sq);
  pendingRow_.qw = mq[0]; pendingRow_.qx = mq[1];
  pendingRow_.qy = mq[2]; pendingRow_.qz = mq[3];
  pendingRow_.qwSflp = sq[0]; pendingRow_.qxSflp = sq[1];
  pendingRow_.qySflp = sq[2]; pendingRow_.qzSflp = sq[3];
  if (!isfinite(pendingRow_.vaccFir)) pendingRow_.vaccFir = 0.0f;
  if (!isfinite(pendingRow_.vaccSflpFir)) pendingRow_.vaccSflpFir = 0.0f;
  if (!isfinite(pendingRow_.vacc)) pendingRow_.vacc = 0.0f;
  if (!isfinite(pendingRow_.vaccSflp)) pendingRow_.vaccSflp = 0.0f;
  winFirDone_ = true;
}

// -----------------------------------------------------------------------------
// Raw multi-byte burst read on the shared SPI bus. With IF_INC = 1 (the default) the
// sensor auto-increments the address, so consecutive registers arrive in ONE CS-low
// transfer. Ported from ORB_test's Imu::imuBurstRead; the only difference is that
// this class shares the global SPI object rather than owning an SPIClass.
//
// Settings must match what the driver uses for its own reads or the two would
// disagree about the bus: MODE3, MSB first, 0x80 as the read bit.
// -----------------------------------------------------------------------------
void ImuSampler::imuBurstRead(uint8_t startReg, uint8_t *buf, uint8_t len) {
  SPI.beginTransaction(SPISettings(kImuSpiHz, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI_CS_IMU_PIN, LOW);
  SPI.transfer(startReg | 0x80);  // 0x80 = READ bit
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(SPI_CS_IMU_PIN, HIGH);
  SPI.endTransaction();
}

// Pop one FIFO word: the tag byte at 0x78 and its six payload bytes at 0x79..0x7E,
// in a single transaction. Reading 0x7E is what advances the FIFO, so the whole word
// leaves the sensor atomically.
//
// This is not only faster than the wrapper's FIFO_Get_Tag + FIFO_Get_X_Axes pair - it
// is the only version that is CORRECT while the FIFO is under pressure. Two separate
// transactions leave a window in which the buffer can move on between the tag read
// and the payload read, pairing one sample's tag with another's data; an accel word
// decoded with gyro sensitivity reads ~287000 mdps, which is the |g| that showed up
// whenever fifo_ovf was set. FIFO_MODE closes that window from the sensor side and
// this closes it from the bus side.
//
// Returns tag_sensor, the top 5 bits of the tag byte (FIFO_DATA_OUT_TAG: bit 0 unused,
// bits 2:1 tag_cnt, bits 7:3 tag_sensor).
uint8_t ImuSampler::readFifoWord(uint8_t payload[6]) {
  uint8_t w[7];
  imuBurstRead(kFifoDataOutTagReg, w, 7);
  for (uint8_t i = 0; i < 6; i++) payload[i] = w[i + 1];
  return (uint8_t)(w[0] >> 3);
}

// Payload -> three int16 in LSB order. The driver truncates its own conversion to
// int32 (FIFO_Get_X_Axes returns whole mg), which throws away the sub-LSB range the
// sensitivity actually provides; decoding here keeps it in float.
static inline void payloadToAxes(const uint8_t p[6], float sens, float out[3]) {
  for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = (int16_t)((uint16_t)p[2 * i] | ((uint16_t)p[2 * i + 1] << 8));
    out[i] = (float)raw * sens;
  }
}

// IEEE half -> float, bit for bit as the driver's npy_halfbits_to_floatbits does.
// The SFLP words are three halves; the driver's own decoder is private, so it is
// mirrored rather than called.
static inline float halfToFloat(uint16_t h) {
  union { float f; uint32_t b; } c;
  const uint32_t sgn = ((uint32_t)h & 0x8000u) << 16;
  const uint16_t exp = h & 0x7c00u;
  if (exp == 0x0000u) {            // zero or subnormal
    uint16_t sig = h & 0x03ffu;
    if (sig == 0) { c.b = sgn; return c.f; }
    uint16_t e = 0;
    sig <<= 1;
    while ((sig & 0x0400u) == 0) { sig <<= 1; e++; }
    c.b = sgn + (((uint32_t)(127 - 15 - e)) << 23) + (((uint32_t)(sig & 0x03ffu)) << 13);
  } else if (exp == 0x7c00u) {     // inf or NaN
    c.b = sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
  } else {                         // normalised
    c.b = sgn + ((((uint32_t)(h & 0x7fffu)) + 0x1c000u) << 13);
  }
  return c.f;
}

// SFLP game rotation vector: x,y,z as halves, w reconstructed from the unit norm.
// Same reconstruction as the driver's sflp2q, including the renormalisation guard
// for a sum of squares that rounds above 1. Output is [x,y,z,w] to match the
// wrapper's FIFO_Get_Rotation_Vector, which is what the caller already expects.
static inline void payloadToQuat(const uint8_t p[6], float q[4]) {
  float sumsq = 0.0f;
  for (uint8_t i = 0; i < 3; i++) {
    q[i] = halfToFloat((uint16_t)p[2 * i] | ((uint16_t)p[2 * i + 1] << 8));
    sumsq += q[i] * q[i];
  }
  if (sumsq > 1.0f) {
    const float n = sqrtf(sumsq);
    q[0] /= n; q[1] /= n; q[2] /= n;
    sumsq = 1.0f;
  }
  q[3] = sqrtf(1.0f - sumsq);
}

// Close the current window -> finish the ImuRow and hand it to the row sink.
void ImuSampler::closeWindow() {
  if (winNAcc_ == 0) return;
  // No raw sample reached the window centre - a FIFO gap. Read the filters at the
  // window edge instead of dropping the row: the filtered value is still valid, it
  // is just centred up to kRowPeriodMs late. Counted so a capture can be judged.
  if (!winFirDone_) {
    latchRowValues();
    nFirLateEval_++;
  }
  pendingRow_.winStartMs = (uint32_t)curWinIdx_ * kRowPeriodMs;
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
  // INT1 gate, with a deadline. The interrupt is a hint about WHEN to drain, never
  // the authority on WHETHER to: a single lost edge would otherwise stop the capture
  // permanently, which is exactly what was measured on 2026-08-04 (0 Hz, forever, no
  // warning). Past kFifoPollFallbackMs the drain runs regardless and costs two
  // register reads if there was genuinely nothing there.
  if (kImuUseInt1 && !fifoFlag_ && (millis() - lastDrainMs_) < kFifoPollFallbackMs) {
    // Still print, so a stalled interrupt shows up as frozen counters, not silence.
    debugPrintStatus(dbg);
    return;
  }
  fifoFlag_ = false;

  // Diagnostic only - in STREAM mode there is nothing to recover from, the sensor
  // just overwrote its oldest words. Note FIFO_FULL_IA is predictive ("full at the
  // next ODR"), so treat a zero here as "probably fine", never as proof: r.n and
  // fir_late_eval_windows are the ground truth for whether samples went missing.
  uint8_t full = 0;
  imu_.FIFO_Get_Full_Status(&full);
  if (full) {
    nOverflow_++;       // reset by the debug print; nOverflowTotal_ is the per-capture one
    nOverflowTotal_++;
    winFifoOvf_ = true; // flag the window that was open when it happened (fifo_ovf in imu.csv)
  }

  uint16_t nSamples = 0;
  imu_.FIFO_Get_Num_Samples(&nSamples);

  for (uint16_t i = 0; i < nSamples; i++) {
    uint8_t payload[6];
    const uint8_t tag = readFifoWord(payload);   // tag + data, one transfer
    if (tag == 2) {  // accel (mg)
      float a[3];
      payloadToAxes(payload, kAccSensMgPerLsb, a);
      // Squared magnitude only; the sqrt is taken once per debug print. Rooting per
      // sample cost ~960 double sqrt/s here and as many again in the gyro branch,
      // for a line of serial output.
      sumAccMag2_ += (double)a[0] * a[0] + (double)a[1] * a[1] + (double)a[2] * a[2];
      nAccDbg_++;
      // Windowing: samples arrive in FIFO bursts but represent evenly spaced points
      // in time. A running, monotonic clock (sampleTms_ += samplePeriodMs_) gives a
      // steady kRowPeriodMs binning; samplePeriodMs_ self-calibrates below so the axis
      // tracks the wall clock without drift.
      uint32_t tms = (uint32_t)sampleTms_;
      int32_t widx = (int32_t)(tms / kRowPeriodMs);
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
        qDelay_.reset(ahrs_.quaternion(), sflpQ);
      } else {
        // One update per raw sample - no divider. dt comes from samplePeriodMs_,
        // which self-calibrates against the wall clock, so a slow drain shows up as
        // a longer dt rather than as a silently wrong integration rate.
        ahrs_.update(latestGx_ * kMdps2Rads, latestGy_ * kMdps2Rads, latestGz_ * kMdps2Rads,
                     axS, ayS, azS, (float)samplePeriodMs_ * 1.0e-3f);
        // The attitude is not filtered, but it is carried back by the same group
        // delay the FIR imposes on ax..gz, so the whole row describes one instant.
        // At one step per sample that delay is kFirHalf pushes exactly.
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
      if (!winFirDone_ && tms >= (uint32_t)curWinIdx_ * kRowPeriodMs + kFirS1CenterMs) {
        latchRowValues();
      }

      sampleTms_ += samplePeriodMs_;
      accelIdx_++;
    } else if (tag == 1) {  // gyro (mdps)
      float g[3];
      payloadToAxes(payload, kGyrSensMdpsPerLsb, g);
      sumGyrMag2_ += (double)g[0] * g[0] + (double)g[1] * g[1] + (double)g[2] * g[2];
      nGyrDbg_++;
      latestGx_ = g[0]; latestGy_ = g[1]; latestGz_ = g[2];
    } else if (tag == kSflpGameRotationTag) {  // quaternion [x,y,z,w]
      float q[4];
      payloadToQuat(payload, q);
      // NaN guard: a corrupt FIFO word decodes to an invalid half-float and the
      // w = sqrt(1 - sumsq) reconstruction yields NaN; one NaN poisons az_ned_sflp and the
      // whole SFLP spectrum. Keep the last valid quaternion instead.
      if (isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) && isfinite(q[3])) {
        latestQx_ = q[0]; latestQy_ = q[1]; latestQz_ = q[2]; latestQw_ = q[3];
      } else {
        winSflpNan_ = true;
      }
    }
    // Unknown tag: nothing to do. readFifoWord() already popped the word, so the
    // FIFO pointer has advanced either way - unlike the old path, which needed an
    // explicit FIFO_Get_Data to avoid stalling on a tag it did not recognise.
  }

  lastDrainMs_ = millis();

  if (kImuUseInt1) {
    // Robustness against a lost rising edge. The watermark interrupt is LEVEL
    // driven: if this drain ended with the FIFO still above the watermark - after a
    // blocking SD flush, say - the line never falls, no new RISING edge ever
    // arrives, fifoFlag_ is never set again and the stream dies silently. Re-arm
    // ourselves whenever a backlog remains. ORB_test found this the hard way; it is
    // not defensive padding.
    uint16_t remaining = 0;
    imu_.FIFO_Get_Num_Samples(&remaining);
    if (remaining >= kFifoWatermark) fifoFlag_ = true;
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
  dbg.print(" mdps");
  if (nOverflow_ > 0) {
    dbg.print("  [WARN] FIFO overflow x"); dbg.print(nOverflow_);
    nOverflow_ = 0;
  }
  dbg.println();

  nAccDbg_ = nGyrDbg_ = 0;
  sumAccMag2_ = sumGyrMag2_ = 0.0;
  dbgLastPrint_ = now;
}
