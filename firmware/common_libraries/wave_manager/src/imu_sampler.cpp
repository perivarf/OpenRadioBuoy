#include "imu_sampler.h"
#include "config.h"
#include "rotation.h"
#include <math.h>

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

  // Eval gives value at centre of tap, so the series are aligned 
  // for both filtered and unfiltered.

  // Filtered
  r.ax = ax_.eval(); r.ay = ay_.eval(); r.az = az_.eval();
  r.axnSflp = nx_.eval(); r.aynSflp = ny_.eval(); r.aznSflp = nz_.eval();
  r.gx = gx_.eval(); r.gy = gy_.eval(); r.gz = gz_.eval();
  r.vaccFir = vacc_.eval();
  r.vaccSflpFir = r.aznSflp * kMg2Ms2;
  
  // Unfiltered
  r.vacc = vacc_.center();
  r.vaccSflp = nz_.center() * kMg2Ms2;
}

// -----------------------------------------------------------------------------
// ImuSampler
// -----------------------------------------------------------------------------
ImuSampler *ImuSampler::s_self = nullptr;

// Runs in interrupt context: Interrupt service routine sets the flag and returns. 
// Reading the FIFO from here would be a bad idea
void ImuSampler::isrTrampoline() {
  if (s_self) s_self->fifoFlag_ = true;
}

ImuSampler::ImuSampler()
    : imu_(&SPI, (int)SPI_CS_IMU_PIN, kImuSpiHz) { s_self = this; }

bool ImuSampler::begin(Print &dbg) {

  // Keep CS high before any bus activity
  pinMode(SPI_CS_IMU_PIN, OUTPUT);
  digitalWrite(SPI_CS_IMU_PIN, HIGH);

  if (imu_.begin() != LSM6DSV16X_OK) {
    dbg.println("LSM6DSV: begin() failed - check SPI wiring/CS");
    return false;
  }
  dbg.println("LSM6DSV: begin() OK");

  applyConfig();

  if (kImuUseInt1) {
    
    // Need to set pinMode() before attachInterrupt() or the ISR never fires.
    pinMode(INT1_IMU_PIN, INPUT);

    // The sensor drives INT1 push-pull, active high -> trigger on the rising edge.
    attachInterrupt(digitalPinToInterrupt(INT1_IMU_PIN), isrTrampoline, RISING);
  }

  return true;
}

// All sensor config
void ImuSampler::applyConfig() {

  // Enable accelerometer and gyroscope
  imu_.Enable_X();      // accelerometer
  imu_.Enable_G();      // gyroscope

  // Set ranges, ODR and mode (high accuracy, low power etc) from wave_config.h
  imu_.Set_X_FS((int32_t)kAccelFS);   // full-scale from wave_config (kAccelFS)
  imu_.Set_G_FS((int32_t)kGyroFS);    // full-scale from wave_config (kGyroFS)
  imu_.Set_X_ODR((float)kImuOdrHz, kImuAccMode);
  imu_.Set_G_ODR((float)kImuOdrHz, kImuGyrMode);

  // Accelerometer low-pass filter
  if (kUseLpf2) {
    imu_.Set_X_Filter_Mode(0, kLpf2Bw);  // 0 => low-pass mode, arg2 = bandwidth
  }

  // Batch accel + gyro into the FIFO at the chosen ODR; SFLP rotation vector
  // (quaternion) is batched alongside them. Stream mode is started separately.
  imu_.FIFO_Set_X_BDR((float)kImuOdrHz);
  imu_.FIFO_Set_G_BDR((float)kImuOdrHz);

  // SFLP fusion is AHRS filter built into the LSM6DSV16X.
  // It is a quaternion that rotates the sensor frame into the
  // gravitation frame
  //
  // SFLP can return estimated gyro bias
  // Can inject gyro bias at start (if known) to improve convergence
  // Set_SFLP_GBIAS(float x, float y, float z)  // rad/s
  if (kEnableSflp) {
    imu_.Enable_Rotation_Vector();
    imu_.Set_SFLP_ODR(kSflpOdrHz);
    imu_.Set_SFLP_Batch(true, false, false);  // (Rotation, Gravity, gBias) -> FIFO
  } else {
    imu_.Disable_Rotation_Vector();
    imu_.Set_SFLP_Batch(false, false, false);
  }

  // Interrupt watermark
  if (kImuUseInt1) {
    // Let INT1 go high once the FIFO holds kFifoWatermark words, so the drain runs
    // on the sensor's cadence rather than the main loop's. INT1_CTRL has no setter
    // in the wrapper, hence the raw register write (see wave_config.h).
    imu_.FIFO_Set_Watermark_Level((uint8_t)kFifoWatermark);
    imu_.Write_Reg(kInt1CtrlReg, kInt1FifoTh);
  }
}

// Start streaming using STREAM_MODE. Both STREAM_MODE and FIFO_MODE use the same FIFO buffer
// but they differ when it fills. STREAM overwrites the oldest
// word and keeps running. FIFO stops until a BYPASS trip restarts it
// STREAM_MODE is more stable as it allows for some missing reads, but it continues
// to fill the FIFO
void ImuSampler::startStreaming() {
  fifoFlag_ = false;
  imu_.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
}


void ImuSampler::shutdownIMU() {
  imu_.Disable_G();
  imu_.Disable_X();
}

// Check if acceleration sensor 
bool ImuSampler::checkImu(Print &dbg) {

  uint8_t drdy = 0;
  const uint32_t deadline = millis() + 100;
  while (millis() < deadline) {
    if (imu_.Get_X_DRDY_Status(&drdy) == LSM6DSV16X_OK && drdy) break;
  }

  // Read one accel sample, then put back to sleep
  int32_t a[3] = {0, 0, 0};
  const bool read_ok = (imu_.Get_X_Axes(a) == LSM6DSV16X_OK);

  // Shutdown IMU after read, so it is left OFF until next capture.
  shutdownIMU();

  if (!drdy) {
    dbg.println("LSM6DSV: no data-ready within 100 ms - sensor is not converting");
    return false;
  }
  if (!read_ok) {
    dbg.println("LSM6DSV: sample read failed");
    return false;
  }

  return true;
}

// Flush the hardware FIFO and leave it OFF. Streaming is startStreaming's job
void ImuSampler::resetFifo() {
  imu_.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);  // flush hardware FIFO, stay idle
  fifoFlag_ = false;
  // Belongs to the FIFO contents being thrown away here, so carrying it across would
  // pin a previous capture's overrun on the first window of the next one.
  pendingOvrLatched_ = false;
}

void ImuSampler::resetWindowing(uint32_t captureStartMs) {
  sessionStartMs_ = captureStartMs;
  nOverflowTotal_ = 0;       // per-capture, reported in ana.csv
  nOverflow_ = 0;
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
  nWordsDbg_ = nUnknownDbg_ = nFullDbg_ = 0;
  maxLevelDbg_ = 0;
  lastUnknownTag_ = 0;
}

// Evaluate the decimation filters and read the delay-matched quaternions into the
// row being assembled. Everything here should referr to the same point in time instant:
// the centre of the current window
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

// Raw multi-byte burst read on the shared SPI bus. With IF_INC = 1 (the default) the
// sensor auto-increments the address, so consecutive registers arrive in ONE CS-low
// transfer. 
// Settings must match what the driver uses for its own reads or the two would
// disagree about the bus: MODE3, MSB first, 0x80 as the read bit.
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
//   w[0..6] = tag | x_lo x_hi | y_lo y_hi | z_lo z_hi     (three LE int16, tag >> 3 = sensor)
//
// One burst rather than the wrapper's FIFO_Get_Tag + FIFO_Get_X_Axes
//
// Returns tag_sensor, the top 5 bits of the tag byte (FIFO_DATA_OUT_TAG: bit 0 unused,
// bits 2:1 tag_cnt, bits 7:3 tag_sensor).
uint8_t ImuSampler::readFifoWord(uint8_t payload[6]) {
  uint8_t w[7];
  imuBurstRead(kFifoDataOutTagReg, w, 7);
  
  // place payload bytes into the caller's buffer
  for (uint8_t i = 0; i < 6; i++) payload[i] = w[i + 1];
  
  // return tag (top 5 bits)
  return (uint8_t)(w[0] >> 3);
}

// Raw log. Little-endian, layout documented at wave_raw_log in wave_config.h.
void ImuSampler::rawAppend(const uint8_t *p, uint8_t n) {
  if (!rawSink_) return;
  for (uint8_t i = 0; i < n; i++) {
    rawBuf_[rawLen_++] = p[i];

    // Check if rawBuf_ is full
    if (rawLen_ >= kRawBlockBytes) {
      

      if (!rawSink_(rawBuf_, rawLen_)) {
        nRawWriteFail_++;
        rawWriteFailPending_ = true;
      }
      rawLen_ = 0;
    }
  }
}

// The FIFO word exactly as it came off the bus. The
void ImuSampler::rawEmitWord(uint8_t tag, const uint8_t payload[6]) {
  if (!rawSink_) return;
  uint8_t rec[kRawWordBytes];
  rec[0] = tag;
  for (uint8_t i = 0; i < 6; i++) rec[i + 1] = payload[i];
  rawAppend(rec, kRawWordBytes);
}

// One per drain, written BEFORE that drain's words. Pins the sample axis to the
// clock: t_us is the fractional sample time (sampleTms_ is a self-calibrating
// double), accel_n is the cumulative accel count so a gap is arithmetic rather than
// guesswork, and millis is when the drain actually ran - which is the measurement
// that says whether SD stalls are threatening the FIFO.
void ImuSampler::rawEmitSync(uint16_t nWords, uint16_t flags) {
  if (!rawSink_) return;
  uint8_t rec[kRawSyncBytes];
  uint8_t o = 0;
  rec[o++] = kRawSyncTag;
  const uint32_t tUs = (uint32_t)(sampleTms_ * 1000.0);
  const uint32_t ms  = millis();
  const uint32_t vals[3] = {tUs, accelIdx_, ms};
  for (uint8_t v = 0; v < 3; v++) {
    rec[o++] = (uint8_t)(vals[v]);
    rec[o++] = (uint8_t)(vals[v] >> 8);
    rec[o++] = (uint8_t)(vals[v] >> 16);
    rec[o++] = (uint8_t)(vals[v] >> 24);
  }
  // Fold in any block lost since the last sync. This is the only place the loss can
  // be reported IN the stream, and it must be reported there: ana.csv can say a
  // capture lost blocks, but not WHERE - and where is the whole question, because
  // everything after the first loss is misaligned.
  if (rawWriteFailPending_) flags |= kRawFlagWriteFail;
  const uint16_t vals16[2] = {nWords, flags};
  for (uint8_t v = 0; v < 2; v++) {
    rec[o++] = (uint8_t)(vals16[v]);
    rec[o++] = (uint8_t)(vals16[v] >> 8);
  }
  const uint32_t failBefore = nRawWriteFail_;
  rawAppend(rec, kRawSyncBytes);
  // Clear only if appending the report did not itself lose a block. When it did, the
  // flag stays pending and the NEXT sync carries it - one record late in the file, but
  // never silently dropped. Late-but-present is the right way round: the decoder's job
  // is to stop trusting the tail, and it still does.
  if (nRawWriteFail_ == failBefore) rawWriteFailPending_ = false;
}

void ImuSampler::flushRaw(void) {
  if (rawSink_ && rawLen_ > 0) {
    if (!rawSink_(rawBuf_, rawLen_)) {
      nRawWriteFail_++;
      rawWriteFailPending_ = true;
    }
    rawLen_ = 0;
  }
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

// SFLP rotation vector: x,y,z as halves, w reconstructed from the unit norm.
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
// (1) and SFLP rotation / quaternion (0x13). The accel tag is the clock: it
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

  // Level and flags from one read, so the count and the flags describe the same
  // instant - see readFifoStatus for the register and why it is not the wrapper's.
  const FifoStatus st = readFifoStatus();
  const uint16_t nSamples = st.level;

  // Loss is OVR, not FULL. Counting FULL as loss over-reports - the brim can be reached
  // and drained in time - which cuts both ways: it also means a capture reporting ZERO
  // is strong evidence the level never even approached the top.
  //
  // pendingOvrLatched_ belongs to the PREVIOUS drain's pop loop, so it is reported one
  // drain late. That is a coarser timestamp, not a false positive: the loss is real, and
  // there is no earlier read that could have carried it.
  const bool lost = st.ovr || st.ovrLatched || pendingOvrLatched_;
  pendingOvrLatched_ = false;
  if (lost) {
    nOverflow_++;       // reset by the debug print; nOverflowTotal_ is the per-capture one
    nOverflowTotal_++;
    winFifoOvf_ = true; // flag the window that was open when it happened (fifo_ovf in imu.csv)
  }
  if (st.full) nFullDbg_++;

  // Peak level over the debug interval
  if (nSamples > maxLevelDbg_) maxLevelDbg_ = nSamples;

  // Sync record first, so the words that follow it are the ones it describes.
  rawEmitSync(nSamples, lost ? kRawFlagFifoOvf : 0);

  for (uint16_t i = 0; i < nSamples; i++) {
    uint8_t payload[6];
    const uint8_t tag = readFifoWord(payload);   // tag + data, one transfer
    nWordsDbg_++;   // every word popped, whatever it turns out to be
    // EVERY word, including tags this code does not decode: the raw log is a record
    // of what the sensor produced, not of what the wave chain happens to consume.
    rawEmitWord(tag, payload);
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
      const float ax = (float)a[0], ay = (float)a[1], az = (float)a[2];

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

      // World-frame linear accel from the SFLP quaternion, gravity removed. These are
      // the ax_ned_sflp / vacc_sflp columns and nothing else - the wave chain reads
      // them only when wave_use_sflp is set.
      //
      // Left at zero when the fusion block is off. The delay lines start zeroed and
      // nothing but this pushes them, so zeros here make every _sflp column read zero
      // rather than body-frame values wearing a world-frame name. cfg.csv's
      // sflp_enabled is what tells the two zeros apart afterwards.
      float wX = 0.0f, wY = 0.0f, wZ = 0.0f;
      if (kEnableSflp) {
        float w[3];
        rotateBodyToWorld(sflpQ, ax, ay, az, w);
        wX = w[0]; wY = w[1]; wZ = w[2] - 1000.0f;   // remove 1 g, mg units
      }

      // Brake flag with debounce: LINEAR accel (gravity removed) must stay over
      // threshold for kBrakeMinSamples in a row.
      //
      // Rotated with the orientation the build actually trusts, not with SFLP
      // unconditionally. It used to be the latter, which let a NaN or an unconverged
      // SFLP poison a quality flag on a build whose measurement came from Madgwick -
      // the flag and the measurement now stand or fall together.
      //
      // Placed after the AHRS update so it sees this sample's attitude rather than the
      // previous one, and so the very first sample uses the seeded attitude from
      // initFromAccel instead of an uninitialised default.
      float bX = wX, bY = wY, bZ = wZ;
      if (!wave_use_sflp) {
        float b[3];
        rotateBodyToWorld(ahrs_.quaternion(), ax, ay, az, b);
        bX = b[0]; bY = b[1]; bZ = b[2] - 1000.0f;
      }
      const double aMag2 = (double)bX * bX + (double)bY * bY + (double)bZ * bZ;
      if (aMag2 > kBrakeThresholdMg2) {
        if (brakeRun_ < 0xFFFF) brakeRun_++;
        if (brakeRun_ >= kBrakeMinSamples) winBraking_ = true;
      } else {
        brakeRun_ = 0;
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
    } else if (tag == kSflpRotationTag) {  // quaternion [x,y,z,w]
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
    } else {
      nUnknownDbg_++;
      lastUnknownTag_ = tag;
    }
  }

  lastDrainMs_ = millis();

  if (kImuUseInt1) {
    // Robustness against a lost rising edge. The watermark interrupt is LEVEL
    // driven: if this drain ended with the FIFO still above the watermark - after a
    // blocking SD flush, say - the line never falls, no new RISING edge ever
    // arrives, fifoFlag_ is never set again and the stream dies silently. Re-arm
    // ourselves whenever a backlog remains. ORB_test found this the hard way; it is
    // not defensive padding.
    //
    // readFifoStatus rather than FIFO_Get_Num_Samples: same single transaction, but it
    // is also the only read positioned to catch FIFO_OVR_LATCHED from the pop loop
    // above. A card stall mid-loop can fill the FIFO and overwrite words, and the rest
    // of the loop then drains the level back under the brim - so the next drain's
    // FIFO_OVR_IA reads zero and the loss would leave no trace anywhere else.
    const FifoStatus after = readFifoStatus();
    if (after.ovrLatched) pendingOvrLatched_ = true;
    if (after.level >= kFifoWatermark) fifoFlag_ = true;
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
  dbg.print("  |  words/s "); dbg.print(elapsedS > 0 ? nWordsDbg_ / elapsedS : 0.0, 0);
  dbg.print(", peak ");       dbg.print(maxLevelDbg_);
  dbg.print("/512");
  if (nUnknownDbg_ > 0) {
    dbg.print(", unk "); dbg.print(nUnknownDbg_);
    dbg.print(" (tag 0x"); dbg.print(lastUnknownTag_, HEX); dbg.print(")");
  }
  if (nFullDbg_ > 0) { dbg.print(", full x"); dbg.print(nFullDbg_); }
  if (nOverflow_ > 0) {
    dbg.print("  [WARN] FIFO overrun x"); dbg.print(nOverflow_);
    nOverflow_ = 0;
  }
  dbg.println();
  nWordsDbg_ = nUnknownDbg_ = nFullDbg_ = 0;
  maxLevelDbg_ = 0;

  nAccDbg_ = nGyrDbg_ = 0;
  sumAccMag2_ = sumGyrMag2_ = 0.0;
  dbgLastPrint_ = now;
}
