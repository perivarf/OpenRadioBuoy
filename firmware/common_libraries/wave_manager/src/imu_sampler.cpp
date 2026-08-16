#include "imu_sampler.h"
#include "config.h"
#include "rotation.h"
#include <math.h>
#include <string.h>   // memmove: flushRaw flytter halen fram etter en delvis skriving

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

  // vacc_ is the only channel the wave chain reads - StreamAnalyzer::ingest takes
  // r.vaccFir from the row and nothing else that comes from this bank - so it is
  // evaluated unconditionally, filtered and unfiltered alike.
  r.vaccFir = vacc_.eval();
  r.vacc    = vacc_.center();

  /*
    The other nine are imu.csv columns and nothing else. Each of them has exactly one
    reader: the print run in WaveManager::onRow, behind its early return on a closed
    file. In WaveLogMode::Raw that file is never opened, and evaluating them anyway was
    9/10 of TIM_FIR - 210 of 234 ms/s, the largest single per-second cost in the capture
    loop - spent on nobody.

    The gate is the COMPILE-TIME mode, not the runtime imuCsvActive_. That flag is also
    false when the file merely failed to open, and a Csv capture must not quietly lose
    nine columns because the sd-card had a bad day; it has to fail the way it does today.

    Only the convolution is skipped. push() still runs for all ten (ImuSampler::update),
    so every delay line stays warm and this remains a change to eval() alone - see
    fir.h on why the two are separate and which of them carries the cost.
  */
  if constexpr (wave_mode_imu_csv()) {
    // Filtered
    r.ax = ax_.eval(); r.ay = ay_.eval(); r.az = az_.eval();
    r.axnSflp = nx_.eval(); r.aynSflp = ny_.eval(); r.aznSflp = nz_.eval();
    r.gx = gx_.eval(); r.gy = gy_.eval(); r.gz = gz_.eval();
    r.vaccSflpFir = r.aznSflp * kMg2Ms2;

    // Unfiltered
    r.vaccSflp = nz_.center() * kMg2Ms2;
  } else {
    // Zeroed rather than left alone: pendingRow_ is reused across rows, so a field
    // nothing writes would carry an indeterminate value under a real column name. No
    // file can ever receive these - their only writer is gated on the same constant -
    // but the row leaves here fully defined either way.
    r.ax = r.ay = r.az = 0.0f;
    r.axnSflp = r.aynSflp = r.aznSflp = 0.0f;
    r.gx = r.gy = r.gz = 0.0f;
    r.vaccSflpFir = r.vaccSflp = 0.0f;
  }
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
  nUnknownDbg_ = 0;
  lastUnknownTag_ = 0;
}

// Evaluate the decimation filters and read the delay-matched quaternions into the
// row being assembled. Everything here should referr to the same point in time instant:
// the centre of the current window
void ImuSampler::latchRowValues() {
  // Timed here rather than at the two call sites: the normal one (a sample reaching the
  // window centre) and the late one (closeWindow, after a FIFO gap) do the same work,
  // and both run inside the pop loop. How much work that is depends on the log mode -
  // kFirNtap taps across ten channels, or across one when imu.csv is not written. See
  // FirRowBank::eval.
  WAVE_TIME(TIM_FIR);
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
//
// The payload goes out as ONE block transfer, not a byte-at-a-time loop. Both forms
// clock the same bits down the same CS-low transaction - the difference is how often
// the core is entered: SPI.transfer(uint8_t) runs the whole of spi_transfer() per byte,
// so a 7-byte FIFO word paid that entry eight times. Measured 2026-08-15 at 37 us per
// word, of which only ~11 is the bus (see below); the rest was the eight entries.
//
// NB: the bus does NOT run at kImuSpiHz. spi_init picks the fastest prescaler that does
// not exceed the request, so 8 MHz against a 48 MHz PCLK2 lands on /8 = 6 MHz. /4 would
// be 12 MHz, past the sensor's 10 MHz rating, so 6 MHz is the ceiling here and raising
// kImuSpiHz buys nothing.
//
// tx_buf = nullptr clocks out 0xFF rather than the 0x00 the old loop sent. MOSI is
// don't-care for the duration of a read, so the sensor cannot tell the two apart.
void ImuSampler::imuBurstRead(uint8_t startReg, uint8_t *buf, uint16_t len) {
  SPI.beginTransaction(SPISettings(kImuSpiHz, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI_CS_IMU_PIN, LOW);
  SPI.transfer(startReg | 0x80);    // 0x80 = READ bit
  SPI.transfer(nullptr, buf, len);  // len dummy bytes out, the answer straight into buf
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
// Pull up to kFifoBurstWords words in ONE transfer. Reading 0x7E is what advances the
// FIFO, so a continuous read past it rolls the address back to the tag register and the
// next word follows in the same CS-low transaction - that is the property this rests on,
// and the one to check in the datasheet if the tags ever come out wrong.
//
// n is what the caller still has left to pop, and it is a CEILING and not a request:
// reading past the level the status word reported would return words the FIFO does not
// hold. kFifoBurstWords = 1 makes this exactly the old one-word-per-transaction read.
void ImuSampler::fillFifoBurst(uint16_t n) {
  burstFill_ = n < kFifoBurstWords ? n : kFifoBurstWords;
  burstIdx_  = 0;
  imuBurstRead(kFifoDataOutTagReg, burstBuf_,
               (uint16_t)(burstFill_ * kRawWordBytes));
}

// Raw log. Little-endian, layout documented at wave_raw_log in wave_config.h.
//
// APPEND ONLY - ingen skriving herfra. Bufferet rommer resten under skrivegrensa
// pluss en hel drenering (kRawBufBytes), og flushRaw() kalles av update() ETTER at
// pop-løkka har tømt FIFO-en. Det er hele poenget: et sd-kort som stanser 800 ms skal
// treffe en tom FIFO med 256 ledige nivåer, ikke en halvtømt med ~128.
//
// Kallet er uendret etter at kRawFlushThreshold kom inn 2026-08-15 - det er flushRaw()
// selv som avgjør om denne dreneringen faktisk skal røre kortet. Plasseringen rett
// etter en tømt FIFO er fortsatt den samme; stallen treffer bare sjeldnere.
// Se kRawBufBytes og skrivegrense-avsnittet i wave_config.h.
void ImuSampler::rawAppend(const uint8_t *p, uint8_t n) {
  if (!rawSink_) return;
  // Nødventil, ikke normal vei: bufferet er dimensjonert for resten under
  // skrivegrensa pluss en verste drenering, så dette kan bare skje om FIFO-en
  // leverer mer enn kFifoDepthWords - altså om den konstanten er feil igjen. Da er
  // en skriving midt i løkka bedre enn å skrive utenfor bufferet, og static_assert-en
  // over er det som skal fange det først. force, ellers ville et kall som bare
  // skriver hele sektorer kunne la det være igjen for lite plass til å hjelpe.
  if (rawLen_ + n > kRawBufBytes) flushRaw(true);
  for (uint8_t i = 0; i < n; i++) rawBuf_[rawLen_++] = p[i];
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

// Se skrivegrense-avsnittet i wave_config.h. Kort: normalveien skriver bare hele
// sektorer, og bare når kRawFlushThreshold har samlet seg, slik at SdFat kan skyve
// dem rett fra rawBuf_ til kortet uten å gå veien om sin ene 512-bytes cache.
uint16_t ImuSampler::flushRaw(bool force) {
  if (!rawSink_ || rawLen_ == 0) return 0;

  // Grensa hører hjemme HER og ikke på kallstedet: kRawBufBytes er dimensjonert ut
  // fra at det aldri ligger mer enn kRawFlushThreshold - 1 igjen når en drenering
  // starter, og den invarianten holder bare hvis hvert kall respekterer den.
  if (!force && rawLen_ < kRawFlushThreshold) return 0;

  // force tar halen med; ellers ligger den igjen til neste drenering fyller opp en
  // hel sektor rundt den. Restens plass er budsjettert i kRawBufBytes.
  const uint16_t n = force ? rawLen_
                           : (uint16_t)(rawLen_ / kRawBlockBytes) * kRawBlockBytes;
  if (n == 0) return 0;

  if (!rawSink_(rawBuf_, n)) {
    nRawWriteFail_++;
    rawWriteFailPending_ = true;
  }

  // Resten flyttes fram. Opptil 511 B et par ganger i sekundet; forsvinnende mot
  // skrivingen den nettopp ventet på.
  rawLen_ -= n;
  if (rawLen_ > 0) memmove(rawBuf_, rawBuf_ + n, rawLen_);
  return n;
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
  if (rowSink_) {
    // The sink is the analyzer plus - in a Csv build - the imu.csv row. Both sit inside
    // the pop loop, so this is one of the two ways a row can hold up the drain; the
    // other is the FIR eval in latchRowValues.
    const uint32_t tSink = timeStart();
    rowSink_(pendingRow_);
    timeAdd(TIM_ROWSINK, tSink);
  }
}

// Drain all pending FIFO words. Three tags are batched together: accel (2), gyro
// (1) and SFLP rotation / quaternion (0x13). The accel tag is the clock: it
// drives windowing, the AHRS, and all ten decimation filters, so every delay line
// advances exactly once per accel sample and they can never drift apart. Gyro and
// SFLP words only latch their latest value for the accel branch to pair with.
void ImuSampler::update(Print &dbg, uint32_t captureLeftMs) {
  // GATE 1, INT1 mode: the edge is the trigger, and since 2026-08-16 it is the ONLY
  // trigger - the deadline (kDrainIntervalMs, then still named kMaxDrainIntervalMs) was
  // taken out of this gate deliberately. Read that decision and its price in the INT1
  // section of wave_config.h before restoring it.
  //
  // What the deadline used to do here was break the state the 2026-08-04 measurement
  // found: one lost edge, and the capture sat at 0 Hz forever with no warning. What
  // remains against that is the re-arm at the end of this function, and it is now load
  // bearing rather than belt-and-braces: it recovers the case where a drain ends with
  // the FIFO still above the watermark - the aftermath of an sd-stall, i.e. the one
  // that happens in practice - but nothing recovers an edge lost while the level is
  // BELOW the watermark. That capture is over until the next reset.
  if (kImuUseInt1 && !fifoFlag_) {
    // Still print, so a stalled interrupt shows up as frozen counters, not silence.
    debugPrintStatus(dbg, captureLeftMs);
    return;
  }

  // DRAIN, the other mode: elapsed time is the trigger, and since 2026-08-16 it is the
  // only one here. It used to be "level >= kFifoWatermark OR the deadline", i.e. the
  // watermark emulated in software with the deadline behind it. The two are now a
  // CHOICE and not a pair: WTM drains on the flag, DRAIN drains on the clock, and each
  // one empties the whole buffer when it fires.
  //
  // What the level test bought was batching. Without any gate at all the drain ran on
  // every loop iteration: once the GPS work fell from 17 ms to 4.4 ms per iteration it
  // became 169 drains a second at 6.9 words each, and every drain costs a kRawSyncBytes
  // record - 2.9 kB/s of sync into a raw log whose preAllocate never budgeted for it.
  // kDrainIntervalMs has to carry that batching alone now: at 127 ms it is ~8 drains a
  // second, and the batch is whatever accumulated rather than a fixed word count.
  if (!kImuUseInt1 && (millis() - lastDrainMs_) < kDrainIntervalMs) {
    debugPrintStatus(dbg, captureLeftMs);
    return;
  }
  fifoFlag_ = false;

  // Level and flags from one read, so the count and the flags describe the same
  // instant - see readFifoStatus for the register and why it is not the wrapper's.
  //
  // BELOW both gates since 2026-08-16. It used to sit above them because the level it
  // returned WAS the polling gate; now neither gate needs it, so it runs only on calls
  // that actually drain. That also retires a hazard rather than just saving a read:
  // this read RESETS FIFO_OVR_LATCHED, so while it ran on gated calls it consumed
  // overruns nobody would ever see, and the gate had to carry the bit forward by hand.
  // With no read between drains the latch simply survives until the drain that reports
  // it, and pendingOvrLatched_ is left with the one producer that still makes sense -
  // the re-arm read at the end of this function.
  const uint32_t tStatus = timeStart();
  const FifoStatus st = readFifoStatus();
  timeAdd(TIM_STATUS, tStatus);

  // Below both gates, so TIM_UPDATE counts drains and not the far more numerous calls
  // that only tested a flag or a clock - a mean over those would be meaningless.
  WAVE_TIME(TIM_UPDATE);

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

  // Sync record first, so the words that follow it are the ones it describes.
  rawEmitSync(nSamples, lost ? kRawFlagFifoOvf : 0);

  // TIM_POP spans the whole loop; TIM_SPI inside it isolates the bus from the work done
  // on the words, and the difference between them is what the maths costs.
  const uint32_t tPop = timeStart();
  burstFill_ = burstIdx_ = 0;   // nothing carries over between drains
  for (uint16_t i = 0; i < nSamples; i++) {
    uint8_t payload[6];
    // TIM_SPI still wraps the WHOLE per-word cost, refill included, so n stays the word
    // count and the mean stays directly comparable to the 28 us this replaced. What
    // changed is max: it is now a whole burst (~320 us at 32 words), not one word.
    const uint32_t tSpi = timeStart();
    if (burstIdx_ == burstFill_) fillFifoBurst((uint16_t)(nSamples - i));
    const uint8_t *w = burstBuf_ + burstIdx_ * kRawWordBytes;
    burstIdx_++;
    const uint8_t tag = (uint8_t)(w[0] >> 3);          // tag_sensor, top 5 bits
    for (uint8_t k = 0; k < 6; k++) payload[k] = w[k + 1];
    timeAdd(TIM_SPI, tSpi);
    // EVERY word, including tags this code does not decode: the raw log is a record
    // of what the sensor produced, not of what the wave chain happens to consume.
    rawEmitWord(tag, payload);
    if (tag == 2) {  // accel (mg)
      float a[3];
      payloadToAxes(payload, kAccSensMgPerLsb, a);
      nAccDbg_++;   // counted, not summed: the RATE is the loss signal, |a| was not
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
        const uint32_t tAhrs = timeStart();
        ahrs_.update(latestGx_ * kMdps2Rads, latestGy_ * kMdps2Rads, latestGz_ * kMdps2Rads,
                     axS, ayS, azS, (float)samplePeriodMs_ * 1.0e-3f);
        timeAdd(TIM_AHRS, tAhrs);
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
  timeAdd(TIM_POP, tPop);

  // HER, og bare her, skrives dreneringen til kortet. FIFO-en er nettopp tømt, så
  // dette er det ene punktet i runden der et sd-stall møter fullt overskrivnings-
  // budsjett. Lå skrivingen inne i løkka over - som den gjorde til 2026-08-14 - startet
  // stallen med FIFO-en halvfull og halve budsjettet allerede brukt.
  //
  // Rekkefølgen mot readFifoStatus() under er ikke tilfeldig: den lesingen skal skje
  // ETTER skrivingen, for det er under skrivingen en overflow nå oppstår, og
  // FIFO_OVR_LATCHED er det eneste sporet den etterlater seg.
  // TIM_FLUSH is the sd-card, seen from the one place that can threaten the FIFO. Its
  // max is the number to hold against the drain budget when a capture reports an
  // overrun; flushBytes turns it into us/kB so a slow card and a big write can be told
  // apart.
  //
  // Bare bokført når det FAKTISK ble skrevet: etter at kRawFlushThreshold kom inn
  // returnerer de fleste dreneringene uten å røre kortet, og å telle dem med ville
  // fylle bøtta med nuller og gjøre middelverdien meningsløs nettopp som mål på hvor
  // dyr en skriving er. n faller tilsvarende - ca. 8 i sekundet, ikke 18.
  const uint32_t tFlush = timeStart();
  const uint16_t wroteBytes = flushRaw();
  if (wroteBytes > 0) {
    timeAdd(TIM_FLUSH, tFlush);
    wave_timing.addFlushBytes(wroteBytes);
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
    const uint32_t tRearm = timeStart();
    const FifoStatus after = readFifoStatus();
    timeAdd(TIM_STATUS, tRearm);
    if (after.ovrLatched) pendingOvrLatched_ = true;
    if (after.level >= kFifoWatermark) fifoFlag_ = true;
  }

  // Calibrate the accel sample period from real elapsed time / total samples, so
  // the time axis tracks the wall clock and the last sample lands on real elapsed time.
  if (accelIdx_ > 0) {
    samplePeriodMs_ = (double)(millis() - sessionStartMs_) / (double)accelIdx_;
  }

  debugPrintStatus(dbg, captureLeftMs);
}

// Print the effective accel/gyro sample rate + mean magnitudes
void ImuSampler::debugPrintStatus(Print &dbg, uint32_t captureLeftMs) {
  if (!debug_serial || imu_debug_print_period == 0) return;
  uint32_t now = millis();
  if (now - dbgLastPrint_ < imu_debug_print_period) return;

  // The printout is itself work done inside the capture loop: ~250 characters at 115200
  // baud is over 20 ms of blocking writes, which is a tenth of the drain budget. Timed
  // like everything else, so it can be ruled in or out rather than assumed harmless.
  const uint32_t tDbg = timeStart();

  double elapsedS = (now - dbgLastPrint_) / 1000.0;
  double accHz = elapsedS > 0 ? nAccDbg_ / elapsedS : 0.0;
  double gyrHz = elapsedS > 0 ? nGyrDbg_ / elapsedS : 0.0;

  dbg.print("[IMU] ");              dbg.print(captureLeftMs / 1000UL);
  dbg.print(" s left  |  Accel: "); dbg.print(accHz, 0);
  dbg.print(" Hz, Gyro: ");         dbg.print(gyrHz, 0);

  if (nUnknownDbg_ > 0) {
    dbg.print("  [WARN] unk "); dbg.print(nUnknownDbg_);
    dbg.print(" (tag 0x"); dbg.print(lastUnknownTag_, HEX); dbg.print(")");
  }
  
  if (nOverflow_ > 0) {
    dbg.print("  [WARN] FIFO overrun x"); dbg.print(nOverflow_);
    nOverflow_ = 0;
  }
  
  dbg.println();
  nUnknownDbg_ = 0;
  nAccDbg_ = nGyrDbg_ = 0;
  dbgLastPrint_ = now;

  if (wave_timing_enabled) {
    /*
      Every bucket as n and mean/max in microseconds, four to a line. MEAN AND MAX BOTH,
      and the max is the one that matters here: a stall is by definition a tail event, and
      a mean over a 20 s window with ~380 drains in it would divide a 300 ms stall down to
      under a millisecond and hide exactly what we are looking for.

      Microseconds throughout rather than a scaled unit per bucket - a reader comparing
      two columns should not first have to check which one is in ms.
    */
    for (uint8_t i = 0; i < TIM_COUNT; i++) {
      const TimeStat &s = wave_timing.b[i];
      if (i % 4 == 0) dbg.print(i == 0 ? "[TIM] " : "\n      ");
      dbg.print(kTimingNames[i]); dbg.print(' ');
      dbg.print(s.n);             dbg.print("x ");
      dbg.print(s.meanUs());      dbg.print('/');
      dbg.print(s.maxUs);         dbg.print("us  ");
    }
    dbg.println();

    // The two per-unit numbers the buckets cannot show directly, plus the raw log's write
    // cost per byte - which is what separates a slow card from a big write.
    const TimeStat &spi = wave_timing.b[TIM_SPI];
    const TimeStat &ahrs = wave_timing.b[TIM_AHRS];
    const TimeStat &flush = wave_timing.b[TIM_FLUSH];
    dbg.print("      per word ");   dbg.print(spi.meanUs());
    dbg.print(" us  per ahrs ");    dbg.print(ahrs.meanUs());
    dbg.print(" us  raw ");         dbg.print(wave_timing.flushBytes / 1024.0, 1);
    dbg.print(" kB");
    if (wave_timing.flushBytes > 0) {
      dbg.print(" at ");
      dbg.print((float)flush.sumUs / (float)wave_timing.flushBytes, 2);
      dbg.print(" us/B");
    }
    dbg.println();

    // Reset the window, THEN charge this print to the window that just opened: measured
    // into the window it closes, the value would be wiped by the reset one line above it.
    // The dbgprint column therefore describes the PREVIOUS print - one interval late,
    // which is the only way a report can carry its own cost at all.
    wave_timing.resetInterval();
    timeAdd(TIM_DBGPRINT, tDbg);
  }
}
