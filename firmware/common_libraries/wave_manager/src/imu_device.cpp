#include "imu_device.h"
#include "config.h"        // SPI_CS_IMU_PIN, INT1_IMU_PIN
#include "wave_config.h"   // the choices written into the registers below

#include <math.h>

Lsm6dsvDevice *Lsm6dsvDevice::s_self = nullptr;

// Interrupt context: set the flag and return.
void Lsm6dsvDevice::isrTrampoline() {
  if (s_self) s_self->fifoFlag_ = true;
}

Lsm6dsvDevice::Lsm6dsvDevice()
    : imu_(&SPI, (int)SPI_CS_IMU_PIN, kImuSpiHz) { s_self = this; }

// -----------------------------------------------------------------------------
// Bring-up and configuration
// -----------------------------------------------------------------------------
bool Lsm6dsvDevice::begin(Print &dbg) {
  // Keep CS high before any bus activity
  pinMode(SPI_CS_IMU_PIN, OUTPUT);
  digitalWrite(SPI_CS_IMU_PIN, HIGH);

  if (imu_.begin() != LSM6DSV16X_OK) {
    dbg.println("LSM6DSV: begin() failed - check SPI wiring/CS");
    return false;
  }
  dbg.println("LSM6DSV: begin() OK");

  configure();

  if (kImuUseInt1) {
    // pinMode before attachInterrupt, or the ISR never fires.
    pinMode(INT1_IMU_PIN, INPUT);
    // The sensor drives INT1 push-pull, active high -> trigger on the rising edge.
    attachInterrupt(digitalPinToInterrupt(INT1_IMU_PIN), isrTrampoline, RISING);
  }

  return true;
}

void Lsm6dsvDevice::configure() {
  imu_.Enable_X();      // accelerometer
  imu_.Enable_G();      // gyroscope

  // Ranges, ODR and power mode - the choices from imu_config.h, already translated into
  // this part's enums by the mapping helpers in imu_device.h
  imu_.Set_X_FS((int32_t)kAccelFS);
  imu_.Set_G_FS((int32_t)kGyroFS);
  imu_.Set_X_ODR((float)kImuOdrHz, kImuAccMode);
  imu_.Set_G_ODR((float)kImuOdrHz, kImuGyrMode);

  // Accelerometer low-pass filter. Arg 0 = low-pass mode, arg 2 = bandwidth.
  if (kUseLpf2) {
    imu_.Set_X_Filter_Mode(0, kLpf2Bw);
  }

  // Batch accel + gyro into the FIFO at the chosen ODR. Stream mode starts separately.
  imu_.FIFO_Set_X_BDR((float)kImuOdrHz);
  imu_.FIFO_Set_G_BDR((float)kImuOdrHz);

  // SFLP is the AHRS built into the LSM6DSV16X: a quaternion rotating the sensor frame
  // into the gravity frame, batched into the FIFO alongside accel and gyro. It can also
  // return an estimated gyro bias, and Set_SFLP_GBIAS(x, y, z) in rad/s can inject a
  // known one at start to speed up convergence.
  if (kEnableSflp) {
    imu_.Enable_Rotation_Vector();
    imu_.Set_SFLP_ODR(kSflpOdrHz);
    imu_.Set_SFLP_Batch(true, false, false);  // (rotation, gravity, gBias) -> FIFO
  } else {
    imu_.Disable_Rotation_Vector();
    imu_.Set_SFLP_Batch(false, false, false);
  }

  // Drive INT1 high once the FIFO holds kFifoWatermark words, so the drain runs on the
  // sensor's cadence rather than the main loop's. INT1_CTRL has no setter in the
  // wrapper, hence the raw register write.
  if (kImuUseInt1) {
    imu_.FIFO_Set_Watermark_Level((uint8_t)kFifoWatermark);
    imu_.Write_Reg(kInt1CtrlReg, kInt1FifoTh);
  }
}

bool Lsm6dsvDevice::checkAlive(Print &dbg) {
  uint8_t drdy = 0;
  const uint32_t deadline = millis() + 100;
  while (millis() < deadline) {
    if (imu_.Get_X_DRDY_Status(&drdy) == LSM6DSV16X_OK && drdy) break;
  }

  int32_t a[3] = {0, 0, 0};
  const bool read_ok = (imu_.Get_X_Axes(a) == LSM6DSV16X_OK);

  // Leave the sensor OFF until the next capture.
  shutdown();

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

void Lsm6dsvDevice::startStreaming() {
  fifoFlag_ = false;
  imu_.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
}

void Lsm6dsvDevice::bypassFifo() {
  imu_.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
  fifoFlag_ = false;
}

void Lsm6dsvDevice::shutdown() {
  imu_.Disable_G();
  imu_.Disable_X();
}

// -----------------------------------------------------------------------------
// SPI + FIFO
// -----------------------------------------------------------------------------
void Lsm6dsvDevice::burstRead(uint8_t startReg, uint8_t *buf, uint16_t len) {
  SPI.beginTransaction(SPISettings(kImuSpiHz, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI_CS_IMU_PIN, LOW);
  SPI.transfer(startReg | 0x80);    // 0x80 = READ bit
  SPI.transfer(nullptr, buf, len);  // len dummy bytes out, the answer straight into buf
  digitalWrite(SPI_CS_IMU_PIN, HIGH);
  SPI.endTransaction();
}

ImuFifoStatus Lsm6dsvDevice::status() {
  uint8_t sb[2] = {0, 0};
  burstRead(kFifoStatus1Reg, sb, 2);
  return ImuFifoStatus{
      (uint16_t)(((uint16_t)(sb[1] & 0x01u) << 8) | sb[0]),
      (sb[1] & 0x40u) != 0,
      (sb[1] & 0x20u) != 0,
      (sb[1] & 0x08u) != 0};
}

void Lsm6dsvDevice::resetBurst() {
  burstFill_ = burstIdx_ = 0;
}

void Lsm6dsvDevice::fillBurst(uint16_t n) {
  burstFill_ = n < kFifoBurstWords ? n : kFifoBurstWords;
  burstIdx_  = 0;
  burstRead(kFifoDataOutTagReg, burstBuf_, (uint16_t)(burstFill_ * kImuFifoWordBytes));
}

// -----------------------------------------------------------------------------
// Payload decoding
// -----------------------------------------------------------------------------

// Payload -> three int16 in LSB order. The driver truncates its own conversion to
// int32 (FIFO_Get_X_Axes returns whole mg), which throws away the sub-LSB range the
// sensitivity actually provides; decoding here keeps it in float.
static inline void payloadToAxes(const uint8_t p[6], float sens, float out[3]) {
  for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = (int16_t)((uint16_t)p[2 * i] | ((uint16_t)p[2 * i + 1] << 8));
    out[i] = (float)raw * sens;
  }
}

// IEEE half -> float, bit for bit as the driver's npy_halfbits_to_floatbits does. The
// SFLP words are three halves; the driver's own decoder is private, so it is mirrored
// here rather than called.
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

// SFLP rotation vector: x,y,z as halves, w reconstructed from the unit norm. Same
// reconstruction as the driver's sflp2q, including the renormalisation guard for a sum
// of squares that rounds above 1. Output is [x,y,z,w], matching the wrapper's
// FIFO_Get_Rotation_Vector.
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

/*
  Pop one word: the tag byte at 0x78 and its six payload bytes at 0x79..0x7E.

    w[0..6] = tag | x_lo x_hi | y_lo y_hi | z_lo z_hi    (three LE int16, tag >> 3 = sensor)

  FIFO_DATA_OUT_TAG is bit 0 unused, bits 2:1 tag_cnt, bits 7:3 tag_sensor - so the tag
  is shifted down by 3 here, and that shifted value is what the raw log stores.

  Decoding is unconditional: a word is exactly one kind, so the branch below does the
  same work the caller's would have. Only Unknown skips it, having nothing to decode.
*/
void Lsm6dsvDevice::popWord(ImuFifoWord &w, uint16_t remaining) {
  if (burstIdx_ == burstFill_) fillBurst(remaining);
  const uint8_t *raw = burstBuf_ + burstIdx_ * kImuFifoWordBytes;
  burstIdx_++;

  w.tag = (uint8_t)(raw[0] >> 3);
  for (uint8_t k = 0; k < 6; k++) w.payload[k] = raw[k + 1];

  if (w.tag == kTagAccel) {
    w.kind = ImuSampleKind::Accel;
    payloadToAxes(w.payload, kAccSensMgPerLsb, w.v);      // mg
  } else if (w.tag == kTagGyro) {
    w.kind = ImuSampleKind::Gyro;
    payloadToAxes(w.payload, kGyrSensMdpsPerLsb, w.v);    // mdps
  } else if (w.tag == kTagSflpRotation) {
    w.kind = ImuSampleKind::Quat;
    payloadToQuat(w.payload, w.v);                        // [x, y, z, w]
  } else {
    w.kind = ImuSampleKind::Unknown;   // datasheet table 210 lists the rest
  }
}
