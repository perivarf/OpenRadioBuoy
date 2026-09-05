#ifndef IMU_DEVICE_H
#define IMU_DEVICE_H

#include <Arduino.h>
#include <SPI.h>
#include <LSM6DSV16XSensor.h>

// PIF TODO

/*
  IMU Sensor specifics found here.
*/

// SPI clock request. The LSM6DSV16X is rated max 10 MHz - exceeding it may give corrupt reads
static constexpr uint32_t kImuSpiHz = 8000000;

// -----------------------------------------------------------------------------
// FIFO
// -----------------------------------------------------------------------------

// FIFO depth in words: 1.5 kB of data / 6 payload bytes = 256 levels.
static constexpr uint16_t kFifoDepthWords = 256;

// Size of one FIFO word (off the bus, ie. without address byte): 
// the tag byte plus its six payload bytes. 
static constexpr uint8_t kImuFifoWordBytes = 7;

/*
  SPI burst read. How many FIFO words come out per SPI transaction.
  Higher burst is better for efficiency, worse for memory.
*/
static constexpr uint16_t kFifoBurstWords = 32;
static_assert(kFifoBurstWords > 0 && kFifoBurstWords <= kFifoDepthWords,
              "a burst cannot be empty, nor larger than the FIFO it reads from");

// -----------------------------------------------------------------------------
// Register map
// -----------------------------------------------------------------------------

// INT1_CTRL (0x0D): which events the sensor drives out on the INT1 pin. 
// Found no wrapper setter for this, so must use Write_Reg.
static constexpr uint8_t kInt1CtrlReg = 0x0D;
static constexpr uint8_t kInt1FifoTh  = 0x08;  // bit 3: FIFO watermark reached

// FIFO_STATUS1/FIFO_STATUS2. Read as a 2-byte burst rather than through the wrapper:
// lsm6dsv16x_fifo_status_get drops FIFO_OVR_LATCHED
static constexpr uint8_t kFifoStatus1Reg = 0x1B;

// FIFO_DATA_OUT_TAG. The six payload registers follow it (0x79..0x7E), and reading
// 0x7E is what pops the word - so a 7-byte auto-incrementing burst from here takes tag
// and payload out together, which is what makes the pairing atomic.
static constexpr uint8_t kFifoDataOutTagReg = 0x78;

// Relevant FIFO tag_sensor values
static constexpr uint8_t kTagGyro          = 0x01;
static constexpr uint8_t kTagAccel         = 0x02;
static constexpr uint8_t kTagSflpRotation  = 0x13;   // SFLP rotation vector

// -----------------------------------------------------------------------------
// List of potential ranges for accelerometer and gyroscope. 
// The enum value is the number passed to Set_X_FS/Set_G_FS (g / dps),
// and the raw-FIFO sensitivity is derived from it
// A larger range captures bigger motion at coarser resolution. 
// The range used is selected in imu_config.h
// -----------------------------------------------------------------------------
enum class AccelFS : uint8_t  { G2 = 2, G4 = 4, G8 = 8, G16 = 16 };
enum class GyroFS  : uint16_t { DPS125 = 125, DPS250 = 250, DPS500 = 500,
                                DPS1000 = 1000, DPS2000 = 2000, DPS4000 = 4000 };

static constexpr float accSensMgPerLsb(AccelFS fs) {
  return fs == AccelFS::G2 ? 0.061f
       : fs == AccelFS::G4 ? 0.122f
       : fs == AccelFS::G8 ? 0.244f
       :                     0.488f;   // G16
}
static constexpr float gyrSensMdpsPerLsb(GyroFS fs) {
  return fs == GyroFS::DPS125  ? 4.375f
       : fs == GyroFS::DPS250  ? 8.75f
       : fs == GyroFS::DPS500  ? 17.5f
       : fs == GyroFS::DPS1000 ? 35.0f
       : fs == GyroFS::DPS2000 ? 70.0f
       :                         140.0f;  // DPS4000
}

// -----------------------------------------------------------------------------
// Accel LPF2 - Return enum value for the strongest LPF2 (largest divisor) whose cutoff still clears bandwidth.
// From datasheet
// -----------------------------------------------------------------------------

// Strongest LPF2 (largest divisor) whose cutoff still clears bandwidth.
static constexpr uint16_t lpf2DivForOdr(uint16_t odr, uint16_t bandwidth, uint16_t acc_power_mode) {

  if (acc_power_mode == LSM6DSV16X_ACC_LOW_POWER_MODE1)
    return LSM6DSV16X_XL_ULTRA_LIGHT;

  const uint32_t ratio = (uint32_t)odr / bandwidth;

  const uint16_t div = ratio >= 800 ? 800
                      : ratio >= 400 ? 400
                      : ratio >= 200 ? 200
                      : ratio >= 100 ? 100
                      : ratio >=  45 ?  45
                      : ratio >=  20 ?  20
                      : ratio >=  10 ?  10
                      :                  4;

  return div;
}

static constexpr uint16_t lpf2DivEnum(uint16_t div) {

  return div ==   4 ? LSM6DSV16X_XL_ULTRA_LIGHT   // ODR/4
       : div ==  10 ? LSM6DSV16X_XL_VERY_LIGHT    // ODR/10
       : div ==  20 ? LSM6DSV16X_XL_LIGHT         // ODR/20
       : div ==  45 ? LSM6DSV16X_XL_MEDIUM        // ODR/45
       : div == 100 ? LSM6DSV16X_XL_STRONG        // ODR/100
       : div == 200 ? LSM6DSV16X_XL_VERY_STRONG   // ODR/200
       : div == 400 ? LSM6DSV16X_XL_AGGRESSIVE    // ODR/400
       :              LSM6DSV16X_XL_XTREME;       // ODR/800
}


// -----------------------------------------------------------------------------
// Power mode. Config asks for low-power or not
// -----------------------------------------------------------------------------
static constexpr LSM6DSV16X_ACC_Operating_Mode_t accModeFor(bool lowPower) {
  return lowPower ? LSM6DSV16X_ACC_LOW_POWER_MODE1 : LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE;
}
static constexpr LSM6DSV16X_GYRO_Operating_Mode_t gyrModeFor(bool lowPower) {
  return lowPower ? LSM6DSV16X_GYRO_LOW_POWER_MODE : LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE;
}

// IMU fifo tags internal meaning.
enum class ImuSampleKind : uint8_t { Accel, Gyro, Quat, Unknown };

struct ImuFifoWord {
  ImuSampleKind kind;
  uint8_t tag;          // tag_sensor (top 5 bits of the tag byte) - what the raw log stores
  uint8_t payload[6];   // verbatim off the bus, for the raw log
  float   v[4];         // Decoded value, acc: mg, gyro: mdps, quat: [x, y, z, w]
};

// One decoded read of FIFO_STATUS1/2.
struct ImuFifoStatus {
  uint16_t level;      // 9-bit DIFF_FIFO: words waiting
  bool     ovr;        // words already overwritten - the DATA LOST flag
  bool     full;       // at the brim, nothing lost yet
  bool     ovrLatched; // overran at some point since the previous read
};

class Lsm6dsvDevice {
 public:
  Lsm6dsvDevice();

  // Called at boot and at each capture
  bool begin(Print &dbg);

  // Check if IMU responds
  bool checkAlive(Print &dbg);

  // FIFO stream mode. Drop any pending watermark flag and start filling the FIFO
  // Stream mode overwrites the oldest word and keeps running, however overflow is reported
  void startStreaming();

  // Flush the hardware FIFO (BYPASS) and clear the pending flag
  void bypassFifo();

  // Shutdown sensor (between readings)
  void shutdown();

  /*
    Number of words in FIFO buffer and status flags (FIFO_STATUS1/2, 0x1B/0x1C)
    
    Datasheet DS13476 table 78:
      FIFO_WTM_IA      bit 7  filling >= WTM
      FIFO_OVR_IA      bit 6  FIFO overrun
      FIFO_FULL_IA     bit 5  FIFO will be full at the next ODR
      FIFO_OVR_LATCHED bit 3  latched overrun, is reset when this register is read
      DIFF_FIFO_8      bit 0  high bit of the 9-bit level in FIFO_STATUS1
  */
  ImuFifoStatus status();

  // Reset burst buffer
  void resetBurst();

  // One FIFO word, decoded
  void popWord(ImuFifoWord &w, uint16_t remaining);

  // The INT1 watermark flag, set by the ISR and by the post-drain re-arm.
  bool fifoReady() const { return fifoFlag_; }
  void clearFifoReady() { fifoFlag_ = false; }
  void setFifoReady() { fifoFlag_ = true; }

 private:

  // Configure IMU (register settings)
  void configure();

  // One block transfer for len auto-incrementing registers
  void burstRead(uint8_t startReg, uint8_t *buf, uint16_t len);

  // Pull up to kFifoBurstWords words into burstBuf_ in one transfer, capped at n.
  void fillBurst(uint16_t n);

  // attachInterrupt takes a plain function, so the ISR is a static trampoline that
  // reaches the instance through s_self
  static Lsm6dsvDevice *s_self;
  static void isrTrampoline();
  volatile bool fifoFlag_ = false;

  LSM6DSV16XSensor imu_;

  // Words held by fillBurst and how far the pop loop has got through them
  uint8_t  burstBuf_[kFifoBurstWords * kImuFifoWordBytes];
  uint16_t burstFill_ = 0; // Number of words in burstBuf_
  uint16_t burstIdx_  = 0; // Next word to pop
};

// Concrete ImuDevice for this build
using ImuDevice = Lsm6dsvDevice;

#endif  // IMU_DEVICE_H
