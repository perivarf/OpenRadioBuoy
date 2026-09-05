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

// What a FIFO word turned out to be. The sampler switches on this instead of on the
// sensor's tag numbers.
enum class ImuSampleKind : uint8_t { Accel, Gyro, Quat, Unknown };

struct ImuFifoWord {
  ImuSampleKind kind;
  uint8_t tag;          // tag_sensor (top 5 bits of the tag byte) - what the raw log stores
  uint8_t payload[6];   // verbatim off the bus, for the raw log
  // Decoded: accel mg, gyro mdps, or the rotation quaternion as [x, y, z, w].
  // Untouched when kind is Unknown.
  float   v[4];
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

  // ODR/FS/filter, FIFO + SFLP batching, INT1. Does NOT start the FIFO stream (see
  // startStreaming). Assumes the shared SPI bus is already begun.
  //
  // Called at boot and again before each capture, so the return value answers "is the
  // IMU alive now", not "was it at boot".
  bool begin(Print &dbg);

  // Boot liveness check: begin() only proves the part ANSWERS, so a dead or stuck
  // converter passes it. Waits for data-ready, reads one sample and shuts down again.
  // A direct register read, not a FIFO drain - at boot there is no capture open.
  bool checkAlive(Print &dbg);

  // FIFO -> STREAM/continuous, so it starts filling, and drop any pending watermark
  // flag. STREAM and FIFO mode share the buffer but differ when it fills: STREAM
  // overwrites the oldest word and keeps running, FIFO stops until a BYPASS trip
  // restarts it. STREAM tolerates a missed read; FIFO turns one into a hard gap.
  void startStreaming();

  // Flush the hardware FIFO (BYPASS) and clear the pending flag. Leaves it idle -
  // starting the stream again is startStreaming's job.
  void bypassFifo();

  // Park the sensor between captures; begin() is the other half. Only the ODR fields
  // move - everything else begin() wrote stays in its register, which is what makes
  // begin() cheap enough to be the way back up.
  void shutdown();

  /*
    Level AND every status flag from ONE 2-byte burst of FIFO_STATUS1/2 (0x1B, 0x1C),
    so the count and the flags describe the same instant.

    Datasheet DS13476 table 78 and section 6.10.3 (continuous mode), verbatim:
      FIFO_WTM_IA      bit 7  filling >= WTM
      FIFO_OVR_IA      bit 6  "FIFO is completely filled"; 6.10.3 adds that on an
                              overrun "at least one of the oldest samples in FIFO has
                              been overwritten" - this is the DATA LOST flag
      FIFO_FULL_IA     bit 5  "FIFO will be full at the next ODR" - the brim WARNING,
                              which by definition asserts one ODR BEFORE OVR does
      FIFO_OVR_LATCHED bit 3  latched overrun, "reset when this register is read"
      DIFF_FIFO_8      bit 0  high bit of the 9-bit level in FIFO_STATUS1

    A raw read and not the wrapper's: lsm6dsv16x_fifo_status_get reads this register,
    discards FIFO_OVR_LATCHED and clears it in the same breath, which would make that
    bit permanently unobservable. Every caller must come through here.
  */
  ImuFifoStatus status();

  // Nothing carries over between drains - call before a pop loop.
  void resetBurst();

  // One FIFO word, decoded. remaining is what the caller still has left to pop; it is
  // a CEILING on the next burst and not a request, since reading past the level the
  // status word reported would return words the FIFO does not hold.
  void popWord(ImuFifoWord &w, uint16_t remaining);

  // The INT1 watermark flag, set by the ISR and by the post-drain re-arm.
  bool fifoReady() const { return fifoFlag_; }
  void clearFifoReady() { fifoFlag_ = false; }
  void setFifoReady() { fifoFlag_ = true; }

 private:
  // Every register setting: what to ask for comes from imu_config.h, what to call it
  // and where to write it from the device facts at the top of this file.
  void configure();

  /*
    Auto-incrementing register burst on the shared SPI bus: one CS-low transfer for
    len consecutive registers (IF_INC = 1, the default). Settings must match what the
    driver uses for its own reads, or the two would disagree about the bus: MODE3, MSB
    first, 0x80 as the read bit.

    The payload goes out as ONE block transfer, not a byte-at-a-time loop. Both clock
    the same bits down the same transaction; the difference is how often the core is
    entered, since SPI.transfer(uint8_t) runs the whole of spi_transfer() per byte. A
    7-byte word paid that entry eight times and measured 37 us, of which only ~11 is
    the bus.

    len is uint16_t, not uint8_t: a FIFO burst reads kFifoBurstWords * 7 bytes - 224 at
    32 words, and past 36 words it would no longer fit a byte.

    tx_buf = nullptr clocks out 0xFF rather than 0x00. MOSI is don't-care for the
    duration of a read, so the sensor cannot tell the two apart.
  */
  void burstRead(uint8_t startReg, uint8_t *buf, uint16_t len);

  // Pull up to kFifoBurstWords words into burstBuf_ in ONE transfer, capped at n.
  // Reading 0x7E is what advances the FIFO, so a continuous read past it rolls the
  // address back to the tag register and the next word follows in the same CS-low
  // transaction. See kFifoBurstWords at the top of this file.
  void fillBurst(uint16_t n);

  // attachInterrupt takes a plain function, so the ISR is a static trampoline that
  // reaches the instance through s_self. It only sets the flag - reading the FIFO from
  // interrupt context would be a bad idea.
  static Lsm6dsvDevice *s_self;
  static void isrTrampoline();
  volatile bool fifoFlag_ = false;

  LSM6DSV16XSensor imu_;

  // Words held by fillBurst and how far the pop loop has got through them. Reset per
  // burst, not per drain: a drain longer than kFifoBurstWords refills.
  uint8_t  burstBuf_[kFifoBurstWords * kImuFifoWordBytes];
  uint16_t burstFill_ = 0;
  uint16_t burstIdx_  = 0;
};

// The swap point. Same shape as WaveAhrs in analysis_config.h: one concrete type chosen
// at compile time, no virtuals, so nothing indirect lands in the pop loop.
using ImuDevice = Lsm6dsvDevice;

#endif  // IMU_DEVICE_H
