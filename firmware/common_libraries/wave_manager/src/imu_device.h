#ifndef IMU_DEVICE_H
#define IMU_DEVICE_H

#include <Arduino.h>
#include <SPI.h>
#include <LSM6DSV16XSensor.h>
#include "wave_config.h"

/*
  The IMU, and the only place that knows which one it is.

  Everything sensor-specific lives behind this class: register addresses, the SPI
  transactions, the FIFO tag numbers, the sensitivities, the half-float decoding and
  the INT1 watermark plumbing. ImuSampler above it deals in ImuFifoWord and knows none
  of it, so a different sensor is a new class here plus the ImuDevice alias at the
  bottom - not an edit to the sampling pipeline.

  Shares the global Arduino SPI object (sd_writer brings up SPI1) rather than owning a
  bus. NB: the bus does NOT run at kImuSpiHz. spi_init picks the fastest prescaler that
  does not exceed the request, so 8 MHz against a 48 MHz PCLK2 lands on /8 = 6 MHz. /4
  would be 12 MHz, past the sensor's 10 MHz rating, so 6 MHz is the ceiling and raising
  kImuSpiHz buys nothing.
*/

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
  // Every register setting that comes from wave_config.h
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
  // transaction. See kFifoBurstWords in wave_config.h.
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
  uint8_t  burstBuf_[kFifoBurstWords * kRawWordBytes];
  uint16_t burstFill_ = 0;
  uint16_t burstIdx_  = 0;
};

// The swap point. Same shape as WaveAhrs in wave_config.h: one concrete type chosen at
// compile time, no virtuals, so nothing indirect lands in the pop loop.
using ImuDevice = Lsm6dsvDevice;

#endif  // IMU_DEVICE_H
