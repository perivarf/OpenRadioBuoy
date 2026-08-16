#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include <Arduino.h>
#include <LSM6DSV16XSensor.h>

/*
  Sensor tuning: rates, ranges, filters, and the FIFO timing budget everything else in
  the capture loop is measured against.

  THE BUDGET. Accel, gyro and the on-chip fusion are all batched into the FIFO at their
  own rates, so it takes kFifoWordsPerSec words a second - 1200 at 480 Hz. Divided into
  kFifoDepthWords that gives kFifoFillMs, the time a drain has before the oldest word is
  overwritten: 213 ms at 480 Hz, 118 at 960. Every constant here is spent against it,
  and the numbers to hold against it are tim_flush_us_max and tim_welch_us_max in
  ses.csv - an sd-stall has been measured at 516 ms, the Welch FFT at 88.

  Draining is not the problem. A word is 7 bytes over the bus plus an address byte, so
  a full FIFO is 256 * 64 bit / 6 MHz = 2.7 ms. The AHRS and the sd-card are what fill
  the budget.
*/

// Upper edge of the analysed band. An analysis quantity, but it lives here because the
// LPF2 divisor below is derived from it and has to come after it.
static constexpr float kWaveFMax = 1.0f;

// -----------------------------------------------------------------------------
// Data rate and power mode, accel AND gyro
// -----------------------------------------------------------------------------
static constexpr uint16_t kImuOdrHz    = 480;   // 120/240/480/960 Hz
static constexpr uint8_t  kImuLowPower = 0;     // 1 = low power (ODR<=240), 0 = high performance
static_assert(kImuOdrHz == 120 || kImuOdrHz == 240 || kImuOdrHz == 480 || kImuOdrHz == 960,
              "kImuOdrHz must be 120, 240, 480 or 960 Hz");
static_assert(!(kImuLowPower && kImuOdrHz > 240),
              "Low-power is only valid for ODR <= 240 Hz");

static constexpr LSM6DSV16X_ACC_Operating_Mode_t kImuAccMode =
    kImuLowPower ? LSM6DSV16X_ACC_LOW_POWER_MODE1 : LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE;
static constexpr LSM6DSV16X_GYRO_Operating_Mode_t kImuGyrMode =
    kImuLowPower ? LSM6DSV16X_GYRO_LOW_POWER_MODE : LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE;

static constexpr uint16_t kAccelOdrHz = kImuOdrHz;

// SPI clock request. The LSM6DSV16X is rated max 10 MHz - exceeding it gives corrupt
// FIFO reads and NaN quaternions. The bus does not actually run at this rate: spi_init
// picks the fastest prescaler that does not exceed the request, so this lands on
// 48 MHz / 8 = 6 MHz. /4 would be 12 MHz and over the rating, so 6 is the ceiling and
// raising this buys nothing.
static constexpr uint32_t kImuSpiHz = 8000000;

// -----------------------------------------------------------------------------
// Accel LPF2. The cutoff is a FRACTION OF ODR (CTRL8.hp_lpf2_xl_bw), so pinning one
// enum makes it move with kImuOdrHz - STRONG is 9.6 Hz at 960 Hz but 1.2 Hz at 120,
// on top of the analysed band. The divisor is therefore derived, not chosen.
// -----------------------------------------------------------------------------
static constexpr bool kUseLpf2 = true;

// How far above kWaveFMax the cutoff must sit. 4x keeps the in-band droop small while
// landing near the 5 Hz Nyquist of the 10 Hz series this is decimated to, so LPF2 does
// the anti-alias work up front rather than instead of the FIR stages.
static constexpr float kLpf2Margin = 4.0f;
static constexpr float kLpf2MinHz  = kLpf2Margin * kWaveFMax;   // 4.0 Hz @ kWaveFMax 1.0

// Strongest LPF2 (largest divisor) whose cutoff still clears kLpf2MinHz. The hardware
// offers only these eight divisors, so this picks from the list rather than computing a
// number. At kWaveFMax 1.0 that is 200/100/45/20 for ODR 960/480/240/120.
static constexpr uint16_t lpf2DivForOdr(uint16_t odr) {
  return (float)odr >= 800.0f * kLpf2MinHz ? 800
       : (float)odr >= 400.0f * kLpf2MinHz ? 400
       : (float)odr >= 200.0f * kLpf2MinHz ? 200
       : (float)odr >= 100.0f * kLpf2MinHz ? 100
       : (float)odr >=  45.0f * kLpf2MinHz ?  45
       : (float)odr >=  20.0f * kLpf2MinHz ?  20
       : (float)odr >=  10.0f * kLpf2MinHz ?  10
       :                                        4;
}
static constexpr uint16_t kLpf2Div = lpf2DivForOdr(kImuOdrHz);

// Divisor -> CTRL8.hp_lpf2_xl_bw register value
static constexpr uint8_t lpf2BwForDiv(uint16_t div) {
  return div ==   4 ? LSM6DSV16X_XL_ULTRA_LIGHT   // ODR/4
       : div ==  10 ? LSM6DSV16X_XL_VERY_LIGHT    // ODR/10
       : div ==  20 ? LSM6DSV16X_XL_LIGHT         // ODR/20
       : div ==  45 ? LSM6DSV16X_XL_MEDIUM        // ODR/45
       : div == 100 ? LSM6DSV16X_XL_STRONG        // ODR/100
       : div == 200 ? LSM6DSV16X_XL_VERY_STRONG   // ODR/200
       : div == 400 ? LSM6DSV16X_XL_AGGRESSIVE    // ODR/400
       :              LSM6DSV16X_XL_XTREME;       // ODR/800
}
static constexpr uint8_t kLpf2Bw       = lpf2BwForDiv(kLpf2Div);
static constexpr float   kLpf2CutoffHz = (float)kImuOdrHz / kLpf2Div;  // 4.8 Hz @ 960 Hz

static_assert(kLpf2CutoffHz >= kLpf2MinHz,
              "no LPF2 divisor clears kWaveFMax by kLpf2Margin - raise kImuOdrHz, "
              "lower kWaveFMax, or accept a smaller margin");

// -----------------------------------------------------------------------------
// Full-scale range. The enum value IS the number passed to Set_X_FS/Set_G_FS (g / dps),
// and the raw-FIFO sensitivity is derived from it, so range and scaling cannot drift
// apart. A larger range captures bigger motion at coarser resolution.
// -----------------------------------------------------------------------------
enum class AccelFS : uint8_t  { G2 = 2, G4 = 4, G8 = 8, G16 = 16 };
enum class GyroFS  : uint16_t { DPS125 = 125, DPS250 = 250, DPS500 = 500,
                                DPS1000 = 1000, DPS2000 = 2000, DPS4000 = 4000 };

static constexpr AccelFS kAccelFS = AccelFS::G2;      // +-2 g
static constexpr GyroFS  kGyroFS  = GyroFS::DPS1000;  // +-1000 dps

// LSM6DSV16X sensitivities (datasheet), selected from the ranges above.
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
static constexpr float kAccSensMgPerLsb   = accSensMgPerLsb(kAccelFS);
static constexpr float kGyrSensMdpsPerLsb = gyrSensMdpsPerLsb(kGyroFS);

// -----------------------------------------------------------------------------
// On-chip sensor fusion (SFLP): a 6-axis AHRS in the sensor, emitting a quaternion
// that rotates the sensor frame into the gravity frame, batched into the FIFO at its
// own rate. Declared here and not with the analysis settings because kFifoWordsPerSec
// below needs the rate, and the whole timing budget needs kFifoWordsPerSec.
// -----------------------------------------------------------------------------
static constexpr bool    kEnableSflp      = true;
static constexpr float   kSflpOdrHz       = 240.0f;
static constexpr uint8_t kSflpRotationTag = 0x13;   // FIFO tag: SFLP rotation vector

// Words the FIFO takes in a second: accel and gyro at kImuOdrHz, plus the rotation
// vector at the fusion's own rate when it is batched. The denominator behind every
// timing claim in this file.
static constexpr uint32_t kFifoWordsPerSec =
    2u * (uint32_t)kImuOdrHz + (kEnableSflp ? (uint32_t)kSflpOdrHz : 0u);

// -----------------------------------------------------------------------------
// FIFO depth, watermark and the drain triggers
// -----------------------------------------------------------------------------

// FIFO depth in words. 1.5 kB of data / 6 payload bytes = 256 levels (DS13476 rev 5,
// section 6.10) - NOT the 512 the 9-bit DIFF_FIFO field suggests; those 9 bits are for
// the COMPRESSED modes, which hold 3x as many samples and are not in use here. Each
// level carries 6 data bytes; the tag byte is read out on top, hence 7 bytes per word
// over the bus.
//
// Measured, not only read: the level saturates at exactly 256 across three independent
// captures, hit 12-18 times each, never above.
static constexpr uint16_t kFifoDepthWords = 256;

/*
  Watermark in FIFO words: the level at which the sensor raises INT1, and therefore the
  STANDING level in the FIFO - which makes it the measure of how little free space is
  left when anything blocks. At 480 Hz and 1200 words/s:

    WTM 128 -> 128 free levels = 107 ms of budget
    WTM  64 -> 192 free levels = 160 ms

  The cost of a lower watermark is a shorter drain interval (53 ms at 64) and twice as
  many sync records in the raw log: 17 bytes per drain, 1.9 % -> 3.8 % of the file.

  FIFO_Set_Watermark_Level takes a uint8_t (FIFO_CTRL1.WTM is 8 bits), so this must stay
  under 256 - which is the whole buffer, not half of it.
*/
static constexpr uint16_t kFifoWatermark = 128;
static_assert(kFifoWatermark > 0 && kFifoWatermark < 256,
              "FIFO_CTRL1.WTM is 8 bits - a larger watermark would be truncated");

// The upper bound on the standing level, as a plain fraction of the depth. A CHOICE
// about how much of the buffer is held in reserve, not a measurement: what is left -
// kFifoDepthWords minus kFifoWatermark - is all the drain has to work with.
//
// 80 % caps it at 204. At today's 128 that leaves 128 words, i.e. 107 ms at 1200
// words/s, against a Welch FFT of 88 ms and an sd-stall measured at 516. The compiler
// can see neither of those two numbers - they exist only as tim_welch_us_max and
// tim_flush_us_max in ses.csv, which is where this limit is really checked.
static constexpr uint16_t kFifoWatermarkMaxPct = 80;
static_assert(100u * (uint32_t)kFifoWatermark
                  < kFifoWatermarkMaxPct * (uint32_t)kFifoDepthWords,
              "the watermark is the STANDING level - leaving under 20% of the FIFO free "
              "gives the drain no room for an sd-stall or the Welch FFT to land in");

/*
  Which of the two drain triggers the capture loop uses. They are a CHOICE, not a pair:

    true  (WTM)   the INT1 edge fires the drain, and nothing else does.
    false (DRAIN) elapsed time fires it, every kDrainIntervalMs.

  Either way a drain empties the whole buffer. INT1 is not here for response time - the
  capture loop calls update() every iteration regardless, and the ISR only sets a flag,
  so the drain still runs behind the same sd write and the same Welch FFT. What INT1
  does is HOLD BACK the drain until kFifoWatermark words have collected. It is a
  batching mechanism, and the watermark is the batch size. That is also why it is not an
  argument for raising the watermark.

  The cost of WTM is a LOST EDGE. The interrupt is level driven but wired to RISING: if
  a drain ends with the FIFO still above the watermark the line never falls, no new edge
  arrives, and the stream dies silently - measured as 0 Hz for the rest of the session.
  The re-arm in ImuSampler::update() is the only recovery, and it only covers the case
  where the level IS above the watermark (the aftermath of an sd-stall). An edge lost
  below the watermark ends the capture until the next reset.

  What WTM buys for that risk is the status read on every non-draining call - 0.55 % of
  CPU - and a cleaner overrun latch, since FIFO_OVR_LATCHED is then read only on drains.
*/
static constexpr bool kImuUseInt1 = true;

// INT1_CTRL (0x0D): which events the sensor drives out on the INT1 pin. Raw register
// values because the Arduino wrapper exposes no setter, only Write_Reg.
static constexpr uint8_t kInt1CtrlReg = 0x0D;
static constexpr uint8_t kInt1FifoTh  = 0x08;  // bit 3: FIFO watermark reached

// FIFO_STATUS1/FIFO_STATUS2. Read as a 2-byte burst rather than through the wrapper:
// lsm6dsv16x_fifo_status_get drops FIFO_OVR_LATCHED, and that bit is reset by the very
// read that drops it, so going through the wrapper makes it permanently unobservable.
static constexpr uint8_t kFifoStatus1Reg = 0x1B;

// FIFO_DATA_OUT_TAG. The six payload registers follow it (0x79..0x7E), and reading
// 0x7E is what pops the word - so a 7-byte auto-incrementing burst from here takes tag
// and payload out together, which is what makes the pairing atomic.
static constexpr uint8_t kFifoDataOutTagReg = 0x78;

/*
  How many FIFO words come out per SPI transaction.

  One word per transaction measured 28 us, of which only ~11 is bus time (8 bytes at
  6 MHz). The other ~17 is per-transaction overhead: beginTransaction, two CS writes,
  the address byte entering spi_transfer on its own, endTransaction. A burst pays that
  once instead of once per word - at 32 words, 0.5 us per word. Measured 19 us/word
  after the change, against a floor of 9.3 (pure bus time); the remainder is CPU inside
  STM32duino's block transfer loop, which is what DMA would remove.

  THE ASSUMPTION: that the address rolls from 0x7E back to 0x78, so one continuous read
  yields CONSECUTIVE words rather than the same word repeated. Verified on hardware, not
  only against the datasheet: a temporary self-test counted words with a tag outside
  {1, 2, 0x13} after position 0 in a burst (which would catch the address stopping at
  0x7E) and bursts whose words were all byte-identical to the first (which would catch a
  rollover without the FIFO advancing). Both counted 0 over a full capture.

  kFifoBurstWords = 1 restores one word per transaction with no other change, and is the
  right first test if FIFO data is ever suspected.
*/
static constexpr uint16_t kFifoBurstWords = 32;
static_assert(kFifoBurstWords > 0 && kFifoBurstWords <= kFifoDepthWords,
              "a burst cannot be empty, nor larger than the FIFO it reads from");

// Time for the FIFO to go from empty to full. The whole timing budget in this project
// is measured against this number, and ses.csv writes it out as tim_fifo_budget_us.
static constexpr uint32_t kFifoFillMs =
    (uint32_t)kFifoDepthWords * 1000u / kFifoWordsPerSec;   // 213 ms @ 480 Hz, 118 @ 960

/*
  The longest the DRAIN trigger may wait between drains.

  Derived, not set: a fixed millisecond value cannot serve two ODRs. 80 ms left 480 Hz
  with 96 words of headroom after a missed trigger but 960 Hz with 19, and lowering it
  to 50 to save 960 put it under the watermark's own cadence at 480 - at which point it
  stops being a reserve and becomes the normal trigger, draining on time instead of on
  fill level and paying a sync record per drain however few words it got.

  It is a fraction of kFifoFillMs and not of the watermark, because it guards the
  BUFFER. A watermark is a choice about how often to drain; the depth is a hard limit on
  how long one can afford not to. 3/5 against a ceiling of 3/4.
*/
static constexpr uint32_t kMaxDrainIntervalMs = 3u * kFifoFillMs / 5u;  // 127 ms @ 480 Hz

// The CHOSEN deadline, as a share of the maximum above. The split is worth two lines:
// kMaxDrainIntervalMs is derived from the FIFO depth and is a property of the hardware,
// while this is a choice about how much of that margin to spend. Lowering the percentage
// tightens the deadline without touching the derivation, which is the part that has to
// be reasoned about afresh each time.
//
// kMaxDrainIntervalMs MUST NOT LEAVE THIS FILE: it is the ceiling, not the deadline.
// Code reads kDrainIntervalMs, or the percentage knob is bypassed.
static constexpr uint32_t kDrainIntervalPct = 100;
static constexpr uint32_t kDrainIntervalMs  = kDrainIntervalPct * kMaxDrainIntervalMs / 100u;

// Trivially true for any percentage under 100, which is the point: it guards the knob,
// not the derivation. Above 100 it is a deadline longer than the FIFO survives, and the
// build should stop rather than let the number look legal.
static_assert(kDrainIntervalMs <= kMaxDrainIntervalMs,
              "the chosen drain deadline exceeds what the FIFO depth allows - "
              "kDrainIntervalPct must not go above 100");

#endif  // IMU_CONFIG_H
