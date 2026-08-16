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
  overwritten: 213 ms at 480 Hz, 118 at 960. 

  Draining is not the problem. A word is 7 bytes over the bus plus an address byte, so
  a full FIFO is 256 * 64 bit / 6 MHz = ~2.7 ms. 
  
  The choice of AHRS and writes to SD-card are what fill the budget.
*/

// Upper edge of the analysed band. An analysis quantity, but it lives here because the
// LPF2 divisor below is derived from it and has to come after it.
static constexpr float kWaveFMax = 1.0f;

// -----------------------------------------------------------------------------
// Data rate and power mode, accel AND gyro
// -----------------------------------------------------------------------------
static constexpr uint16_t kImuOdrHz    = 480;   // 120/240/480/960 Hz. Same for Acc and Gyro
static constexpr uint8_t  kImuLowPower = 0;     // 1 = low power (ODR<=240), 0 = high performance
static_assert(kImuOdrHz == 120 || kImuOdrHz == 240 || kImuOdrHz == 480 || kImuOdrHz == 960,
              "kImuOdrHz must be 120, 240, 480 or 960 Hz");
static_assert(!(kImuLowPower && kImuOdrHz > 240),
              "Low-power is only valid for ODR <= 240 Hz");

static constexpr LSM6DSV16X_ACC_Operating_Mode_t kImuAccMode =
    kImuLowPower ? LSM6DSV16X_ACC_LOW_POWER_MODE1 : LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE;
static constexpr LSM6DSV16X_GYRO_Operating_Mode_t kImuGyrMode =
    kImuLowPower ? LSM6DSV16X_GYRO_LOW_POWER_MODE : LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE;


// SPI clock request. The LSM6DSV16X is rated max 10 MHz - exceeding it may give corrupt
// reads. spi_init pick fastest supported rate on or below. 
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

// FIFO depth in words, must match sensor. 1.5 kB of data / 6 payload bytes = 256 levels
static constexpr uint16_t kFifoDepthWords = 256;

/*
  Watermark in FIFO words: the level at which the sensor raises INT1.
  At 480 Hz and 1200 words/s:

    WTM 128 -> 128 free levels = 107 ms of budget
    WTM  64 -> 192 free levels = 160 ms

  The cost of a lower watermark is more sync records in raw log,
  but most importantly it gives less opportunity for MCU sleep and potential energy saving.
*/

static constexpr uint16_t kFifoWatermark = 128; // Should not be set too close to kFifoWatermark, or the FIFO may overrun before the drain is triggered. 128 is a good compromise between latency and energy efficiency.
static_assert(kFifoWatermark > 0 && kFifoWatermark < 256,
              "FIFO_CTRL1.WTM is 8 bits - a larger watermark would be truncated");

/*
  Which of the two drain triggers the capture loop uses. They are a CHOICE, not a pair:

    true  (WTM)   the INT1 edge fires the drain, and nothing else does.
    false (DRAIN) elapsed time fires it, every kDrainIntervalMs.

  Either way a drain empties the whole buffer
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
  SPI burst read.How many FIFO words come out per SPI transaction.
  Higher burst is better for efficiency, worse for memory.
*/
static constexpr uint16_t kFifoBurstWords = 32;
static_assert(kFifoBurstWords > 0 && kFifoBurstWords <= kFifoDepthWords,
              "a burst cannot be empty, nor larger than the FIFO it reads from");

// Time for the FIFO to go from empty to full. The whole timing budget in this project
// is measured against this number, and ses.csv writes it out as tim_fifo_budget_us.
static constexpr uint32_t kFifoFillMs =
    (uint32_t)kFifoDepthWords * 1000u / kFifoWordsPerSec;   // 213 ms @ 480 Hz, 118 @ 960

/*
  The longest the drain trigger may wait between drains * 60%
*/
static constexpr uint32_t kMaxDrainIntervalMs = 3u * kFifoFillMs / 5u;  // 127 ms @ 480 Hz

// The chosen deadline, as a share of the maximum above
static constexpr uint32_t kDrainIntervalPct = 100;
static constexpr uint32_t kDrainIntervalMs  = kDrainIntervalPct * kMaxDrainIntervalMs / 100u;

// Asserting that kDrainIntervalMs <= kMaxDrainIntervalMs 
static_assert(kDrainIntervalMs <= kMaxDrainIntervalMs,
              "the chosen drain deadline exceeds what the FIFO depth allows - "
              "kDrainIntervalPct must not go above 100");

#endif  // IMU_CONFIG_H
