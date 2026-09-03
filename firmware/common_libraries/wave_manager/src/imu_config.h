#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include <Arduino.h>
#include "imu_device.h"

/*
  Sensor settings: rates, ranges, filters, and the FIFO timing budget everything else in
  the capture loop is measured against.

  Device specifics are found in imu_device.h - depth, tags, registers, the tables

  Accel, gyro and the on-chip fusion are all batched into the FIFO at their
  own rates, so it takes kFifoWordsPerSec words a second - 1200 at 480 Hz.
  
  (1s with 480Hz: 480 accel, 480 gyro, 240 fusion = 1200)
  
  The buffer (kFifoDepthWords) is filled up in:
  kFifoDepthWords / kFifoWordsPerSec * 1000 = 256 / 1200 * 1000 = 213 ms.

  Draining is not the problem. A word is 7 bytes over the bus plus an address byte, so
  a full FIFO is 256 * 8B * 8bit/Byte / 6 Mbit/s (6MHz) = ~2.7 ms.

  The choice of AHRS and write or not to SD-card (and SD-card type) are what fill the budget.
*/

// Upper edge of the analysed band. An analysis quantity, but it lives here because the
// LPF2 divisor below is derived from it and has to come after it.
static constexpr float kWaveFMax = 2.0f;

// -----------------------------------------------------------------------------
// Data rate and power mode, accel AND gyro
// -----------------------------------------------------------------------------
static constexpr uint16_t kImuOdrHz    = 480;   // 120/240/480/960 Hz. Same for Acc and Gyro
static constexpr uint8_t  kImuLowPower = 0;     // 1 = low power (ODR<=240), 0 = high performance
static_assert(kImuOdrHz == 120 || kImuOdrHz == 240 || kImuOdrHz == 480 || kImuOdrHz == 960,
              "kImuOdrHz must be 120, 240, 480 or 960 Hz");
static_assert(!(kImuLowPower && kImuOdrHz > 240),
              "Low-power is only valid for ODR <= 240 Hz");

// Get low power mode or not
static constexpr auto kImuAccMode = accModeFor(kImuLowPower);
static constexpr auto kImuGyrMode = gyrModeFor(kImuLowPower);

// -----------------------------------------------------------------------------
// Deriving accelerator low pass filter 2 - LPF2. 
// The cutoff is a fraction of ODR
// STRONG is 9.6 Hz at 960 Hz but 1.2 Hz at 120
// -----------------------------------------------------------------------------
static constexpr bool kUseLpf2 = true;

// How far above kWaveFMax the cutoff must sit. 4x keeps the in-band droop small while
// landing near the 5 Hz Nyquist of the 10 Hz series this is decimated to, so LPF2 does
// the anti-alias work up front rather than instead of the FIR stages.
static constexpr float kLpf2Margin = 4.0f;
static constexpr float kLpf2MinHz  = kLpf2Margin * kWaveFMax;

// Strongest divisor that still clears kLpf2MinHz. At kWaveFMax 1.0 that is
// 200/100/45/20 for ODR 960/480/240/120.
static constexpr uint16_t kLpf2Div      = lpf2DivForOdr(kImuOdrHz, kLpf2MinHz);
static constexpr uint8_t  kLpf2Bw       = lpf2BwForDiv(kLpf2Div);
static constexpr float    kLpf2CutoffHz = (float)kImuOdrHz / kLpf2Div;

static_assert(kLpf2CutoffHz >= kLpf2MinHz,
              "no LPF2 divisor clears kWaveFMax by kLpf2Margin - raise kImuOdrHz, "
              "lower kWaveFMax, or accept a smaller margin");

// -----------------------------------------------------------------------------
// Accelerometer and gyro range. 
// The enum value is the number passed to Set_X_FS/Set_G_FS (g / dps),
// -----------------------------------------------------------------------------
static constexpr AccelFS kAccelFS = AccelFS::G4;      // +-4 g
static constexpr GyroFS  kGyroFS  = GyroFS::DPS500;  // +-500 dps

static constexpr float kAccSensMgPerLsb   = accSensMgPerLsb(kAccelFS);
static constexpr float kGyrSensMdpsPerLsb = gyrSensMdpsPerLsb(kGyroFS);

// -----------------------------------------------------------------------------
// On-chip sensor fusion (SFLP): a 6-axis AHRS in the sensor, emitting a quaternion
// that rotates the sensor frame into the gravity frame
// -----------------------------------------------------------------------------
static constexpr bool  kEnableSflp = true;
static constexpr float kSflpOdrHz  = 240.0f;

// Words the FIFO takes in a second: accel and gyro at kImuOdrHz, plus the rotation
// vector at the fusion's own rate when it is batched. 
static constexpr uint32_t kFifoWordsPerSec =
    2u * (uint32_t)kImuOdrHz + (kEnableSflp ? (uint32_t)kSflpOdrHz : 0u);

/*
  Watermark in FIFO words: the level at which the sensor raises INT1.
  At 480 Hz and 1200 words/s:

    FIFO fills up in 
    WTM 128 -> 128 free levels = 107 ms of budget (213 ms at 480 Hz)
    WTM  64 -> 192 free levels = 160 ms of budget (213 ms at 480 Hz)

  The cost of a lower watermark is more sync records in raw log,
  but most importantly it gives less opportunity for MCU sleep and potential energy saving.
*/

// Should not be set too close to kFifoDepthWords, or the FIFO may overrun before the drain is triggered. 
// 128 is a good compromise between latency and energy efficiency.
static constexpr uint16_t kFifoWatermark = 128; 
static_assert(kFifoWatermark > 0 && kFifoWatermark < 256,
              "FIFO_CTRL1.WTM is 8 bits - a larger watermark would be truncated");
/*
  Which of the two drain triggers the capture loop uses. They are a CHOICE, not a pair:

    true  (WTM)   the INT1 edge fires the drain, and nothing else does.
    false (DRAIN) elapsed time fires it, every kDrainIntervalMs.

  Either way a drain empties the whole buffer
*/
static constexpr bool kImuUseInt1 = true;

// Time for the FIFO to go from empty to full. The whole timing budget in this project
// is measured against this number
static constexpr uint32_t kFifoFillMs =
    (uint32_t)kFifoDepthWords * 1000u / kFifoWordsPerSec;   // 213 ms @ 480 Hz, 118 @ 960

/*
  The longest the drain trigger may wait between drains * 60%
  Fail safe in case interrupt is not received
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
