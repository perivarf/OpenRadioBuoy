#ifndef WAVE_CONFIG_H
#define WAVE_CONFIG_H

#include <Arduino.h>
#include <LSM6DSV16XSensor.h>

#include "config.h"
/*
  IMU + wave-analysis tuning, ported from ORB_test/src/settings.h.

  These constants live inside the wave_manager library (not in the shared
  common_config.h) because they - and the LSM6DSV16X driver types they reference -
  are only ever compiled for the drifter. The basestation, which also includes
  common_config.h, has no IMU and no LSM6DSV16X dependency.

  The shared wave settings the base station DOES need (enable flag + measurement
  period, read by LoRa_Transceiver) stay in common_config.h. The SPI/CS pins come
  from common_config.h too (SPI_MOSI/MISO/SCK_PIN, SPI_CS_IMU_PIN) so the SD card
  and IMU agree on the one shared SPI1 bus.
*/

// -----------------------------------------------------------------------------
// Orientation method used on-device to derive vertical acceleration. Only ONE is
// computed per capture to keep RAM within budget; the raw imu.csv is logged so the
// offline postprocess (ORB_test/tools/postprocess.py) can still compare all three.
// -----------------------------------------------------------------------------
enum class WaveOrientation : uint8_t { Madgwick, Kalman, Sflp };
static constexpr WaveOrientation wave_orientation_method = WaveOrientation::Madgwick;

// Log the full ORB_test-style CSV set to SD per capture, one timestamped directory
// per wave session (mirrors ORB_test/src/Logger.cpp). The directory is created as
// "<stamp>_tmp" and renamed to "<stamp>" on a clean stop, so an interrupted capture
// is easy to spot. Five files inside, all prefixed "<stamp>_": imu (raw window rows,
// lets postprocess.py compare Madgwick/SFLP/Kalman), gps (drift track sampled during
// the capture), ses (key/value anchor + summary), spec (full elevation PSD), ana.
static constexpr bool     wave_log_csv                    {true};
static constexpr uint16_t wave_csv_sync_rows              {128}; // File.sync() cadence

static constexpr char     wave_log_dir[]     = "waves";  // parent dir for session folders
#define WAVE_IMU_PREFIX      "imu"
#define WAVE_GPS_PREFIX      "gps"
#define WAVE_SESSION_PREFIX  "ses"
#define WAVE_SPEC_PREFIX     "spec"
#define WAVE_ANA_PREFIX      "ana"

// Firmware build sequence, written to the session anchor so a log can be tied back
// to the code that produced it (mirrors ORB_test kBuildSeq). Bump on release.
static constexpr uint16_t wave_build_seq     = 1;

// -----------------------------------------------------------------------------
// IMU: datarate (ODR) + power mode for accel AND gyro.
// -----------------------------------------------------------------------------
static constexpr uint16_t kImuOdrHz    = 960;   // 120/240/480/960 Hz
static constexpr uint8_t  kImuLowPower = 0;     // 1 = low power (ODR<=240), 0 = high performance
static_assert(kImuOdrHz == 120 || kImuOdrHz == 240 || kImuOdrHz == 480 || kImuOdrHz == 960,
              "kImuOdrHz must be 120, 240, 480 or 960 Hz");
static_assert(!(kImuLowPower && kImuOdrHz > 240),
              "Low-power is only valid for ODR <= 240 Hz");

static constexpr LSM6DSV16X_ACC_Operating_Mode_t kImuAccMode =
    kImuLowPower ? LSM6DSV16X_ACC_LOW_POWER_MODE1 : LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE;
static constexpr LSM6DSV16X_GYRO_Operating_Mode_t kImuGyrMode =
    kImuLowPower ? LSM6DSV16X_GYRO_LOW_POWER_MODE : LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE;

// SPI clock for the IMU. LSM6DSV16X is rated max 10 MHz - do NOT exceed (corrupt
// FIFO reads / NaN). 8 MHz drains the FIFO ~4x faster than 2 with margin to 10 MHz.
static constexpr uint32_t kImuSpiHz = 8000000;

static constexpr bool    kUseLpf2 = true;                      // extra LPF2 low-pass on accel
static constexpr uint8_t kLpf2Bw  = LSM6DSV16X_XL_STRONG;      // LPF2 bandwidth

static constexpr float   kSflpOdrHz = 120.0f;                  // on-chip fusion rate
static constexpr uint8_t kSflpGameRotationTag = 0x13;          // FIFO tag: SFLP game rotation vector

// Sensitivity (LSB -> physical unit) for raw FIFO data. MUST match Set_X_FS/Set_G_FS.
static constexpr float kAccSensMgPerLsb   = 0.122f;  // +-4 g
static constexpr float kGyrSensMdpsPerLsb = 70.0f;   // +-2000 dps

// FIFO watermark (words). Scales with ODR to keep the drain cadence ~constant.
static constexpr uint16_t kFifoWatermark = (kImuOdrHz >= 480) ? 128 : 64;

static constexpr uint16_t kAccelOdrHz = kImuOdrHz;

// -----------------------------------------------------------------------------
// Windowing (window-aggregated IMU rows -> imu.csv), ~100 Hz.
// -----------------------------------------------------------------------------
static constexpr uint16_t kOutputRateHz = 100;
static constexpr uint16_t kWindowMs     = 1000 / kOutputRateHz;  // 10 ms @ 100 Hz

// IMU debug: how often update() prints the effective accel/gyro sample rate + mean
// magnitudes (ms). Only emitted when debug_serial is set. 0 disables the printout.
static constexpr uint32_t imu_debug_print_period = {10*s_2_ms};  // 10 s

// -----------------------------------------------------------------------------
// Brake detection (linear |a| over threshold long enough within a window).
// -----------------------------------------------------------------------------
static constexpr float  kBrakeGThreshold = 0.5f;
static constexpr double kBrakeThresholdMg2 =
    (double)(kBrakeGThreshold * 1000.0) * (kBrakeGThreshold * 1000.0);
static constexpr uint16_t kBrakeMinMs = 5;
static constexpr uint16_t kBrakeMinSamples =
    ((kBrakeMinMs * kAccelOdrHz + 999) / 1000) < 1 ? 1
    : (uint16_t)((kBrakeMinMs * kAccelOdrHz + 999) / 1000);

// -----------------------------------------------------------------------------
// Analysis: Madgwick 6-axis AHRS -> vertical acceleration.
// -----------------------------------------------------------------------------
static constexpr float    kMadgwickBeta      = 0.05f;
static constexpr float    kGravity           = 9.80665f;
static constexpr uint16_t kVacc10HzBucketMs  = 100;
static constexpr float    kVaccFsHz          = 1000.0f / kVacc10HzBucketMs;   // 10 Hz
static constexpr float    kMg2Ms2            = kGravity / 1000.0f;            // mg -> m/s^2
static constexpr float    kMdps2Rads         = 1.0e-3f * (float)M_PI / 180.0f; // mdps -> rad/s

// ---- Welch spectrum -> wave parameters ----
// Segment length is 2048 (not ORB_test's live 1024) so the shared welch_bin_min/max
// (9..64 in common_config.h) land at the intended frequencies for a 10 Hz series:
// bin 64 = 64*10/2048 = 0.3125 Hz.
static constexpr uint16_t kWelchSegLen     = 2048;
static constexpr uint16_t kWelchOverlapDiv = 4;      // step = seglen/4 => 75% overlap
static constexpr float    kWaveFMax        = 0.5f;   // upper band edge (Hz)
static_assert((kWelchSegLen & (kWelchSegLen - 1)) == 0, "kWelchSegLen must be a power of two");

// Low-frequency half-cosine taper (Kohout / Tucker & Pitt 2001) on the elevation PSD.
static constexpr float kTaperF1 = 0.03f;   // T=0 below
static constexpr float kTaperF2 = 0.05f;   // T=1 above
static_assert(kTaperF1 < kTaperF2 && kTaperF2 <= kWaveFMax, "need kTaperF1 < kTaperF2 <= kWaveFMax");

enum class WindowType { Hann, Hamming };
static constexpr WindowType kWelchWindow = WindowType::Hann;

// -----------------------------------------------------------------------------
// Kalman (roll/pitch, 2-state: angle + gyro bias). Translated from
// ORB_test/tools/postprocess.py (KalmanAngle). Not present in ORB_test firmware.
// For a wave buoy the accel tilt must be trusted strongly (small R) so the low
// frequency orientation pins to gravity; a large R drifts and omega^-4 amplifies
// it into an unreasonable Hs.
// -----------------------------------------------------------------------------
static constexpr float kKalmanQAngle = 0.001f;
static constexpr float kKalmanQBias  = 0.003f;
static constexpr float kKalmanR      = 0.001f;

#endif  // WAVE_CONFIG_H
