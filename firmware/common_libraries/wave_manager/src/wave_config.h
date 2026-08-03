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

// Human-readable name for the session log (ses.csv), so a capture folder says which
// method produced its vacc column without decoding the enum value.
static constexpr const char *orientationName(WaveOrientation m) {
  return m == WaveOrientation::Madgwick ? "Madgwick"
       : m == WaveOrientation::Kalman   ? "Kalman"
       :                                  "SFLP";
}

// Log the full ORB_test-style CSV set to SD per capture, one timestamped directory
// per wave session (mirrors ORB_test/src/Logger.cpp). The directory is created as
// "<stamp>_tmp" and renamed to "<stamp>" on a clean stop, so an interrupted capture
// is easy to spot. Six files inside, all prefixed "<stamp>_": imu (raw window rows,
// lets postprocess.py compare Madgwick/SFLP/Kalman), gps (drift track sampled during
// the capture), ses (key/value anchor + summary, per RUN), cfg (every tuning constant
// below, per BUILD), spec (full elevation PSD), ana.
static constexpr bool     wave_log_csv                    {true};
static constexpr uint16_t wave_csv_sync_rows              {128}; // File.sync() cadence

static constexpr char     wave_log_dir[]     = "waves";  // parent dir for session folders
#define WAVE_IMU_PREFIX      "imu"
#define WAVE_GPS_PREFIX      "gps"
#define WAVE_SESSION_PREFIX  "ses"
#define WAVE_SPEC_PREFIX     "spec"
#define WAVE_ANA_PREFIX      "ana"
#define WAVE_CFG_PREFIX      "cfg"

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

// -----------------------------------------------------------------------------
// Accelerometer LPF2 low-pass. The LSM6DSV16X specifies this bandwidth as a FRACTION
// OF ODR (CTRL8.hp_lpf2_xl_bw), never as an absolute frequency, so pinning one enum
// makes the real cutoff move whenever kImuOdrHz changes: a fixed STRONG is 9.6 Hz at
// 960 Hz ODR but 1.2 Hz at 120 Hz, which would sit on top of the analysed band and
// quietly bias the spectrum. Derive it from the ODR instead.
//
// The divisors are datasheet values - the driver defines only the enum names, not
// what they mean.
// -----------------------------------------------------------------------------
static constexpr bool kUseLpf2 = true;

// Choose the lowest (strongest) cutoff that still clears the analysed band edge
// kWaveFMax = 1.0 Hz by ~4x, while landing near the 5 Hz Nyquist of the 10 Hz
// vertical-acceleration series this is eventually decimated to. That way LPF2 does
// the anti-alias work up front, ahead of the 10 ms and 100 ms boxcar averaging, which
// alone leave 5-10 Hz content to fold back into the wave band.
static constexpr uint16_t lpf2DivForOdr(uint16_t odr) {
  return odr >= 960 ? 200   // 960/200 = 4.8 Hz
       : odr >= 480 ? 100   // 480/100 = 4.8 Hz
       : odr >= 240 ?  45   // 240/45  = 5.3 Hz
       :               20;  // 120/20  = 6.0 Hz - coarsest usable; /45 would be 2.7 Hz
}
static constexpr uint16_t kLpf2Div = lpf2DivForOdr(kImuOdrHz);

// Divisor -> CTRL8.hp_lpf2_xl_bw register value. kLpf2Div is the single source of
// truth, so the register setting and the cutoff in Hz below cannot disagree.
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

static constexpr float   kSflpOdrHz = 120.0f;                  // on-chip fusion rate
static constexpr uint8_t kSflpGameRotationTag = 0x13;          // FIFO tag: SFLP game rotation vector

// -----------------------------------------------------------------------------
// Full-scale range for accel and gyro. Pick one value each; the enum value IS the
// number passed to Set_X_FS/Set_G_FS (in g / dps), and the raw-FIFO sensitivity
// (LSB -> physical unit) is derived from it below - so the range and the scaling
// can never drift out of sync. A larger range captures bigger motion but with
// coarser resolution (e.g. +-2 g is ~2x finer than +-4 g).
// -----------------------------------------------------------------------------
enum class AccelFS : uint8_t  { G2 = 2, G4 = 4, G8 = 8, G16 = 16 };
enum class GyroFS  : uint16_t { DPS125 = 125, DPS250 = 250, DPS500 = 500,
                                DPS1000 = 1000, DPS2000 = 2000, DPS4000 = 4000 };

static constexpr AccelFS kAccelFS = AccelFS::G2;      // +-2 g
static constexpr GyroFS  kGyroFS  = GyroFS::DPS500;  // +-500 dps

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
// On the 10 Hz vertical-acceleration series a 1024-sample segment gives
// df = 10/1024 = 0.009766 Hz and a 102.4 s segment, so a 30 min capture averages 67
// segments at 75% overlap. 2048 resolved finer but halved the segment count (32),
// leaving a noisier PSD, and cost ~14 kB more RAM (segBuf_ + psdAcc_ + the FFT
// scratch buffers) - which matters here. A fetch-limited fjord peaks at 0.15-0.6 Hz;
// 1024 places ~15 bins across a 0.3 Hz peak, far more than Tp needs.
static constexpr uint16_t kWelchSegLen     = 1024;
static constexpr uint16_t kWelchOverlapDiv = 4;      // step = seglen/4 => 75% overlap
static_assert((kWelchSegLen & (kWelchSegLen - 1)) == 0, "kWelchSegLen must be a power of two");

static constexpr float kPsdDfHz = kVaccFsHz / kWelchSegLen;  // 0.009766 Hz per bin

// Upper edge of the analysed band: the moment integration (m0/m2/m4) and the peak
// search both stop here. It is a physical limit, not anti-aliasing - Nyquist for a
// 10 Hz series is 5 Hz. 1.0 Hz keeps the whole fetch-limited wave band, including the
// ~0.56 Hz peak of a light breeze over a short fetch, which a 0.5 Hz edge truncated
// (understating m0/Hs and pinning Tp to the last bin below the cut).
//
// The moments are NOT equally protected from this edge, and m4 is not protected at
// all. Substituting the acc->elevation conversion S_eta = S_acc/omega^4 into each:
//   m0 = int S_eta df           -> ~f^-4 rolloff, dominated by the peak. Safe.
//   m2 = int S_eta f^2 df       -> ~f^-2 rolloff. Fairly safe.
//   m4 = int S_eta f^4 df = (1/(2*pi)^4) int S_acc df
// The f^4 weight cancels the omega^4 division exactly, so m4 is just the variance of
// the raw vertical acceleration over the band - flat, with no rolloff to suppress
// accelerometer noise or buoy heave resonance near the edge. (For a Phillips f^-5
// tail the integrand goes as f^-1 and m4 diverges logarithmically, so it never
// converges on its own.) Raising this constant therefore lowers Tc = sqrt(m2/m4).
//
// Consequence: Hs and Tz are trustworthy across configurations, but Tc is a
// band-edge-dependent diagnostic and must not be compared between captures logged
// with different kWaveFMax - hence it, and this edge, land in cfg.csv.
static constexpr float kWaveFMax = 1.0f;

// Placed here rather than beside kLpf2Bw because kWaveFMax is only defined this far
// down: the analog-side low-pass must not eat into the band the analysis integrates.
static_assert(kLpf2CutoffHz >= 4.0f * kWaveFMax,
              "accel LPF2 cutoff has fallen into the analysed band - lower kWaveFMax "
              "or widen the divisor picked by lpf2DivForOdr");

// Low-frequency half-cosine taper (Kohout / Tucker & Pitt 2001) on the elevation PSD.
static constexpr float kTaperF1 = 0.03f;   // T=0 below
static constexpr float kTaperF2 = 0.05f;   // T=1 above
static_assert(kTaperF1 < kTaperF2 && kTaperF2 <= kWaveFMax, "need kTaperF1 < kTaperF2 <= kWaveFMax");

// ---- Transmitted spectrum slice ----
// Which PSD bins are quantised into the LoRa message. welch_bins (the count) is the
// shared wire-format constant in common_config.h; the range that produces it is
// drifter-side physics and belongs here, next to the seglen and fs that give the bins
// a frequency at all.
//
// Sized for a fetch-limited fjord, where the peak sits at 0.15-0.6 Hz. Open-ocean
// swell below ~0.05 Hz simply is not present there, so bins spent on it are wasted
// payload; the old 0.044-0.308 Hz window instead cut off the common moderate-wind
// cases at the top.
//   bin  5 =  5 * 0.009766 = 0.0488 Hz -> 20.5 s   (taper^2 = 0.98, effectively full)
//   bin 61 = 61 * 0.009766 = 0.5957 Hz ->  1.7 s   (welch_bin_max is exclusive)
static constexpr size_t welch_bin_min {5};
static constexpr size_t welch_bin_max {62};

static_assert(welch_bin_max > welch_bin_min, "empty transmitted bin range");
static_assert(welch_bin_max - welch_bin_min == welch_bins,
              "transmitted bin range must match welch_bins, the shared wire-format "
              "count in common_config.h - update both or the base station misparses");
static_assert(welch_bin_max <= kWelchSegLen / 2 + 1,
              "transmitted bins must fit inside the one-sided PSD");
static_assert((welch_bin_max - 1) * kPsdDfHz <= kWaveFMax,
              "top transmitted bin must lie inside the analysed band, otherwise it is "
              "normalised against a peak that was never allowed to see it");

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
