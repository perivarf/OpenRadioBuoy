#ifndef WAVE_CONFIG_H
#define WAVE_CONFIG_H

#include <Arduino.h>
#include <LSM6DSV16XSensor.h>

#include "config.h"
#include "madgwick.h"   // Madgwick   \ the two AHRS the orientation selection at
#include "kalman.h"     // KalmanAhrs / the bottom of this file chooses between
#include "fir.h"         // kFirNtap / kFirHalf - the decimation filter's own geometry
#include "fir_coeffs.h"  // the tap tables + firTapsForDecimX10(); knows no rates itself
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
// The selection itself (WaveAhrs / wave_use_sflp) is at the BOTTOM of this file,
// where the tuning constants it constructs from are already defined.
// -----------------------------------------------------------------------------

// Log the full ORB_test-style CSV set to SD per capture, one timestamped directory
// per wave session (mirrors ORB_test/src/Logger.cpp). The directory is created as
// "<stamp>_tmp" and renamed to "<stamp>" on a clean stop, so an interrupted capture
// is easy to spot. Six files inside, all prefixed "<stamp>_": imu (raw window rows,
// lets postprocess.py compare Madgwick/SFLP/Kalman), gps (drift track sampled during
// the capture), ses (key/value anchor + summary, per RUN), cfg (every tuning constant
// below, per BUILD), spec (full elevation PSD), ana.
static constexpr bool     wave_log_csv                    {true};
static constexpr uint16_t wave_csv_sync_rows              {1024}; // File.sync() cadence

// Pre-allocation, and why it is not optional here. open(O_CREAT) makes a file with
// ZERO clusters; the chain is then built one link at a time by addCluster() as the
// write position crosses each cluster boundary - roughly every 1.5 s at 100 rows/s
// with a 32 kB cluster. Every one of those is a FAT write at a completely different
// LBA than the data, i.e. the non-sequential access an SD card is worst at, and a
// card that stalls a few hundred ms there overruns the IMU FIFO (512 words is only
// ~240 ms of headroom at this ODR) and the capture loses samples outright.
//
// FatFile::preAllocate() takes one contiguous run up front instead. The write path
// then hits `isContiguous() && m_fileSize > m_curPosition` and merely increments the
// cluster number - no FAT read, no FAT write, and the card sees a purely sequential
// stream. It also keeps sync() cheap: write() only sets FILE_FLAG_DIR_DIRTY once
// m_curPosition passes m_fileSize, which pre-allocation has already pushed to the
// end, so a periodic sync flushes one sequential data sector and nothing else.
//
// Two consequences. preAllocate() must run before the first byte is written (it
// requires m_firstCluster == 0), and the directory entry claims the full length
// until truncate() hands the tail back at close - so an interrupted capture leaves
// an oversized file. The "_tmp" directory suffix already marks those.
//
// The row bounds are worst-case widths, not averages: over-reserving costs only the
// clusters truncate() frees again, while under-reserving silently drops the file
// back onto the slow path partway through the capture.
static constexpr uint16_t wave_imu_row_bytes_max          {256};
static constexpr uint16_t wave_gps_row_bytes_max          {96};

static constexpr char     wave_log_dir[]     = "waves";  // parent dir for session folders
#define WAVE_IMU_PREFIX      "imu"
#define WAVE_GPS_PREFIX      "gps"
#define WAVE_SESSION_PREFIX  "ses"
#define WAVE_SPEC_PREFIX     "spec"
#define WAVE_ANA_PREFIX      "ana"
#define WAVE_CFG_PREFIX      "cfg"
#define WAVE_RAW_PREFIX      "raw"

// Firmware build sequence, written to the session anchor so a log can be tied back
// to the code that produced it (mirrors ORB_test kBuildSeq). Bump on release.
// 2: AHRS moved to the raw stream and the boxcar means became FIR decimation. Both
//    the row semantics and the column set changed - see kImuCsvHeader.
// 3: imu.csv columns renamed so each one names its orientation source: _sflp for the
//    on-chip fusion, unsuffixed for the selected filter (mqw..mqz became qw..qz). No
//    values or column ORDER changed - see kImuCsvHeader.
static constexpr uint16_t wave_build_seq     = 3;

// How long takeReading() waits for a valid GNSS solution before it gives up and
// aborts the capture. A wave record with no position is not worth the half hour and
// the ~28 MB of card it costs - Hs without a location cannot be assigned to a sea
// state - so a timeout here skips the whole measurement rather than logging a drift
// track that stays empty. The receiver is left in a controlled GNSS stop between
// measurements, so this budget has to cover the hot start (seconds with ephemeris in
// battery-backed RAM, up to a minute or two after a long stop or a cold antenna).
// Charged against the capture, not on top of it: the wait runs before the FIFO
// stream starts, so nothing is being sampled while it ticks.
static constexpr uint32_t wave_gps_fix_timeout {120000};  // ms

// Whether that timeout ABORTS the capture or merely ends the wait. True is the
// deployment setting: a position is part of the measurement, and half an hour of IMU
// that cannot be placed on a map is not worth the card space. False keeps the IMU
// chain running with an empty drift track, which is what a bench or a harbour test
// wants - there the point is the spectrum, not where it was measured.
//
// The wait itself runs either way: a fix that arrives at second 90 still gives the
// capture a position and a correct RTC stamp, and there is nothing else to do with
// that time - the FIFO stream has not started yet. Set wave_gps_fix_timeout to 0 to
// skip the wait entirely.
//
// enable_GPS (config.h) overrules this: requiring a fix from a build with no receiver
// would abort every capture forever, so a false enable_GPS wins.
static constexpr bool wave_measurement_require_gps {true};

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
// the anti-alias work up front, ahead of the two FIR stages rather than instead of
// them - it is what keeps the band above kFirS1CutoffHz nearly empty to begin with.
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

static constexpr float   kSflpOdrHz = 240.0f;                  // on-chip fusion rate
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
// FIFO watermark + INT1. Ported from ORB_test/src/Imu.cpp (kWakeMode::Interrupt),
// where this has been the working arrangement; it was dropped in the first port and
// kFifoWatermark was left behind describing a setting nothing applied.
//
// The sensor drives INT1 high once the FIFO holds kFifoWatermark words, so the drain
// runs on the sensor's cadence instead of the main loop's, and update() costs nothing
// but a flag test when there is nothing to fetch. At 128 words and the current word
// rate that is a burst roughly every 107 ms, well inside the 512-word buffer.
//
// The interrupt is LEVEL-driven, not edge-driven, which is the trap: if a drain ends
// with the FIFO still above the watermark - after a blocking SD flush, say - the line
// never falls, no new rising edge arrives, and the stream dies silently. update()
// therefore re-arms itself whenever a backlog remains. ORB_test learned this the hard
// way; do not remove that check.
//
// Set kImuUseInt1 = false to fall back to pure polling, which is what shipped before
// and remains the behaviour to compare against if INT1 misbehaves on a given board.
// -----------------------------------------------------------------------------
static constexpr bool kImuUseInt1 = true;

// Watermark in FIFO words. Scales with ODR to keep the drain cadence ~constant.
// FIFO_Set_Watermark_Level takes a uint8_t (FIFO_CTRL1.WTM is 8 bits), so this must
// stay under 256 even though the buffer itself is 512 words deep.
static constexpr uint16_t kFifoWatermark = (kImuOdrHz >= 480) ? 128 : 64;
static_assert(kFifoWatermark > 0 && kFifoWatermark < 256,
              "FIFO_CTRL1.WTM is 8 bits - a larger watermark would be truncated");

// INT1_CTRL (0x0D): which events the sensor drives out on the INT1 pin. Raw register
// values because the Arduino wrapper exposes no setter for them, only Write_Reg.
static constexpr uint8_t kInt1CtrlReg = 0x0D;
static constexpr uint8_t kInt1FifoTh  = 0x08;  // bit 3: FIFO watermark reached

// Deadline after which update() drains whether or not INT1 fired. The interrupt is a
// hint about WHEN to drain, never the authority on WHETHER to - a single lost edge
// must not be able to end the capture, and on 2026-08-04 exactly that happened (0 Hz
// for the rest of the run, with no warning, because the counters were never touched
// again either). Chosen below the FIFO's own fill time so the fallback preempts an
// overrun rather than merely reporting one: 512 words at ~2160 words/s is 237 ms at
// the highest supported rate. Firing it spuriously costs two register reads.
static constexpr uint32_t kFifoPollFallbackMs = 150;

// FIFO_DATA_OUT_TAG. The six payload registers follow it (0x79..0x7E), and reading
// 0x7E is what pops the word - so a 7-byte auto-incrementing burst from here takes
// tag and payload out together, which is what makes the pairing atomic.
static constexpr uint8_t kFifoDataOutTagReg = 0x78;

static constexpr uint16_t kAccelOdrHz = kImuOdrHz;

// -----------------------------------------------------------------------------
// THE RATE CHAIN. Every rate below is named for what CONSUMES it, and no rate
// carries its own value in its name - a constant called kVacc10HzBucketMs stops
// being true the moment someone edits it, and the offline mirror had already
// drifted into exactly that lie.
//
//   kImuOdrHz         raw accel/gyro out of the FIFO
//     -> kAhrsInputOdrHz   the orientation filter (= kImuOdrHz, undivided)
//     -> kRowOdrHz         FIR stage 1; the rows written to imu.csv
//     -> kWelchInputOdrHz  FIR stage 2; the series fed to Welch
//
// Each stage states its ODR and DERIVES its period, never both independently, so
// the two can no longer disagree. The static_asserts by the FIR tables below are
// what make an illegal decimation a build error rather than a slow time-base drift
// that only shows up as a wrong Hs months later.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Row rate (FIR-decimated IMU rows -> imu.csv).
//
// This is the row rate written to SD, and it is chosen for the CARD, not for the
// analysis: halving it halves the bytes per second, the number of clusters the file
// walks, and the per-row formatting work in the drain loop - all of which sit
// between the FIFO and the next chance to drain it. The wave chain is unaffected
// either way, since stage 2 decimates to kWelchInputOdrHz regardless.
//
// What it does cost: rows are the only record of anything above kRowOdrHz/2, so an
// offline re-analysis at a wider band is no longer possible from a capture. That band
// was already out of reach - LPF2 sits at kLpf2CutoffHz = 4.8 Hz - so the loss is
// bookkeeping, not signal. Stage 1's cutoff follows automatically (fs_out/2).
// -----------------------------------------------------------------------------
static constexpr uint16_t kRowOdrHz    = 100;
static constexpr uint16_t kRowPeriodMs = 1000 / kRowOdrHz;
static_assert(1000 % kRowOdrHz == 0,
              "kRowOdrHz must divide 1000 - the row grid is kept in whole ms");

// IMU debug: how often update() prints the effective accel/gyro sample rate + mean
// magnitudes (ms). Only emitted when debug_serial is set. 0 disables the printout.
static constexpr uint32_t imu_debug_print_period = {10*s_2_ms};  // 10 s

// -----------------------------------------------------------------------------
// Raw IMU log (<stamp>_raw.bin): every FIFO word, verbatim.
//
// imu.csv is the DECIMATED record - one row per kRowPeriodMs, after FIR stage 1. This
// file is the undecimated one: the 6-byte payload of every FIFO word exactly as the
// sensor produced it, so an offline re-analysis sees the samples the device saw
// rather than a filtered summary of them. That is what wave_config.h's row-rate note
// says was given up, and it is what makes the offline AHRS comparable to the on-board
// one, which runs at kAhrsInputOdrHz rather than at the row rate.
//
// NOT decoded on the way out: no mg, no mdps, no half-float expansion. Decoding is
// lossy in the sense that matters here - it bakes in the sensitivity constants, and
// those are in the header, so the far side can redo it. It is also the expensive part,
// and this runs inside the FIFO drain.
//
// TIME. There is no timestamp per word, deliberately: at kImuOdrHz = 480 the sample
// period is 2.08 ms, so a whole-millisecond stamp would be COARSER than the data it
// labels, while costing 4 bytes on a 7-byte record. The order of the words is the time
// axis; a sync record per drain pins it to the clock with sub-ms resolution and makes
// gaps explicit. sampleTms_ is a double that self-calibrates against the wall clock
// (see ImuSampler), so the reconstructed axis is drift-free.
//
// LAYOUT, little-endian throughout:
//   header, kRawHeaderBytes once at the top
//   word   1 B tag_sensor + 6 B payload                       = kRawWordBytes
//   sync   1 B kRawSyncTag + u32 t_us + u32 accel_n
//                          + u32 millis + u16 n_words + u16 flags = kRawSyncBytes
// A reader dispatches on the tag: kRawSyncTag means kRawSyncBytes, anything else means
// kRawWordBytes. The two can never collide - tag_sensor is the top 5 bits of the FIFO
// tag byte (see readFifoWord), so a sensor tag is at most 0x1F.
// -----------------------------------------------------------------------------
// Which of the two IMU logs a capture writes. They are independent files, not two
// formats of one thing: imu.csv is the DECIMATED record the offline chain reads
// (postprocess.py -> read_imu_rows, and everything built on it), raw.bin is the
// undecimated one. Rates measured on a real capture: 15.7 kB/s and 8.6 kB/s.
//
// Csv  - what shipped before raw.bin existed.
// Raw  - raw.bin only. Nothing offline reads it yet except rawlog.py, so this mode
//        needs raw_to_csv.py in the loop before the usual tools work again.
// Both - use this for the first raw captures: it is the only configuration where the
//        reconstruction can be checked against the device's own imu.csv sample for
//        sample, on the same capture.
//
// The on-board wave analysis is unaffected by all three - it consumes rows through the
// row sink and never reads imu.csv - so ses/cfg/spec/ana are written regardless.
// Note the two-level gate: wave_log_csv above is the MASTER switch for session logging
// (no session directory at all when false, in any mode); wave_log_mode only chooses
// which of the two IMU logs that session contains.
// Madgwick 480 + SFLP 240 + BOTH gives overflows.
enum class WaveLogMode : uint8_t { Csv = 0, Raw = 1, Both = 2 };
static constexpr WaveLogMode wave_log_mode = WaveLogMode::Raw;

static constexpr bool wave_mode_imu_csv(void) {
  return wave_log_mode == WaveLogMode::Csv || wave_log_mode == WaveLogMode::Both;
}
static constexpr bool wave_mode_imu_raw(void) {
  return wave_log_mode == WaveLogMode::Raw || wave_log_mode == WaveLogMode::Both;
}
static constexpr uint32_t kRawMagic          = 0x4257524FUL;  // "ORWB" little-endian
static constexpr uint8_t  kRawFormatVersion  = 1;
static constexpr uint8_t  kRawSyncTag        = 0xFF;
static constexpr uint8_t  kRawWordBytes      = 7;
static constexpr uint8_t  kRawSyncBytes      = 17;
static constexpr uint8_t  kRawHeaderBytes    = 32;
static constexpr uint16_t kRawBlockBytes     = 512;   // SD block; buffered, not per word
static_assert(kRawSyncTag > 0x1F,
              "the sync tag must not collide with a FIFO tag_sensor (top 5 bits)");
static_assert(kRawBlockBytes >= kRawSyncBytes + kRawWordBytes,
              "raw block must hold at least a sync record plus one word");

// Sync-record flag bits. Adding bits does NOT need kRawFormatVersion bumped: the field
// is already a uint16 in v1, and a decoder that does not know a bit ignores it.
//
// The two failures are not the same kind of thing, and the difference is the reason
// both exist. FifoOvf means the SENSOR overwrote words: the file is intact, a stretch
// of time is missing from it, and the time axis is compressed there. WriteFail means
// THIS FILE lost bytes - and because the format is a byte stream where a record may
// straddle a block, losing a partial block does not merely lose data, it desynchronises
// every byte after it: the decoder reads a payload byte as a tag and keeps going. So a
// capture with FifoOvf is usable with care; one with WriteFail is not trustworthy past
// the flag, which is exactly why it must be recorded IN the stream rather than only in
// ana.csv - the damage is positional.
static constexpr uint16_t kRawFlagFifoOvf   = 0x0001;
static constexpr uint16_t kRawFlagWriteFail = 0x0002;

// -----------------------------------------------------------------------------
// Bench test: synthetic wave message.
//
// Enqueues a fabricated WaveResult on a short timer instead of running a capture,
// so the serialise -> LoRa -> SD path can be exercised in seconds rather than after
// a full wave_measurement_duration. What is faked is ONLY the source of the result:
// updateTransmitMessage, LORA.sendData and sd_writer.logByteArray are the production
// ones, which is the whole point - a test that re-implemented the packing would
// prove nothing about the packing that ships.
//
// A BUILD FLAG, not a constant to edit: -DDEBUG_WAVE_MSG=1 (env orb_drifter_test_wave).
// A bool in this header would eventually be committed as true; an #if cannot be,
// because the production env never defines it and the code is not compiled at all.
// -----------------------------------------------------------------------------
#ifndef DEBUG_WAVE_MSG
#define DEBUG_WAVE_MSG 0
#endif
static constexpr uint32_t debug_wave_msg_period = {30*s_2_ms};  // 30 s between fakes

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
// Welch input rate: the end of the chain. ODR is what is SET here; the period is
// derived, so the two cannot drift apart the way kVacc10HzBucketMs/kVaccFsHz could.
static constexpr uint16_t kWelchInputOdrHz    = 10;
static constexpr uint16_t kWelchInputPeriodMs = 1000 / kWelchInputOdrHz;
static_assert(1000 % kWelchInputOdrHz == 0,
              "kWelchInputOdrHz must divide 1000 - the bucket grid is kept in whole ms");
static constexpr float    kMg2Ms2            = kGravity / 1000.0f;            // mg -> m/s^2
static constexpr float    kMdps2Rads         = 1.0e-3f * (float)M_PI / 180.0f; // mdps -> rad/s

// -----------------------------------------------------------------------------
// AHRS rate: the raw stream, undivided.
//
// The AHRS runs on the RAW FIFO stream, so the gyro is integrated at the resolution
// it was measured at and every accel sample drives exactly one update. There is no
// divider: an every-Nth-sample AHRS was a second, hidden decimation sitting next to
// the FIR one, with its own phase convention (kQuatDelaySteps had to stay a whole
// number of AHRS steps) and its own way of being wrong. One rate is cheaper to
// reason about than two, and it makes the quaternion delay a plain sample count.
//
// kAhrsInputOdrCapHz is therefore a CEILING, not a divider - the rate above which the
// orientation filter stops fitting in the CPU budget. KalmanAhrs is ~20x dearer per
// update than Madgwick (6-state MEKF, two 6x6 matmuls in predict + Joseph form in
// correct), so the cap is set for Madgwick: ~18 % of a soft-float 48 MHz core at
// 960 Hz. Building with KalmanAhrs at this ODR does not fit and never did - drop
// kImuOdrHz instead, which the assert below forces you to do deliberately.
// -----------------------------------------------------------------------------
static constexpr uint16_t kAhrsInputOdrCapHz = 960;
static constexpr float    kAhrsInputOdrHz    = (float)kImuOdrHz;
static_assert(kImuOdrHz <= kAhrsInputOdrCapHz,
              "the AHRS runs on every raw sample - kImuOdrHz above kAhrsInputOdrCapHz does "
              "not fit the CPU budget; lower the ODR rather than re-introducing a divider");

// -----------------------------------------------------------------------------
// FIR decimation. Two stages replace what used to be two boxcar means:
//   stage 1: kImuOdrHz -> kRowOdrHz        (the ax..gz/vacc columns in imu.csv)
//   stage 2: kRowOdrHz  -> kWelchInputOdrHz (the series fed to Welch)
//
// A mean over D samples IS an FIR filter, but a poor one: its response leaks badly
// between the nulls, so everything above Nyquist folds into the wave band. The tap
// values live in fir_coeffs.h, generated by ORB_test/tools/gen_fir_table.py from the
// same firwin_lowpass() that postprocess.py uses offline - device and offline filter
// with identical numbers rather than two implementations that drift apart.
//
// Stage 1 is generally NOT an integer decimation (kImuOdrHz/kRowOdrHz = 4.8 at the
// rates shipped so far). The output grid therefore stays time-driven - the kRowPeriodMs
// windows that already exist - and the FIR is evaluated on the raw sample nearest the
// window centre. Residual jitter is at most half a raw period, and winStartMs stays
// exactly on the kRowPeriodMs grid.
//
// Stage 2 is different and MUST be exact: kWelchInputPeriodMs has to be a whole number
// of rows, or the bucket centres slide against the row grid and the decimation drifts
// in time. That is asserted below - the offline mirror only warns about the same
// condition (fir.py), so a configuration the device rejects at compile time can still
// be analysed silently offline.
// -----------------------------------------------------------------------------
// kFirNtap and kFirHalf (the group delay in samples) come from fir.h - they are
// geometry of the filter itself, not of the buoy, and the filter must be buildable on
// a host without this file.
//
// WHICH TABLE, and why the rates are not part of the answer. Both stages use
// cutoff = fs_out/2, and firwin_lowpass only ever sees cutoff/fs - so the taps depend
// on the DECIMATION FACTOR alone, as 1/(2*D). 960 -> 100 and 480 -> 50 are therefore
// the same table, bit for bit, and changing the rates needs no regeneration as long as
// the ratio has an entry in fir_coeffs.h.
//
// The key is 10*D so a non-integer factor like 9.6 stays an exact integer comparison;
// the arithmetic below is exact for every rate on the grid (10*960/100 = 96,
// 10*480/50 = 96, ...). A ratio with no table selects nullptr, and the assert says so
// at compile time rather than letting the wrong coefficients through.
static constexpr float    kFirS1CutoffHz  = 0.5f * kRowOdrHz;   // fs_out/2, fir.py's convention
static constexpr float    kFirS2CutoffHz  = 0.5f * kWelchInputOdrHz;
static constexpr uint16_t kFirS2Decim     = kWelchInputPeriodMs / kRowPeriodMs;  // rows per bucket
static constexpr uint16_t kFirS2CenterMs  = (kFirS2Decim / 2) * kRowPeriodMs;  // = fir.py's dec//2
static constexpr uint16_t kFirS1CenterMs  = kRowPeriodMs / 2;                  // same convention
static constexpr float    kFirS1DelayS    = (float)kFirHalf / (float)kImuOdrHz;
static constexpr float    kFirS2DelayS    = (float)kFirHalf / (float)kRowOdrHz;

static constexpr uint16_t kFirS1DecimX10 = (uint16_t)((10u * kImuOdrHz) / kRowOdrHz);
static constexpr uint16_t kFirS2DecimX10 = (uint16_t)(10u * kFirS2Decim);
static constexpr const float *kFirCoeffsStage1 = firTapsForDecimX10(kFirS1DecimX10);
static constexpr const float *kFirCoeffsStage2 = firTapsForDecimX10(kFirS2DecimX10);

static_assert(10u * kImuOdrHz % kRowOdrHz == 0,
              "kImuOdrHz/kRowOdrHz must land on a whole tenth - the table key is 10*D");
static_assert(kFirCoeffsStage1 != nullptr,
              "no FIR table for kImuOdrHz/kRowOdrHz - add the ratio to DEFAULT_DECIM "
              "in ORB_test/tools/gen_fir_table.py and regenerate fir_coeffs.h");
static_assert(kFirCoeffsStage2 != nullptr,
              "no FIR table for kRowOdrHz/kWelchInputOdrHz - add the ratio to DEFAULT_DECIM "
              "in ORB_test/tools/gen_fir_table.py and regenerate fir_coeffs.h");

static_assert(kFirNtap % 2 == 1,
              "odd tap count - the group delay must be a whole number of samples");
static_assert(kWelchInputPeriodMs % kRowPeriodMs == 0,
              "stage 2 must decimate a whole number of rows - fir.py assumes the same");
static_assert(kRowOdrHz <= kImuOdrHz,
              "the row rate is a DECIMATION of the raw stream - it cannot exceed kImuOdrHz");
static_assert(kWelchInputOdrHz <= kRowOdrHz,
              "the Welch input is a DECIMATION of the rows - it cannot exceed kRowOdrHz");
static_assert(wave_measurement_filter_warm_up > (uint32_t)kFirNtap * kRowPeriodMs + 2000,
              "warm-up must cover FIR start-up (1.29 s) plus AHRS convergence");

// ---- Orientation delay ----
// The quaternions are NOT filtered: attitude is already the output of a heavy
// low-pass (Madgwick's accel correction has a multi-second time constant, and the
// gyro feeding it is band-limited by the sea), so there is nothing to alias, and a
// linear FIR over quaternion components neither preserves |q| = 1 nor survives the
// +-q sign ambiguity. What they DO need is the same delay: the FIR ahead of ax..gz
// is causal, so those columns describe the signal kFirHalf raw samples earlier than
// the row's timestamp. Holding a plain latest-quaternion would make the row describe
// two different instants - ~4 deg of error at 1 Hz / 10 deg tilt. QuatDelay carries
// the attitude the same distance back; see quat_delay.h.
// The AHRS steps once per raw sample, so the delay is the FIR group delay itself -
// no conversion, nothing that has to divide evenly, and zero residual phase error.
static constexpr uint16_t kQuatDelaySteps = kFirHalf;              // 64 raw samples
static constexpr uint16_t kQuatDelaySlots = kQuatDelaySteps + 1;   // 65 (2080 B)

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

static constexpr float kPsdDfHz = (float)kWelchInputOdrHz / kWelchSegLen;  // 0.009766 Hz per bin

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
// The transmitted resolution is NOT the analysis resolution. Each wire bin is the
// BAND AVERAGE of kSpecBinGroup consecutive PSD bins, which decouples the two: the
// moments and Tp keep the full kPsdDfHz, while the message spans the whole wave band
// without the payload growing with it. Covering 0-1 Hz as a raw 1:1 slice would take
// 103 bins; at group 2 it takes 51, i.e. LESS than the 57 the old 0.049-0.596 Hz
// window cost.
//
// Averaging, not decimating, is what makes this lossless in the sense that matters:
// sum_j Shat_j * (G*df) == sum_k S_k * df, so the integral under the transmitted
// curve still equals the integral under the PSD. Picking every G-th bin instead would
// throw away (G-1)/G of the energy and give a shape that depends on where the peak
// happened to land. The averaging also cuts each wire bin's variance by sqrt(G), so
// the shipped spectrum is smoother than a raw slice rather than coarser.
//
//   wire bin  0 = PSD bins   0-1, centre 0.0049 Hz  (taper^2 = 0 here; see below)
//   wire bin 50 = PSD bins 100-101, centre 0.9814 Hz
//   band covered: 0 .. 102 * 0.009766 = 0.9961 Hz
//
// The bottom bins are deliberately kept even though lowFreqTaper zeroes everything
// below kTaperF1 = 0.03 Hz: a spectrum plotted from 0 is easier to read than one that
// starts at an arbitrary offset, and it costs 1.5 wire bins.
static constexpr size_t kSpecBinGroup {2};
static constexpr size_t welch_bin_min {0};
static constexpr size_t welch_bin_max {102};

static_assert(welch_bin_max > welch_bin_min, "empty transmitted bin range");
static_assert(welch_bin_max - welch_bin_min == welch_bins * kSpecBinGroup,
              "transmitted bin range must be welch_bins groups of kSpecBinGroup PSD "
              "bins - welch_bins is the shared wire-format count in common_config.h, "
              "so update both or the base station misparses");
static_assert(welch_bin_max <= kWelchSegLen / 2 + 1,
              "transmitted bins must fit inside the one-sided PSD");
static_assert((welch_bin_max - 1) * kPsdDfHz <= kWaveFMax,
              "top transmitted bin must lie inside the analysed band, otherwise it is "
              "normalised against a peak that was never allowed to see it");

// Frequency axis of the transmitted spectrum, as the receiver must reconstruct it:
//   f_j = kSpecFMinHz + j * kSpecBinWidthHz,  j = 0 .. welch_bins-1
static constexpr float kSpecBinWidthHz = kSpecBinGroup * kPsdDfHz;          // 0.019531 Hz
static constexpr float kSpecFMinHz     = (welch_bin_min + 0.5f * (kSpecBinGroup - 1)) * kPsdDfHz;
static constexpr float kSpecFMaxHz     = kSpecFMinHz + (welch_bins - 1) * kSpecBinWidthHz;

enum class WindowType { Hann, Hamming };
static constexpr WindowType kWelchWindow = WindowType::Hann;

// -----------------------------------------------------------------------------
// Kalman: quaternion error-state EKF with an ADAPTIVE measurement noise, ported
// from ORB_test/tools/kalman.py. See kalman.h for what R's three terms do; the
// values here are that file's, and they were swept there against the measured
// noise floor (mg/sqrt(Hz)) on the Skjaerhalden captures - not guessed:
//
//   r0     1e-7 -> 8.36,  1e-6 -> 6.34,  1e-5 -> 4.30,  1e-4 -> 4.85,  1e-3 -> 6.24
//   lambdaA  0 -> 4.299,  25 -> 4.315,  100 -> 4.535    (no effect - see kalman.h)
//   lambdaW  0 -> 6.543,   2 -> 4.299,   10 -> 4.653    (the whole gain, -34 %)
//
// THE r0 SWEEP ABOVE IS NOT THE WHOLE STORY, and the value below no longer follows
// it. Those numbers were measured on a 50 Hz capture (20260731_131527). Re-running
// the sweep with ORB_test/tools/firmware_test.py on 2026-08-04, scoring the
// 0.03-0.15 Hz acceleration-PSD floor against Madgwick's on the same capture:
//
//   capture              rows    r0=1e-5   r0=1e-3   r0=1e-2
//   20260731_131522     100 Hz     4.72x     1.11x     1.10x
//   20260731_122652     100 Hz     3.08x     1.01x     1.22x
//   20260731_110314     100 Hz     2.94x     1.06x     1.20x
//   20260731_131527      50 Hz     0.34x     0.38x     1.24x
//
// The three 100 Hz captures want 1e-3; the 50 Hz one - the very capture the sweep
// above was run on - wants 1e-5, and reproduces that sweep's ordering. So the two
// measurements agree per capture and disagree across them. r0 = 1e-3 is the choice
// because it is the one that is never much worse than Madgwick anywhere (1.01-1.11x
// on three, 0.38x on the fourth), where 1e-5 is 3-5x worse on three of four. The
// 0.4-0.6 Hz wave band is unchanged at every value, so this costs no signal.
//
// Two things left open: 20260731_131527 has a floor 40x the others and is the only
// capture where Kalman beats Madgwick at all, so it may be a different sea state or
// a different unit rather than a rate effect; and lambdaA was measured as a no-op
// while R was ~100x smaller, which is not the same as measuring it now.
// ALL of this is four captures from one site on one day. Re-measure before trusting.
//
// Rate invariance interacts with this, and the AHRS rate is now kImuOdrHz itself.
// dtRef = 20 ms is the rate the ORIGINAL sweep ran at, so its factor is exactly 1.
// The AHRS runs on the raw stream at kAhrsInputOdrHz (= kImuOdrHz). The worked
// example below was written when that was 960 Hz, where the factor is
// dtRef/dt = 0.020*960 = 19.2, so the EFFECTIVE R on the device is 19.2*r0. The
// 2026-08-04 sweep ran on 100 Hz rows (factor 2), so its plateau of 1e-3..1e-2 is
// an effective R of 2e-3..2e-2 - and 19.2*1e-3 = 1.9e-2 sits at the TOP EDGE of
// that plateau, where it used to sit in the middle at 240 Hz (4.8e-3). Rate
// invariance means the same bandwidth, not the same per-sample variance, so this is
// not automatically wrong - but a Kalman build at this ODR should re-sweep r0
// downward (3e-4 puts the effective R back near the plateau centre) rather than
// assume the number above still applies. r0 cannot be reasoned about at all without
// knowing kAhrsInputOdrHz - which is the point: at the ODR shipped today, 480 Hz,
// the factor is 9.6 rather than 19.2, so the example's arithmetic has to be redone
// for whatever kImuOdrHz is set to, not read off as a constant.
//
// Fixed-R history, since it is the mistake worth not repeating: the first version
// of KalmanAhrs had r constant and left a flat 8.5e-4 (m/s^2)^2/Hz tilt-leakage
// floor below 0.25 Hz, which omega^-4 turned into Hs 0.222 m where Madgwick and
// SFLP both said 0.097 m (Skjaerhalden 20260731_110314). With the adaptive R the
// same filter lands at 0.135 m. Re-measure through postprocess.py's MEKF column
// (ORB_test/tools/mekf.py mirrors this filter) after any change here.
//
// NB: the accel unit is now load-bearing - it must be m/s^2, since (|a|-g)/g asks
// how far the sample is from 1 g. wave_analysis.cpp converts before it feeds the
// AHRS; Madgwick normalises and does not care either way.
// -----------------------------------------------------------------------------
static constexpr KalmanAhrsParams kKalmanParams = {
    /* sigmaG  */ 0.005f,        // rad/s/sqrt(Hz), ~0.3 deg/s/sqrt(Hz)
    /* sigmaB  */ 1.0e-5f,       // rad/s^2/sqrt(Hz)
    /* r0      */ 1.0e-3f,       // was 1e-5 - see the 2026-08-04 sweep above
    /* dtRef   */ 0.020f,        // s - the rate the ORIGINAL sweep above was run at
    /* lambdaA */ 0.0f,
    /* lambdaW */ 2.0f,
    /* w0      */ 1.0f,          // rad/s
    /* gravity */ kGravity,
    /* p0Angle */ 5.0f * (float)M_PI / 180.0f,    // 5 deg
    /* p0Bias  */ 1.0f * (float)M_PI / 180.0f,    // 1 deg/s
};

// -----------------------------------------------------------------------------
// THE orientation selection. Which filter the drifter runs is decided here, at
// compile time - only the selected one is linked into the firmware, and only its
// state occupies RAM. It sits at the BOTTOM of this file because the constructor
// arguments are the tuning constants above.
//
// Madgwick and KalmanAhrs deliberately share the same API (reset / initFromAccel /
// update(gx..az,dt) / quaternion), so switching is a typedef - no interface, no
// virtual calls. makeWaveAhrs() exists only because the two take different
// constructor arguments; swap the two lines together.
// -----------------------------------------------------------------------------
using WaveAhrs = Madgwick;
inline WaveAhrs makeWaveAhrs(void) { return WaveAhrs{kMadgwickBeta}; }

// Kalman alternative. Measured cost of the swap: +200 B RAM, +1792 B flash. The CPU
// is the reason to think twice, not the memory: the AHRS now steps once per raw
// sample, so at kImuOdrHz = 960 KalmanAhrs would need several times the whole core.
// The knob is kImuOdrHz, not an AHRS divider - drop it to 240 Hz, which also shrinks
// kQuatDelaySlots and every FIR delay line with it, and re-sweep r0 (see above).
//   using WaveAhrs = KalmanAhrs;
//   inline WaveAhrs makeWaveAhrs(void) { return WaveAhrs{kKalmanParams}; }

// Feed the wave chain the chip's own SFLP fusion instead of the AHRS above. SFLP
// arrives ready-made in every IMU row (quaternion + gravity-compensated vertical
// accel) and is logged in every row regardless of this flag; setting it true makes
// that column the one the spectrum, Hs/Tz/Tp and the LoRa message are built from.
static constexpr bool wave_use_sflp = false;

// What produced the vacc column, for ses.csv / cfg.csv. Derived, so a capture
// folder can never disagree with the code that filled it.
static constexpr const char *wave_orientation_name = wave_use_sflp ? "SFLP" : WaveAhrs::kName;

#endif  // WAVE_CONFIG_H
