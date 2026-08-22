#ifndef LOG_CONFIG_H
#define LOG_CONFIG_H

#include <Arduino.h>

#include "config.h"           // s_2_ms, wave_measurement_duration
#include "analysis_config.h"  // rates and Welch settings the derivations below need

/*
  What a capture writes, and in what format.

  One directory "<stamp>" per capture, never renamed, with up to seven files inside all
  prefixed "<stamp>_": imu, gps, ses, cfg, spec, ana, raw.

  An interrupted capture is told apart by what is MISSING, not by the folder name.
  spec/ana are written by processReading and ses.csv gets its summary block there, so a
  folder without ana.csv - or a ses.csv with no stop_utc_epoch - never reached the end.
  That follows from the write order and needs no separate marker to stay in sync with it.
*/

// Master switch for sd-logging
static constexpr bool     wave_log_to_sd     {true};
static constexpr uint16_t wave_csv_sync_rows {1024};  // File.sync() cadence

// Row widths for preAllocate(). 
// Too low, and time is spent on increasing file size during measurements, may lead to 
// missing samples. Too high and the file might take longer to allocate, but will be 
// truncated after successful logging. Err on side of caution here, and do a test with
// timing on before deploying.
static constexpr uint16_t wave_imu_row_bytes_max {256};

// Measured on created GPS-file. Change if file definition is changed.
// 208 and not 192 since the itow column: iTOW runs to 604799999 near the end of a GPS
// week, so ten digits plus the comma. Re-measure on a real file after changing the row.
static constexpr uint16_t wave_gps_row_bytes_max {208};

static constexpr char wave_log_dir[] = "waves";  // parent dir for session folders
#define WAVE_IMU_PREFIX      "imu"
#define WAVE_GPS_PREFIX      "gps"
#define WAVE_SESSION_PREFIX  "ses"
#define WAVE_SPEC_PREFIX     "spec"
#define WAVE_ANA_PREFIX      "ana"
#define WAVE_CFG_PREFIX      "cfg"
#define WAVE_RAW_PREFIX      "raw"

// How often update() prints the effective accel/gyro rate to the serial monitor, in ms.
// 0 disables it
static constexpr uint32_t imu_debug_print_period = {20 * s_2_ms};  // 20 s

// Hot-path timing (wave_timing.h). A diagnostic log: turn it off before deployment.
// Prints to serial, but also to ses.csv
static constexpr bool wave_timing_enabled {true};

// -----------------------------------------------------------------------------
// Which IMU log a capture writes. Independent files, not two formats of one thing.
//   Csv  - Human readable
//   Raw  - Binary file for speed. Use raw_to_csv.py to convert to csv
//   Both - Both above. May not be possible depending on IMU ODR / GPS etc. Test before deploying.
// -----------------------------------------------------------------------------
enum class WaveLogMode : uint8_t { Csv = 0, Raw = 1, Both = 2 };
static constexpr WaveLogMode wave_log_mode = WaveLogMode::Raw;

static constexpr bool wave_mode_imu_csv(void) {
  return wave_log_mode == WaveLogMode::Csv || wave_log_mode == WaveLogMode::Both;
}
static constexpr bool wave_mode_imu_raw(void) {
  return wave_log_mode == WaveLogMode::Raw || wave_log_mode == WaveLogMode::Both;
}

// -----------------------------------------------------------------------------
// Raw log format. The full layout is documented at the top of raw_log.h; these are the
// constants it is built from. Decoded by tools/raw_to_csv.py.
// -----------------------------------------------------------------------------
static constexpr uint32_t kRawMagic         = 0x4257524FUL;  // "ORWB" little-endian
static constexpr uint8_t  kRawFormatVersion = 2;
static constexpr uint8_t  kRawSyncTag       = 0xFF;
// The raw record IS the FIFO word, byte for byte - so this is the device's geometry and
// not a number of the log format's own. See kImuFifoWordBytes in imu_device.h.
static constexpr uint8_t  kRawWordBytes     = kImuFifoWordBytes;
static constexpr uint8_t  kRawSyncBytes     = 17;
static constexpr uint8_t  kRawHeaderBytes   = 32;
static constexpr uint16_t kRawBlockBytes    = 512;   // SD block; buffered, not per word

// Worst case for one drain: a full FIFO emptied in a single round, plus its sync
// record. Not theoretical - it is precisely the drain AFTER an sd-stall, when the FIFO
// built up while the card was busy. The buffer has to survive that round.
static constexpr uint16_t kRawWorstDrainBytes =
    (uint16_t)kFifoDepthWords * kRawWordBytes + kRawSyncBytes;   // 256*7 + 17 = 1809

/*
  FLUSH THRESHOLD: how much accumulates before the raw log touches the card.

  Writing after EVERY drain meant 465 B at watermark 64, ~18 times a second - all under
  a sector and none on a sector boundary. Two things followed:

    * Every write went through SdFat's single 512-byte data cache with a memcpy and a
      read-modify-write. A write covering whole sectors takes SdFat's direct path
      instead and pushes them straight from the buffer to the card. It also stops the
      raw log fighting gps.csv over that one cache block.
    * One sector per command. Aligned multi-sector writes become a single CMD25
      sequence, and the busy phase between blocks inside one is shorter than a full
      commit per block.

  What it does NOT do: fewer block writes. 8 kB/s is 16 sectors a second regardless, and
  the card's garbage collection scales with bytes written, so the ~516 ms outliers stay
  just as likely. The gain is in the mean, not the tail - the overruns still need FIFO
  margin or a different card.

  Two sectors and not four: most of the gain (beyond the cache) arrives at one whole
  sector, and from 2 to 4 only amortises command overhead further while the buffer, and
  so the RAM, grows linearly. With 64 kB on the WLE5, 2 is most of the gain at half the
  cost.
*/
static constexpr uint16_t kRawFlushThreshold = 2 * kRawBlockBytes;   // 1024

// The buffer must hold whatever can be left over when a drain starts, plus one worst
// drain on top. The remainder is at most kRawFlushThreshold - 1: either the flush wrote
// and left under a sector, or it never reached the threshold and left everything.
//
// NOT rounded up to whole sectors - it is the write LENGTH that must be sector aligned,
// not the buffer, and rounding here would be 240 B of RAM with no function. Derived
// rather than written as a number, so the three constants cannot drift apart.
static constexpr uint16_t kRawBufBytes =
    kRawFlushThreshold - 1 + kRawWorstDrainBytes;   // 1024 - 1 + 1809 = 2832

static_assert(kRawSyncTag > 0x1F,
              "the sync tag must not collide with a FIFO tag_sensor (top 5 bits)");
static_assert(kRawBlockBytes >= kRawSyncBytes + kRawWordBytes,
              "raw block must hold at least a sync record plus one word");
static_assert(kRawFlushThreshold % kRawBlockBytes == 0 && kRawFlushThreshold > 0,
              "the flush threshold must be a whole number of sectors, or the write is "
              "never sector aligned and SdFat falls back to its cache - which is the "
              "entire point of having a threshold");
static_assert(kRawBufBytes >= (uint32_t)kRawFlushThreshold - 1 + kRawWorstDrainBytes,
              "rawBuf_ must hold the sub-threshold remainder PLUS one full drain - a "
              "flush inside the pop loop is the very thing this buffer exists to avoid");

// Sync-record flags. FifoOvf means the SENSOR overwrote words: the file is intact, a
// stretch of time is missing. WriteFail means THIS FILE lost bytes, which desynchronises
// everything after it - so it has to be recorded in the stream, because the damage is
// positional.
static constexpr uint16_t kRawFlagFifoOvf   = 0x0001;
static constexpr uint16_t kRawFlagWriteFail = 0x0002;

// Bench test: enqueue a fabricated WaveResult instead of running a capture, so the
// serialise -> LoRa -> SD path can be exercised at once. Only the source is faked.
#ifndef DEBUG_WAVE_MSG
#define DEBUG_WAVE_MSG 0
#endif

#endif  // LOG_CONFIG_H
