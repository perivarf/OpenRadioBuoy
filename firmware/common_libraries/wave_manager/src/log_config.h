#ifndef LOG_CONFIG_H
#define LOG_CONFIG_H

#include <Arduino.h>

#include "config.h"           
#include "analysis_config.h"

/*
  What a capture writes, and in what format.

  One directory "<stamp>" per capture
  Loggger writes following files, prefixed with "<stamp>_":
*/

// File names
#define WAVE_GPS_PREFIX      "gps" // gps.csv - gps
#define WAVE_SESSION_PREFIX  "ses" // ses.csv - session file, including length of session
#define WAVE_SPEC_PREFIX     "spec" // spec.csv - PSD
#define WAVE_ANA_PREFIX      "ana"  // ana.csv - Statistics calculated basesd on PSD (Significant Wave Height etc)
#define WAVE_CFG_PREFIX      "cfg" // cfg.csv - Most relevant config parameters. Can be used to recreate output together with raw.bin
#define WAVE_IMU_PREFIX      "imu" // imu.csv - imu-readings (depending on log format) 
#define WAVE_RAW_PREFIX      "raw" // raw.bin - imu-readings, binary format, see raw_log.h


// Master switch for sd-logging
static constexpr bool     wave_log_to_sd     {true};
static constexpr uint16_t wave_csv_sync_rows {1024}; // How often to call sd sync

// Row widths for preAllocate(). 
// Too low, and time is spent on increasing file size during measurements, may lead to 
// missing samples. Too high and the file might take longer to allocate, but will be 
// truncated after successful logging. Err on side of caution here, and do a test with
// timing on before deploying.
static constexpr uint16_t wave_imu_row_bytes_max {256};

// Measure on created GPS-file. Change if file definition is changed.
static constexpr uint16_t wave_gps_row_bytes_max {208};

// parent dir for session folders (e.g., "waves" will create directory /waves on sd-card)
static constexpr char wave_log_dir[] = "waves";  

// How often update() prints the effective accel/gyro rate to the serial monitor, in ms.
// 0 disables it
static constexpr uint32_t imu_debug_print_period = {20 * s_2_ms};  // 20 s

// Hot-path timing (wave_timing.h). A diagnostic log: turn it off before deployment.
// Prints to serial in loop, and summary to ses.csv (mean and max timings)
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
// Raw log format. See layout in raw_log.h; 
// Decoded by tools/raw_to_csv.py.
// -----------------------------------------------------------------------------
static constexpr uint32_t kRawMagic         = 0x4257524FUL;  // "ORWB" little-endian
static constexpr uint8_t  kRawFormatVersion = 2;
static constexpr uint8_t  kRawSyncTag       = 0xFF;
static constexpr uint8_t  kRawWordBytes     = kImuFifoWordBytes; // Storing FIFO word as we read it from IMU
static constexpr uint8_t  kRawSyncBytes     = 17;
static constexpr uint8_t  kRawHeaderBytes   = 32;
static constexpr uint16_t kRawBlockBytes    = 512;   // SD block; buffered, not per word

// Worst case for one drain: a full FIFO emptied in a single round, plus its sync
// record. Not theoretical - it is precisely the drain AFTER an sd-stall, when the FIFO
// built up while the card was busy. The buffer has to survive that round.
static constexpr uint16_t kRawWorstDrainBytes =
    (uint16_t)kFifoDepthWords * kRawWordBytes + kRawSyncBytes;   // 256*7 + 17 = 1809

/*
  Flush threshold: how much accumulates before the raw log is written to the sd card
  Should be at least kRawBlockBytes. 
*/
static constexpr uint16_t kRawFlushThreshold = 1 * kRawBlockBytes; //TODO PIF check if 1 is enough, used 2 for field experiments

// The buffer must hold whatever can be left over when a drain starts (kRawFlushThreshold-1), 
// plus the worst drain on top of that (kRawWorstDrainBytes)
static constexpr uint16_t kRawBufBytes = kRawFlushThreshold - 1 + kRawWorstDrainBytes; 

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

// Sync-record flags. 
// FifoOvf means the sensor overwrote words
// WriteFail means error in writing to file
static constexpr uint16_t kRawFlagFifoOvf   = 0x0001;
static constexpr uint16_t kRawFlagWriteFail = 0x0002;

#endif  // LOG_CONFIG_H
