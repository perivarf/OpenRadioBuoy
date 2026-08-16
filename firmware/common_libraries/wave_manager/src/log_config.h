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

// Master switch for sd-logging. Off means startSession() is never called, so no session
// directory is created and NONE of the seven files exist - raw.bin included, despite it
// not being a csv. The capture itself is unaffected: the IMU is sampled, the wave chain
// runs and the radio message goes out either way. wave_log_mode below is a different
// question entirely - it picks the IMU FORMAT, and only once this is true.
static constexpr bool     wave_log_to_sd     {true};
static constexpr uint16_t wave_csv_sync_rows {1024};  // File.sync() cadence

// Row widths for preAllocate(). Without it, too much time goes to allocating clusters
// mid-capture, which can overrun the FIFO. It must run before the first byte is
// written, and the file stays oversized until truncate() at close - so the widths are
// worst case, chosen before the hot path and reclaimed after it.
static constexpr uint16_t wave_imu_row_bytes_max {256};

// The decoded 16-column gps.csv is ~95 B on a real fix and 139 in the worst case every
// field can print (signed lat/lon at 6 decimals, four velocities at 4, three accuracies
// as uint32 millimetres). The old 96 B budget - sized for a compact integer format -
// was therefore outgrown MID-CAPTURE, which is the one failure preAllocate cannot
// absorb: see the extent comment in startSession.
static constexpr uint16_t wave_gps_row_bytes_max {160};

static constexpr char wave_log_dir[] = "waves";  // parent dir for session folders
#define WAVE_IMU_PREFIX      "imu"
#define WAVE_GPS_PREFIX      "gps"
#define WAVE_SESSION_PREFIX  "ses"
#define WAVE_SPEC_PREFIX     "spec"
#define WAVE_ANA_PREFIX      "ana"
#define WAVE_CFG_PREFIX      "cfg"
#define WAVE_RAW_PREFIX      "raw"

// How often update() prints the effective accel/gyro rate to the serial monitor, in ms.
// Only emitted when debug_serial is set; 0 disables it. The line also carries the time
// left of the capture, so one interval governs both "is the drain healthy" and "is this
// thing still counting down".
static constexpr uint32_t imu_debug_print_period = {20 * s_2_ms};  // 20 s

/*
  Hot-path timing (wave_timing.h). A DIAGNOSTIC, not an operational log: turn it off
  again before a deployment meant to last, so the measurement is not part of the numbers
  it was meant to explain.

  What it answers: a capture loses words when more than kFifoFillMs passes without a
  drain. The raw log shows THAT such gaps exist - each sync record stamps its drain with
  millis - but not what the time went to. The buckets measure each part of the loop
  against the same budget: SPI per word, AHRS per sample, FIR per row, the raw-log
  write, the periodic sync, the GPS row, the Welch FFT and the whole iteration.

  THE PRICE, to be read before the numbers are used: two micros() calls per FIFO word
  and two per accel sample, so 300-400 calls on a 128-word drain. micros() is a SysTick
  read at ~0.5-1 us, adding ~0.3 ms to a ~5 ms drain - about 6 %. TIM_SPI and TIM_AHRS
  are therefore systematically a little high and TIM_POP carries both. Good enough to
  find a stall of hundreds of ms; not good enough to quote 19 us/word as exact.

  Reported in two places: a [TIM] line on the serial monitor every
  imu_debug_print_period, and an aggregate block in ses.csv at capture end (which
  survives a field run with nobody watching). The ses.csv block is written from
  processReading, so an interrupted capture gets none - the same caveat as the rest of
  the summary.

  Both report n, MEAN and MAX. Max is the one that matters for overruns: a stall is a
  tail event by definition, and a mean over thousands of drains divides a 300 ms outlier
  into the noise. The mean says what the loop normally costs; the max says whether
  anything in it could have emptied the budget.

  Turning it off returns the flash but leaves the ~312 B the wave_timing global puts in
  .bss - a global with external linkage is not dropped just because nobody reads it.
*/
static constexpr bool wave_timing_enabled {true};

// -----------------------------------------------------------------------------
// Which IMU log a capture writes. Independent files, not two formats of one thing.
//   Csv  - 15.7 kB/s, what the offline chain reads today.
//   Raw  - 8.6 kB/s, needs raw_to_csv.py before the usual tools work.
//   Both - the only mode where the reconstruction can be checked against imu.csv, but
//          Madgwick 480 + SFLP 240 + Both overflows.
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
static constexpr uint8_t  kRawWordBytes     = 7;
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
