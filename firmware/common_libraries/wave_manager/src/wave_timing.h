#ifndef WAVE_TIMING_H
#define WAVE_TIMING_H

#include <Arduino.h>

#include "wave_config.h"

/*
  Where the capture loop actually spends its time.

  The raw log already says WHEN each drain ran - the sync record carries millis - so a
  stall is visible as a gap between drains. What it cannot say is what the gap was spent
  ON. These buckets close that hole: every stretch of the hot path is timed with micros()
  and reported as n / mean / max, per serial-print interval and per capture.

  The question they exist to answer: a capture overruns the FIFO only if more than
  kFifoDepthWords / kFifoWordsPerSec (213 ms at 480 Hz) passes without a drain. Measured
  against that budget, each bucket's max says whether it could have been the cause.

  Every measurement compiles away when wave_timing_enabled is false - add() and the
  ScopeUs constructor/destructor collapse to nothing, and what is left is a global of
  zeroes that nothing reads.

  micros() rolls over every ~71 minutes. Every reading below is a uint32_t DIFFERENCE,
  and unsigned arithmetic wraps the same way the counter does, so a rollover inside a
  measurement still yields the right interval. Only a single interval longer than the
  full period would be misread, which no operation here can be.
*/

// One timed stretch. Two sets of accumulators, because the two readers ask different
// questions: the serial line wants "how has it been the last 20 s" and ses.csv wants
// "how was this capture", and a single set cannot serve both without one of them
// resetting the other's numbers.
struct TimeStat {
  uint32_t n = 0, sumUs = 0, maxUs = 0;           // since the last serial print
  uint32_t nCap = 0, sumUsCap = 0, maxUsCap = 0;  // since the start of the capture

  void add(uint32_t us) {
    if (!wave_timing_enabled) return;
    n++; sumUs += us; if (us > maxUs) maxUs = us;
    nCap++; sumUsCap += us; if (us > maxUsCap) maxUsCap = us;
  }

  void resetInterval(void) { n = 0; sumUs = 0; maxUs = 0; }
  void resetCapture(void)  { resetInterval(); nCap = 0; sumUsCap = 0; maxUsCap = 0; }

  uint32_t meanUs(void) const    { return n    ? sumUs    / n    : 0; }
  uint32_t meanUsCap(void) const { return nCap ? sumUsCap / nCap : 0; }
};

// RAII guard: one line at the top of the scope being measured, and no matching call to
// forget at the bottom - which matters here, because several of the measured scopes
// have early returns in them.
class ScopeUs {
 public:
  explicit ScopeUs(TimeStat &s) : s_(s), t0_(wave_timing_enabled ? micros() : 0) {}
  ~ScopeUs() { if (wave_timing_enabled) s_.add(micros() - t0_); }

  // The guard owns a measurement in progress; copying it would report that measurement
  // twice.
  ScopeUs(const ScopeUs &) = delete;
  ScopeUs &operator=(const ScopeUs &) = delete;

 private:
  TimeStat &s_;
  uint32_t  t0_;
};

/*
  The buckets, as an ARRAY with named indices rather than as thirteen named members. The
  resets and the ses.csv writer walk them in a loop against kTimingNames, so adding a
  bucket is one enum entry and one string - not a fourteenth line in four places, three
  of which get forgotten.

  They NEST, and reading them means knowing how:

      loop  >  update  >  pop ( > spi, ahrs, fir, rowsink ) + flush + dbgprint + status
      loop  >  synccsv
      loop  >  welch                                        (deferred out of rowsink)
      loop  >  gps

  welch is a SIBLING of update, not inside it, and that is the whole point of the
  deferral: it runs after update() has returned, so its 88 ms competes with an EMPTY
  FIFO rather than a partly drained one. The number in the bucket is the same either
  way - where it sits in this diagram is what changed.

  So the columns do not sum to the capture: an inner bucket is counted again by every
  bucket around it. That is deliberate. The question is never "where did the second go"
  but "which of these could have held the drain past its budget", and for that each one
  has to be readable on its own.

  Two ratios are worth knowing:

      spi n / update n      words per drain - the batch size, and the only place it is
                            visible.
      status n / update n   ~2 in a WTM build (the drain's read plus the re-arm), ~1 in
                            a DRAIN one. Says nothing about batching: the status read
                            sits BELOW both gates, so it runs only on actual drains.
*/
enum TimingBucket : uint8_t {
  TIM_UPDATE = 0,  // one whole drain; both gates excluded (a gated call is not a drain)
  TIM_STATUS,      // ImuDevice::status(): the drain's own read, plus the INT1 re-arm
  TIM_POP,         // the pop loop: SPI + AHRS + FIR + raw buffering for every word
  TIM_SPI,         // popWord alone -> us per FIFO word
  TIM_AHRS,        // ahrs_.update alone -> us per orientation step
  TIM_FIR,         // latchRowValues: 129 taps x 10 channels, once per row - but ONE
                   // channel, not ten, when wave_log_mode drops imu.csv (FirRowBank::
                   // eval). So this bucket is not comparable across log modes: a Raw
                   // capture is ~1/10 of a Csv one because it does a tenth of the work.
  TIM_ROWSINK,     // analyzer ingest + the imu.csv row, once per row
  TIM_FLUSH,       // RawLogWriter::flush -> the raw log's SD write. See flushBytes.
                   // Counts ONLY the drains that actually wrote: with
                   // kRawFlushThreshold most return without touching the card, and
                   // zeros in the bucket would make the mean useless as a measure of
                   // what a write costs.
  TIM_SYNCCSV,     // the periodic imu.csv sync(): FAT + directory, not just a block
  TIM_WELCH,       // the deferred accumSegment: one FFT every 25.6 s, ~88 ms. n is the
                   // segment count and must match welch_segments in ana.csv exactly -
                   // a shortfall means segments are being dropped, not just delayed.
  TIM_GPS,         // serviceGps: the receiver poll and one gps.csv row. n = 0 when
                   // wave_gps_track_in_capture is off - the call is compiled out
  TIM_LOOP,        // one capture-loop iteration, delay() excluded
  TIM_DBGPRINT,    // the serial report itself - 115200 baud is not free
  TIM_COUNT
};

// Short enough for one serial line, and used verbatim as the ses.csv key: a reader of
// either output looks up the same word in the enum above.
inline constexpr const char *kTimingNames[TIM_COUNT] = {
    "update", "status", "pop", "spi", "ahrs", "fir",
    "rowsink", "flush", "synccsv", "welch", "gps", "loop", "dbgprint"};

/*
  One global, in the same spirit as sd_writer and gps_manager: both ImuSampler and
  WaveManager write to it, and threading a reference through two constructors for a
  diagnostic buys nothing.
*/
struct WaveTiming {
  TimeStat b[TIM_COUNT];

  // Bytes handed to the raw sink, so flush can be read as us/kB and not only us/drain.
  // Same two-window split as TimeStat, for the same reason.
  uint32_t flushBytes = 0, flushBytesCap = 0;

  // One-shot costs, outside the loop and therefore no threat to the FIFO - but they are
  // dead time in the capture window, and preAllocate of a ~28 MB reservation is not
  // small. Single values: a TimeStat with n = 1 would only dress them up.
  uint32_t startSessionUs = 0;
  uint32_t stopImuUs = 0, stopGpsUs = 0, stopRawUs = 0, stopTotalUs = 0;

  void addFlushBytes(uint32_t bytes) {
    if (!wave_timing_enabled) return;
    flushBytes += bytes;
    flushBytesCap += bytes;
  }

  // After every serial report. Leaves the per-capture accumulators alone.
  void resetInterval(void) {
    for (uint8_t i = 0; i < TIM_COUNT; i++) b[i].resetInterval();
    flushBytes = 0;
  }

  // At the start of a capture, so ses.csv describes that capture and nothing before it.
  void resetCapture(void) {
    for (uint8_t i = 0; i < TIM_COUNT; i++) b[i].resetCapture();
    flushBytes = flushBytesCap = 0;
    startSessionUs = 0;
    stopImuUs = stopGpsUs = stopRawUs = stopTotalUs = 0;
  }
};

inline WaveTiming wave_timing;

// Sugar for the call sites, so a measurement reads as one word naming the bucket.
// Measures from here to the end of the enclosing scope.
#define WAVE_TIME(bucket) ScopeUs scopeUs_##bucket(wave_timing.b[bucket])

/*
  The explicit pair, for the two cases the RAII guard serves badly: a stretch too long to
  wrap in a block without re-indenting it (the pop loop), and a single call whose result
  is const at the call site (readFifoWord, readFifoStatus) - a block there would force the
  variable out of the initialisation.

  timeAdd tests the flag before reading the clock, so a disabled build makes no call at
  all rather than a call whose result is discarded.
*/
inline uint32_t timeStart(void) { return wave_timing_enabled ? micros() : 0; }
inline void timeAdd(TimingBucket bucket, uint32_t t0) {
  if (wave_timing_enabled) wave_timing.b[bucket].add(micros() - t0);
}

#endif  // WAVE_TIMING_H
