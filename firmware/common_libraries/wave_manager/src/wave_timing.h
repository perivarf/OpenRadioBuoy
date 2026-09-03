#ifndef WAVE_TIMING_H
#define WAVE_TIMING_H

#include <Arduino.h>

#include "wave_config.h"

/*
  Measuring where the capture loop spends its time.

  The raw log already says when each drain ran, but it cannot say is what the time was spent on.
  Most of the hot path is timed with micros() and reported as n / mean / max, 
  per serial-print interval and per capture/session (written to file).

  Every measurement compiles away when wave_timing_enabled is false
*/

// Two set of accumulators, one for serial print, and one for the entire capture (-Cap)
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
  // Constructor, initializing s_ and t0_.
  explicit ScopeUs(TimeStat &s) : s_(s), t0_(wave_timing_enabled ? micros() : 0) {}

  //Destructor
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
  The buckets, as an ARRAY with named indices

  Measurements nest:

      loop  >  update  >  pop ( > spi, ahrs, fir, rowsink ) + flush + dbgprint + status
      loop  >  synccsv
      loop  >  welch
      loop  >  gps

  I.e. the values do not sum to the capture.

  Two important ratios:

      spi n / update n      words per drain - the batch size, and the only place it is
                            visible.
      status n / update n   ~2 in a WTM build (the drain's read plus the re-arm), ~1 in
                            a DRAIN one. Says nothing about batching: the status read
                            sits BELOW both gates, so it runs only on actual drains.
*/
enum TimingBucket : uint8_t {
  TIM_UPDATE = 0,  // one whole drain
  TIM_STATUS,      // ImuDevice::status(): the drain's own read, plus the INT1 re-arm
  TIM_POP,         // the pop loop: SPI + AHRS + FIR + raw buffering for every word
  TIM_SPI,         // popWord alone -> us per FIFO word
  TIM_AHRS,        // ahrs_.update alone -> us per orientation step
  TIM_FIR,         // latchRowValues: N taps x 10 channels, once per row - but ONE
                   // channel, not ten, when wave_log_mode drops imu.csv
  TIM_ROWSINK,     // analyzer ingest + the imu.csv row, once per row
  TIM_FLUSH,       // RawLogWriter::flush -> the raw log's SD write.
                   // only counts the drains that actually wrote
  TIM_SYNCCSV,     // the periodic imu.csv sync()
  TIM_WELCH,       // welch - PSD - estimation
  TIM_GPS,         // serviceGps: the receiver poll and one gps.csv row
  TIM_LOOP,        // one capture-loop iteration, delay() excluded
  TIM_DBGPRINT,    // the serial report itself
  TIM_COUNT        // Keep this as last element, it is the enum count.
};

// Short enough for one serial line, and used verbatim as the ses.csv key
inline constexpr const char *kTimingNames[TIM_COUNT] = {
    "update", "status", "pop", "spi", "ahrs", "fir",
    "rowsink", "flush", "synccsv", "welch", "gps", "loop", "dbgprint"};


struct WaveTiming {
  TimeStat b[TIM_COUNT];

  // Bytes handed to the raw sink, so flush can be read as us/kB and not only us/drain.
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

  // After every serial report
  void resetInterval(void) {
    for (uint8_t i = 0; i < TIM_COUNT; i++) b[i].resetInterval();
    flushBytes = 0;
  }

  // At the start of a capture
  void resetCapture(void) {
    for (uint8_t i = 0; i < TIM_COUNT; i++) b[i].resetCapture();
    flushBytes = flushBytesCap = 0;
    startSessionUs = 0;
    stopImuUs = stopGpsUs = stopRawUs = stopTotalUs = 0;
  }
};

inline WaveTiming wave_timing;

// Measures from here to the end of the enclosing scope.
// ##bucket replaced with bucket-text
// I.e. WAVE_TIME(TIM_POP); -> ScopeUs scopeUs_TIM_POP(wave_timing.b[TIM_POP]);
#define WAVE_TIME(bucket) ScopeUs scopeUs_##bucket(wave_timing.b[bucket])

/*
  In case we cannot use WAVE_TIME
  timeStart -> call to get starttime
  timeAdd -> call to add time to statistic bucket
*/
inline uint32_t timeStart(void) { return wave_timing_enabled ? micros() : 0; }
inline void timeAdd(TimingBucket bucket, uint32_t t0) {
  if (wave_timing_enabled) wave_timing.b[bucket].add(micros() - t0);
}

#endif  // WAVE_TIMING_H
