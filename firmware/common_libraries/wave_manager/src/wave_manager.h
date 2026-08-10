#ifndef WAVE_MANAGER_H
#define WAVE_MANAGER_H

#include <Arduino.h>
#include <SdFat.h>
#include "config.h"
#include "etl/deque.h"
#include "readings.h"        // wave_analysis_Reading, wave_message_size, welch_bins
#include "imu_sampler.h"
#include "wave_analysis.h"
#include "wave_config.h"
#include "gps_manager.h"     // gps_manager.update()/freshFix()/lastFix() for gps.csv

/*
  Internal wave result. Kept in floats (physical units) so ana.csv logs the real
  values; updateTransmitMessage scales them to the fixed-point uint32 layout that
  wave_analysis_Reading / parse_wave_analysis_message expect.
*/
struct WaveResult {
  uint16_t reading_ID;
  float    Hs;         // significant wave height (m)
  float    Tc;         // crest period (s)
  float    Tp;         // peak period (s)
  float    Tz;         // zero-crossing period (s)
  float    max_value;  // peak elevation PSD value
  uint16_t wave_spectrum[welch_bins];  // quantised elevation PSD, bins welch_bin_min..max
  time_t   timestamp_start;
  time_t   timestamp_end;
};

/*
  Wave manager: owns the IMU sampler and the streaming wave analyzer, and follows
  the same manager contract as thermo_manager / gps_manager
  (begin/wake/sleep/takeReading/processReading/updateTransmitMessage + msgB).
*/
class WaveManager {
 public:
  etl::deque<WaveResult, max_number_of_wave_measurements> wave_analysis_results;

  void begin(void);
  void wake(void);
  void sleep(void);

  // Capture IMU data for wave_measurement_duration, streaming each window into the
  // analyzer (and, when logging is enabled, to imu.csv).
  //   0 = capture ran
  //   1 = no IMU (begin() failed) - nothing was sampled
  //   2 = no GNSS fix within wave_gps_fix_timeout, and wave_measurement_require_gps
  //       is set - capture deliberately skipped. With the flag clear the capture runs
  //       anyway and this is never returned; ses.csv records gps_fix_at_start 0.
  // Both non-zero codes mean the analyzer holds nothing, so the caller must not go
  // on to processReading().
  uint8_t takeReading(void);

  // Finalise the Welch spectrum -> wave params, push a WaveResult, write spec/ana.
  // Returns 0 on success, non-zero if no usable spectrum was produced.
  uint8_t processReading(void);

  // Serialise the front result into msgB ('W' ... 'E') and return the length, or 0
  // if the queue is empty. Does NOT pop - see popTransmittedResult.
  size_t updateTransmitMessage(void);

  // Drop the result updateTransmitMessage just serialised
  void popTransmittedResult(void);

#if DEBUG_WAVE_MSG
  // Bench only (see DEBUG_WAVE_MSG in wave_config.h): push a synthetic result onto
  // the SAME deque processReading feeds, so everything downstream runs unmodified.
  void enqueueFakeResult(void);

  // Dump the result that is about to be transmitted. Takes the stream as an argument
  // rather than using Serial, because on the drifter Serial is never begun - the
  // console is main.cpp's mySerial. Does NOT pop: call it before updateTransmitMessage.
  void printPendingResult(Print &out) const;
#endif

  byte msgB[wave_message_size];

 private:
  static WaveManager *s_self;
  static void rowSinkTrampoline(const ImuRow &r);
  void onRow(const ImuRow &r);
  // bool: false means the block did not reach the sd-card in full, which the sampler
  // turns into kRawFlagWriteFail on the next sync record. See onRawBlock.
  static bool rawSinkTrampoline(const uint8_t *data, uint16_t len);
  bool onRawBlock(const uint8_t *data, uint16_t len);
  void writeRawHeader(void);        // kRawHeaderBytes: magic, rates, sensitivities

  // Session logging (ORB_test Logger style): one timestamped directory per capture.
  bool startSession(void);          // build stamp, mkdir _tmp, open imu/gps/ses, headers
  void stopSession(void);           // close ses, rename _tmp -> final (capture completed)
  void writeSessionAnchor(void);    // reading ID + start UTC (crash-safe, written up front)
  void writeSessionConfig(File &f); // cfg.csv: every constant the capture depends on
  void writeSessionSummary(bool ok, const WaveParams &params);  // stop UTC, duration, rows, params
  void serviceGps(uint32_t relMs);  // drive gps_manager.update(); log a gps.csv row per fix
  bool waitForGpsFix(void);         // block up to wave_gps_fix_timeout for a valid PVT

  ImuSampler imu_;
  StreamAnalyzer analyzer_;

  // CSV logging state. imu.csv is streamed row-by-row via the row sink; gps/ses are
  // held open across the session. sessionDir_ is the "<stamp>_tmp" path so
  // processReading can drop spec/ana into the same folder before the rename.
  File     imuFile_;
  File     gpsFile_;
  File     sessionFile_;
  File     rawFile_;               // <stamp>_raw.bin, only when wave_raw_log
  bool     csvActive_ = false;     // session opened (gps/ses/spec/ana), all log modes
  bool     imuCsvActive_ = false;  // imu.csv specifically - false in WaveLogMode::Raw
  uint16_t rowsSinceSync_ = 0;
  char     logStamp_[16]   = "00000000_000000";  // YYYYMMDD_HHMMSS
  char     sessionDir_[40] = "";                  // waves/<stamp>_tmp
  uint32_t gpsRowsWritten_ = 0;
  // Whether the capture started with a valid fix. Logged to ses.csv: with
  // wave_measurement_require_gps false a capture can legitimately run without one,
  // and this is what separates that from a receiver that failed - both leave gps.csv
  // holding nothing but its header line.
  bool     gpsFixAtStart_ = false;

  uint16_t readingID_ = 0;
  uint32_t rowCount_ = 0;
  time_t   captureStart_ = 0;
  time_t   captureEnd_ = 0;
  bool     imuOk_ = false;
};

extern WaveManager wave_manager;

#endif  // WAVE_MANAGER_H
