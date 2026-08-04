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
  // analyzer (and, when logging is enabled, to imu.csv). Returns 0 on success.
  uint8_t takeReading(void);

  // Finalise the Welch spectrum -> wave params, push a WaveResult, write spec/ana.
  // Returns 0 on success, non-zero if no usable spectrum was produced.
  uint8_t processReading(void);

  // Serialise the front result into msgB ('W' ... 'E'), pop it, return the length.
  size_t updateTransmitMessage(void);

  byte msgB[wave_message_size];

 private:
  static WaveManager *s_self;
  static void rowSinkTrampoline(const ImuRow &r);
  void onRow(const ImuRow &r);

  // Session logging (ORB_test Logger style): one timestamped directory per capture.
  bool startSession(void);          // build stamp, mkdir _tmp, open imu/gps/ses, headers
  void stopSession(void);           // close ses, rename _tmp -> final (capture completed)
  void writeSessionAnchor(void);    // build_seq + start UTC (crash-safe, written up front)
  void writeSessionConfig(File &f); // cfg.csv: every constant the capture depends on
  void writeSessionSummary(bool ok, const WaveParams &params);  // stop UTC, duration, rows, params
  void serviceGps(uint32_t relMs);  // drive gps_manager.update(); log a gps.csv row per fix

  ImuSampler imu_;
  StreamAnalyzer analyzer_;

  // CSV logging state. imu.csv is streamed row-by-row via the row sink; gps/ses are
  // held open across the session. sessionDir_ is the "<stamp>_tmp" path so
  // processReading can drop spec/ana into the same folder before the rename.
  File     imuFile_;
  File     gpsFile_;
  File     sessionFile_;
  bool     csvActive_ = false;
  uint16_t rowsSinceSync_ = 0;
  char     logStamp_[16]   = "00000000_000000";  // YYYYMMDD_HHMMSS
  char     sessionDir_[40] = "";                  // waves/<stamp>_tmp
  uint32_t gpsRowsWritten_ = 0;

  uint16_t readingID_ = 0;
  uint32_t rowCount_ = 0;
  time_t   captureStart_ = 0;
  time_t   captureEnd_ = 0;
  bool     imuOk_ = false;
};

extern WaveManager wave_manager;

#endif  // WAVE_MANAGER_H
