#ifndef WAVE_MANAGER_H
#define WAVE_MANAGER_H

#include <Arduino.h>
#include <SdFat.h>
#include "config.h"
#include "etl/deque.h"
#include "readings.h"
#include "imu_sampler.h"
#include "wave_analysis.h"
#include "wave_config.h"
#include "gps_manager.h"

/*
  Internal wave result. .
*/
struct WaveResult {
  uint16_t reading_ID;
  float    Hs;         // significant wave height (m)
  float    Tc;         // crest period (s)
  float    Tp;         // peak period (s)
  float    Tz;         // zero-crossing period (s)
  float    max_value;  // peak acceleration PSD value ((m/s^2)^2/Hz), the spectrum's scale
  uint16_t wave_spectrum[welch_bins];  // quantised acceleration PSD, bins welch_bin_min..max
  time_t   timestamp_start;
  time_t   timestamp_end;
  // Position at each end of the capture window
  // 1e-7 deg straight off UBX_PVT (i.e. nothing is scaled)
  // 0,0 means no valid fix was held
  int32_t  lat_start_e7;
  int32_t  lng_start_e7;
  int32_t  lat_end_e7;
  int32_t  lng_end_e7;
};

/*
  Owns the IMU sampler and the streaming wave analyzer, and follows the same manager
  contract as thermo_manager / gps_manager
  (begin/wake/sleep/takeReading/processReading/updateTransmitMessage + msgB).

  Implemented across three files: 
  * wave_manager.cpp (lifecycle and the capture loop),
  * wave_session_log.cpp (the sd-card files),
  * wave_message.cpp (radio serialisation).
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
  //       is set - capture deliberately skipped. 
  uint8_t takeReading(void);

  // Finalise the Welch spectrum -> wave params, push a WaveResult, write spec/ana.
  // Returns 0 on success, non-zero if no usable spectrum was produced.
  uint8_t processReading(void);

  // Serialise the (front of queue) parameters into msgB ('W' ... 'E') and return the
  // length, or 0 if the queue is empty. Does not pop - see popTransmittedResult.
  size_t updateTransmitMessage(void);

  // Serialise the front result's SPECTRUM into psdB ('P' ... 'E') and return the
  // length. 0 when the queue is empty or kSendPsd is off
  size_t updatePsdTransmitMessage(void);

  // Drop the result the (two) updateTransmitMessage calls just serialised
  void popTransmittedResult(void);

#if DEBUG_WAVE_MSG
  // Test: push a synthetic result onto the same deque processReading uses, so everything downstream runs unmodified.
  void enqueueFakeResult(void);

  // Dump the result about to be transmitted. Takes the stream as an argument because
  // on the drifter Serial is never begun - the console is main.cpp's mySerial. Does NOT
  // pop: call it before updateTransmitMessage.
  void printPendingResult(Print &out) const;
#endif

  byte msgB[wave_message_size];
  byte psdB[wave_spectrum_message_size];

 private:
  static WaveManager *s_self;
  static void rowSinkTrampoline(const ImuRow &r);
  void onRow(const ImuRow &r);
  void syncImuCsvIfPending(void);   // the imu.csv sync
  static bool rawSinkTrampoline(const uint8_t *data, uint16_t len);
  bool onRawBlock(const uint8_t *data, uint16_t len);

  bool startSession(void);          // build stamp, mkdir, open imu/gps/ses/raw, headers
  void stopSession(void);           // close ses
  void writeSessionAnchor(void);    // reading ID + start UTC/pos
  void writeSessionConfig(File &f); // cfg.csv: every constant the capture depends on
  void writeSessionSummary(void);   // stop UTC/pos, duration, gps rows, timing
  void writeTimingBlock(void);      // wave_timing buckets if  wave_timing_enabled
  void appendImuCsvRow(const ImuRow &r);  // one imu.csv line
  void writeSpecCsv(void);                // spec.csv: the PSD
  void writeAnaCsv(bool ok, const WaveParams &params);  // Wave analysis + misc counters
  void serviceGps(uint32_t relMs);  // when wave_gps_track_in_capture is set: poll the receiver, log a gps.csv row per fresh fix.
  bool waitForGpsFix(void);         // block up to wave_gps_fix_timeout for a fresh PVT
  uint16_t countSessionDirs(void);  // session folders already under waves/
  void     seedReadingId(void);     // readingID_ = countSessionDirs()

  ImuSampler imu_;
  StreamAnalyzer analyzer_;
  RawLogWriter rawLog_;

  // File states
  File     imuFile_;
  File     gpsFile_;
  File     sessionFile_;
  File     rawFile_;

  bool     sessionActive_ = false;
  uint16_t rowsSinceSync_ = 0;
  bool     imuSyncPending_ = false;

  // Log timestamp and session directory
  char     logStamp_[16]   = "00000000_000000";  // YYYYMMDD_HHMMSS
  char     sessionDir_[40] = "";                  // waves/<stamp>
  
  uint32_t gpsRowsWritten_ = 0;
  bool     gpsFixAtStart_ = false;

  uint16_t readingID_ = 0;
  bool     readingIdSeeded_ = false;  // waves/ counted once; see seedReadingId
  uint32_t rowCount_ = 0;
  time_t   captureStart_ = 0;
  time_t   captureEnd_ = 0;

  // Start and stop posistions for the capture.
  struct FixE7 { int32_t lat = 0; int32_t lng = 0; };
  FixE7 captureStartPos_;
  FixE7 captureEndPos_;

  // The current location as 1e-7 deg, or 0,0 when there is no valid fix to give.
  FixE7 currentFixE7(void) const;

  // Write position to session file
  void writePosition(const char *when, const FixE7 &p);
  bool     imuOk_ = false;
};

extern WaveManager wave_manager;

#endif  // WAVE_MANAGER_H
