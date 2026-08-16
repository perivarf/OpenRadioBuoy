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
  float    max_value;  // peak acceleration PSD value ((m/s^2)^2/Hz), the spectrum's scale
  uint16_t wave_spectrum[welch_bins];  // quantised acceleration PSD, bins welch_bin_min..max
  time_t   timestamp_start;
  time_t   timestamp_end;
  // Position at each end of the capture window, 1e-7 deg straight off UBX_PVT so
  // nothing is rescaled before the message needs it. 0 means no valid fix was held
  // then; the 'W' message forwards that as 0,0, the agreed "unknown" (readings.h).
  int32_t  lat_start_e7;
  int32_t  lng_start_e7;
  int32_t  lat_end_e7;
  int32_t  lng_end_e7;
};

/*
  Owns the IMU sampler and the streaming wave analyzer, and follows the same manager
  contract as thermo_manager / gps_manager
  (begin/wake/sleep/takeReading/processReading/updateTransmitMessage + msgB).

  Implemented across three files: wave_manager.cpp (lifecycle and the capture loop),
  wave_session_log.cpp (the sd-card files), wave_message.cpp (radio serialisation).
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

  // Serialise the front result's PARAMETERS into msgB ('W' ... 'E') and return the
  // length, or 0 if the queue is empty. Does NOT pop - see popTransmittedResult.
  size_t updateTransmitMessage(void);

  // Serialise the front result's SPECTRUM into psdB ('P' ... 'E') and return the
  // length. 0 when the queue is empty or kSendPsd is off, which the caller reads as
  // "nothing to send" - a build that does not transmit spectra stops sending the second
  // message rather than sending an empty one. Both messages carry the same ts_start,
  // which is what pairs them up; see the join-key note in readings.h.
  size_t updatePsdTransmitMessage(void);

  // Drop the result the two updateTransmitMessage calls just serialised
  void popTransmittedResult(void);

#if DEBUG_WAVE_MSG
  // Bench only (see DEBUG_WAVE_MSG in wave_config.h): push a synthetic result onto
  // the SAME deque processReading feeds, so everything downstream runs unmodified.
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
  void syncImuCsvIfPending(void);   // the imu.csv sync, deferred out of the pop loop
  // bool: false means the block did not reach the sd-card in full, which RawLogWriter
  // turns into kRawFlagWriteFail on the next sync record. See onRawBlock.
  static bool rawSinkTrampoline(const uint8_t *data, uint16_t len);
  bool onRawBlock(const uint8_t *data, uint16_t len);

  // Session logging (ORB_test Logger style): one timestamped directory per capture.
  // All of it lives in wave_session_log.cpp - this file keeps the capture loop.
  bool startSession(void);          // build stamp, mkdir, open imu/gps/ses, headers
  void stopSession(void);           // close ses (the summary in it marks the capture done)
  void writeSessionAnchor(void);    // reading ID + start UTC (crash-safe, written up front)
  void writeSessionConfig(File &f); // cfg.csv: every constant the capture depends on
  void writeSessionSummary(bool ok, const WaveParams &params);  // stop UTC, duration, rows, params
  void writeTimingBlock(void);      // wave_timing buckets; empty unless wave_timing_enabled
  void appendImuCsvRow(const ImuRow &r);        // one imu.csv line; called from onRow
  void writeSpecCsv(void);                      // spec.csv: the PSD, bin by bin
  void writeAnaCsv(const WaveParams &params);   // ana.csv: counters + wave parameters
  void serviceGps(uint32_t relMs);  // poll the receiver, log a gps.csv row per fresh fix.
                                    // Only called when wave_gps_track_in_capture is set
  bool waitForGpsFix(void);         // block up to wave_gps_fix_timeout for a FRESH valid
                                    // PVT; used at both ends of the capture

  // Reading ID continuity across resets. See seedReadingId in the .cpp.
  uint16_t countSessionDirs(void);  // session folders already under waves/
  void     seedReadingId(void);     // one-shot: readingID_ = countSessionDirs()

  ImuSampler imu_;
  StreamAnalyzer analyzer_;

  // The raw log's byte format. Owned here rather than by the sampler because this is
  // what opens, headers and closes rawFile_; the sampler is only handed a pointer for
  // the duration of the capture and emits into it from inside the drain.
  RawLogWriter rawLog_;

  // CSV logging state. imu.csv is streamed row-by-row via the row sink; gps/ses are
  // held open across the session. sessionDir_ is the "<stamp>" path so
  // processReading can drop spec/ana into the same folder.
  File     imuFile_;
  File     gpsFile_;
  File     sessionFile_;
  File     rawFile_;               // <stamp>_raw.bin, only when wave_mode_imu_raw()
  // Session directory opened: ses/spec/ana in every log mode, and raw.bin with them -
  // hence not "csv". The per-file flags below say which of the OPTIONAL streams made it;
  // this one says whether there is a session at all.
  bool     sessionActive_ = false;
  uint16_t rowsSinceSync_ = 0;
  // Set by onRow when the sync cadence is due, acted on between drains. sync() is the
  // one call in the csv path that is not a plain block write - it seeks off to the FAT
  // and the directory entry and back, which is the kind of card operation that goes
  // busy for hundreds of ms - and onRow runs inside the FIFO pop loop. See
  // syncImuCsvIfPending.
  bool     imuSyncPending_ = false;
  char     logStamp_[16]   = "00000000_000000";  // YYYYMMDD_HHMMSS
  char     sessionDir_[40] = "";                  // waves/<stamp>
  uint32_t gpsRowsWritten_ = 0;
  // Whether the capture started with a valid fix, logged to ses.csv. With
  // wave_measurement_require_gps false a capture can legitimately run without one, and
  // this is what separates that from a receiver that failed.
  bool     gpsFixAtStart_ = false;

  uint16_t readingID_ = 0;
  bool     readingIdSeeded_ = false;  // waves/ counted once; see seedReadingId
  uint32_t rowCount_ = 0;
  time_t   captureStart_ = 0;
  time_t   captureEnd_ = 0;

  /*
    Where the buoy was at each end of the window. Sampled next to the lines that set
    captureStart_/captureEnd_, so each position belongs to its own end of the capture -
    reading the fix later would give whatever the receiver had by then, which after a
    30-minute drift is a different place.

    The END pair is not quite simultaneous when wave_gps_track_in_capture is off: the
    timestamp is taken when the loop exits, then the receiver is re-polled, so the
    position is up to one fix later (~100-200 ms with a lock). The timestamp is the one
    that has to stay honest, since ses.csv and the 'W' message both carry it.
  */
  struct FixE7 { int32_t lat = 0; int32_t lng = 0; };
  FixE7 captureStartPos_;
  FixE7 captureEndPos_;

  // The current solution as 1e-7 deg, or 0,0 when there is no valid fix to give.
  // Zero is the agreed "unknown" here; see readings.h.
  FixE7 currentFixE7(void) const;
  bool     imuOk_ = false;
};

extern WaveManager wave_manager;

#endif  // WAVE_MANAGER_H
