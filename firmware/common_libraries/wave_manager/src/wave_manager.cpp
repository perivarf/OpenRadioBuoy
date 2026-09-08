#include "wave_manager.h"

#include <TimeLib.h>
#include "IWatchdog.h"
#include "sd_writer.h"

/*
  Manager lifecycle and the capture loop. WaveManager is split across three files, all
  of them members of this one class:

    wave_manager.cpp      this file - begin/wake/sleep, the GPS fix wait, the row and
                          raw sinks, takeReading and processReading
    wave_session_log.cpp  everything written to the sd-card: the session directory and
                          its six CSV files, plus reading-ID continuity
    wave_message.cpp      serialising a result for the radio, and the bench fixture
*/

WaveManager wave_manager;
WaveManager *WaveManager::s_self = nullptr;

// Begin capture mode, initialize the IMU. Called once at boot and again from wake() before each capture
void WaveManager::begin(void) {
  s_self = this;
  seedReadingId();
  imu_.setRowSink(&WaveManager::rowSinkTrampoline);

  imuOk_ = imu_.begin(Serial);
  if (imuOk_) {
    imuOk_ = imu_.checkImu(Serial);
    if (!imuOk_ && debug_serial) {
      Serial.println("WaveManager: IMU check failed at boot - will retry each capture");
    }
  } else if (debug_serial) {
    Serial.println("WaveManager: IMU init failed at boot - will retry each capture");
  }
}

void WaveManager::wake(void) {

  // Restart the GNSS engine
  gps_manager.begin();

  // Restart the IMU and reset its FIFO buffer
  imuOk_ = imu_.begin(Serial);
  if (imuOk_) {
    imu_.resetFifo();
  } else if (debug_serial) {
    Serial.println("WaveManager: IMU did not answer - skipping this capture");
  }
}

// Put the system to sleep, shutting down the IMU and GNSS.
void WaveManager::sleep(void) {
  if (imuOk_) imu_.shutdownIMU();
  gps_manager.shutdownGPS();
}

// Wait for the GNSS receiver to produce a fresh and valid fix
// PVT (position, velocity, time) solution survives across captures, 
// so lastFix() alone would pass instantly on a stale fix
// from the previous measurement.
bool WaveManager::waitForGpsFix(void) {
  
  // No receiver in this build - there is nothing to wait for and nothing to report.
  if (!enable_GPS) return false;

  if (!gps_manager.ready()) {
    if (debug_serial) Serial.println("WaveManager: GPS did not init - no fix this capture");
    return false;
  }

  const uint32_t start = millis();
  while (millis() - start < wave_gps_fix_timeout) {
    gps_manager.update();
    if (gps_manager.freshFix() && gps_manager.lastFix().valid) {
      if (debug_serial) {
        Serial.print("WaveManager: GPS fix after ");
        Serial.print(millis() - start);
        Serial.print(" ms, sats ");
        Serial.println(gps_manager.lastFix().numSV);
      }
      return true;
    }
    IWatchdog.reload();
    delay(10);  // do not spin the CPU at 100% while waiting for a fix
  }

  if (debug_serial) {
    Serial.print("WaveManager: no GPS fix in ");
    Serial.print(wave_gps_fix_timeout);
    Serial.println(" ms");
  }
  return false;
}

// The latest PVT solution in the receiver's own 1e-7 deg, or 0,0 if there is none 
WaveManager::FixE7 WaveManager::currentFixE7(void) const {
  FixE7 p;
  if (enable_GPS && gps_manager.lastFix().valid) {
    p.lat = gps_manager.lastFix().lat_e7;
    p.lng = gps_manager.lastFix().lng_e7;
  }
  IWatchdog.reload();
  return p;
}

// -------------------------------------------------------------------------------
// Row sink: analyse the window, then (optionally) append it to imu.csv / raw.bin
// -------------------------------------------------------------------------------
void WaveManager::rowSinkTrampoline(const ImuRow &r) {
  if (s_self) s_self->onRow(r);
}

bool WaveManager::rawSinkTrampoline(const uint8_t *data, uint16_t len) {
  return s_self ? s_self->onRawBlock(data, len) : false;
}

// Write data to raw-file
// Returns true if the write succeded, false if the write failed (disk full, etc)
bool WaveManager::onRawBlock(const uint8_t *data, uint16_t len) {
  if (!rawFile_) return false;
  return rawFile_.write(data, len) == (int)len;
}

// Runs inside the FIFO pop loop (from ImuSampler::closeWindow)
// Push the row into the analyzer, then (optionally) append it to imu.csv
void WaveManager::onRow(const ImuRow &r) {
  analyzer_.ingest(r);  // the row arrives complete; the analyzer only consumes it
  rowCount_++;

  if (!imuFile_) return;   // Raw-only mode: the analyzer above still ran
  appendImuCsvRow(r);           // see wave_session_log.cpp

  // Flag if we should sync the file after this row
  // Sync happens later outside the FIFO pop loop when the FIFO is empty.
  if (++rowsSinceSync_ >= wave_csv_sync_rows) {
    imuSyncPending_ = true;
    rowsSinceSync_ = 0;
  }
}

void WaveManager::syncImuCsvIfPending(void) {
  if (!imuSyncPending_) return;
  imuSyncPending_ = false;
  if (imuFile_) imuFile_.sync();
}


// -----------------------------------------------------------------------------
// Capture: stream the IMU FIFO for wave_measurement_duration.
// -----------------------------------------------------------------------------
uint8_t WaveManager::takeReading(void) {

  if (uint8_t err = checkPreconditions()) return err;

  beginCapture();
  runCaptureLoop();
  closeSessionFiles();

  // End capture timestamp, for ses.csv and WaveResult
  captureEnd_ = now();

  resolveEndPosition();
  return 0;
}

// 0 if the capture may proceed, else the code takeReading() should return without
// having captured anything.
uint8_t WaveManager::checkPreconditions(void) {
  // No IMU
  if (!imuOk_) return 1;

  // Wait for gps fix to set start location for measurements
  gpsFixAtStart_ = waitForGpsFix();

  // In case we require GPS and no fix, we return
  if (!gpsFixAtStart_ && wave_measurement_require_gps && enable_GPS) {
    if (debug_serial) {
      Serial.println("WaveManager: capture skipped - wave_measurement_require_gps");
    }
    return 2;
  }

  return 0;
}

// Reading ID, per-capture counters, start position, and the analyzer/session start.
void WaveManager::beginCapture(void) {
  // Get reading ID
  seedReadingId();
  readingID_++;

  // Initialize row counts
  rowCount_ = 0;
  gpsRowsWritten_ = 0;

  // Zeroed with the other per-capture counters, so the block in ses.csv describes THIS
  // capture and carries nothing over from the previous one.
  wave_timing.resetCapture();
  captureStart_ = now();
  captureStartPos_ = currentFixE7();
  captureEndPos_ = FixE7{}; // Initialize end position as start, will be overwritten at the end if successful logging
  IWatchdog.reload();

  // Turning off GPS if not needed in capture.
  if constexpr (!wave_gps_track_in_capture) {
    gps_manager.shutdownGPS();
    if (debug_serial) Serial.println("WaveManager: GPS off for the capture (no drift track)");
  }

  // Start the analyzer (produces PSD and wave parameters)
  analyzer_.begin();

  // Logging to the sd-card is optional
  sessionActive_ = false;

  if (wave_log_to_sd && sd_writer.active) {
    const uint32_t tStart = timeStart();
    sessionActive_ = startSession();
    if (wave_timing_enabled) wave_timing.startSessionUs = micros() - tStart;
  }

  IWatchdog.reload();
}

// Resets and starts the IMU FIFO stream, then drains it for wave_measurement_duration.
void WaveManager::runCaptureLoop(void) {
  imu_.resetWindowing(millis());
  imu_.resetFifo();
  imu_.startStreaming();

  uint32_t start = millis();
  while (millis() - start < wave_measurement_duration) {

    const uint32_t elapsed = millis() - start;

    // TIM_LOOP covers the body (except the delay(2))
    const uint32_t tLoop = timeStart();

    // Fetch IMU data, process it, and write it to raw log and/or imu.csv
    // The FIFO is drained in update() until it is empty, and the FIR is evaluated for each window
    imu_.update(Serial, wave_measurement_duration - elapsed, gpsRowsWritten_);

    // Sync IMU-file if pending
    const uint32_t tSync = timeStart();
    syncImuCsvIfPending();
    timeAdd(TIM_SYNCCSV, tSync);

    // The full segment is accumulated in the analyzer by imu_.update() -> onRow() -> analyzer_.ingest(). 
    // The analyzer's processPendingSegment() is called here to finalise the segment and accumulate the PSD sums. 
    // It is a no-op if no segment is pending.
    const uint32_t tWelch = timeStart();
    if (analyzer_.processPendingSegment()) timeAdd(TIM_WELCH, tWelch);

    // The GNSS tracking, and the only GPS work inside the loop. 
    // Compiled out entirely when wave_gps_track_in_capture is off
    // the positions at each end of the capture come from outside this loop either way.
    // Not guaranteed to produce uniformly spacing, but with 480Hz IMU field tests
    // show that when we set GPS to 10Hz, we get around 9.5 Hz of actual capture 
    // (due to FIFO pop loop / SD-card-operations sometimes taking longer than 100ms)
    if constexpr (wave_gps_track_in_capture) {
      const uint32_t tGps = timeStart();
      serviceGps(elapsed);          // non-blocking GPS poll -> one gps.csv row per fix
      timeAdd(TIM_GPS, tGps);
    }

    IWatchdog.reload();
    timeAdd(TIM_LOOP, tLoop);

    // let the FIFO refill; keeps the drain loop from spinning hot
    // TODO: If not using GNSS, we could let the FIFO fill up more (and subsequently sleep more)
    delay(2);  
  }
}

// Truncate/sync/close imu, gps and raw files 
// No-op unless a session was actually started.
void WaveManager::closeSessionFiles(void) {
  if (!sessionActive_) return;

  // Timed per file
  const uint32_t tStop = timeStart();

  // truncate() at the current position hands back the clusters pre-allocation
  // reserved but the capture did not use, and sets the directory entry to the real
  // length. Without it every session folder would claim its full reservation and
  // the tail would read as garbage. Safe to call whether or not preAllocate
  // succeeded: with no reservation the position already is the end of the file.
  if (imuFile_) {
    const uint32_t t0 = timeStart();
    imuFile_.truncate();  IWatchdog.reload();
    imuFile_.sync();      IWatchdog.reload();
    imuFile_.close();     IWatchdog.reload();
    if (wave_timing_enabled) wave_timing.stopImuUs = micros() - t0;}

  if (gpsFile_) {
    const uint32_t tStopGps = timeStart();
    gpsFile_.truncate();  IWatchdog.reload();
    gpsFile_.sync();      IWatchdog.reload();
    gpsFile_.close();     IWatchdog.reload();
    if (wave_timing_enabled) wave_timing.stopGpsUs = micros() - tStopGps;
  }

  // The raw log's partial block has to be pushed before truncate()
  if (rawFile_) {
    const uint32_t tStopRaw = timeStart();
    rawLog_.flush(true);      IWatchdog.reload();
    imu_.setRawLog(nullptr);  IWatchdog.reload();
    rawLog_.setSink(nullptr); IWatchdog.reload();
    rawFile_.truncate();      IWatchdog.reload();
    rawFile_.sync();          IWatchdog.reload();
    rawFile_.close();         IWatchdog.reload();
    if (wave_timing_enabled) wave_timing.stopRawUs = micros() - tStopRaw;
  }
  if (wave_timing_enabled) wave_timing.stopTotalUs = micros() - tStop;
  // sessionFile_ stays open: the summary is appended in processReading.
}

/* The end position. With the GNSS on during the capture, we already have a position.
   If not, bring it back up, wait the same wave_gps_fix_timeout as at the start, and
   shut it down again */
void WaveManager::resolveEndPosition(void) {
  if constexpr (wave_gps_track_in_capture) {
    captureEndPos_ = currentFixE7();
  } else {
    gps_manager.begin();
    IWatchdog.reload();
    if (waitForGpsFix()) {
      captureEndPos_ = currentFixE7();
    }
    gps_manager.shutdownGPS();
  }
}


// -----------------------------------------------------------------------------
// Finalise the spectrum -> wave parameters, push a result, write spec/ana CSV.
// -----------------------------------------------------------------------------
uint8_t WaveManager::processReading(void) {
  WaveParams params;
  WaveResult res;
  res.reading_ID = readingID_;
  res.timestamp_start = captureStart_;
  res.timestamp_end = captureEnd_;
  res.lat_start_e7 = captureStartPos_.lat;
  res.lng_start_e7 = captureStartPos_.lng;
  res.lat_end_e7   = captureEndPos_.lat;
  res.lng_end_e7   = captureEndPos_.lng;

  bool ok = analyzer_.finalize(params, res.wave_spectrum);

  res.Hs = params.hs;
  res.Tc = params.tc;
  res.Tp = params.tp;
  res.Tz = params.tz;
  res.max_value = params.maxValue;

  // spec.csv + ana.csv into the same session directory as imu/gps/ses, then the
  // session summary. Only when startSession succeeded (sessionActive_). These files are
  // what a reader checks to tell a finished capture from an interrupted one.
  if (sessionActive_ && sd_writer.active) {
    writeSpecCsv();
    writeAnaCsv(ok, params);
    writeSessionSummary();
    stopSession();
  }

  // Summary to the console (mirrors ORB_test StreamAnalyzer::finalize).
  if (debug_serial) {
    Serial.print("[wave] processReading #"); Serial.println(readingID_);
    Serial.print("  brake_windows: ");    Serial.print(analyzer_.brakeRows());
    Serial.print(" / ");                  Serial.println(analyzer_.rows());
    Serial.print("  vacc_welch_samples: "); Serial.println(analyzer_.samplesWelch());
    Serial.print("  welch_segments: ");   Serial.print(analyzer_.segments());
    Serial.print(" (seglen=");            Serial.print((int)kWelchSegLen); Serial.println(")");
    if (ok) {
      Serial.print("  Hs=");  Serial.print(params.hs, 3); Serial.print(" m");
      Serial.print("  Tz=");  Serial.print(params.tz, 2);
      Serial.print(" s  Tc="); Serial.print(params.tc, 2);
      Serial.print(" s  Tp="); Serial.print(params.tp, 2); Serial.println(" s");
    } else {
      Serial.println("  no usable spectrum (too few Welch segments)");
    }
  }

  if (!ok) return 1;

  if (wave_analysis_results.full()) wave_analysis_results.pop_back();
  wave_analysis_results.push_front(res);
  return 0;
}
