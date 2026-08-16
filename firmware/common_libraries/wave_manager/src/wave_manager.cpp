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

  imuOk_ = imu_.begin(Serial);
  if (imuOk_) {
    imu_.resetFifo();
  } else if (debug_serial) {
    Serial.println("WaveManager: IMU did not answer - skipping this capture");
  }
}

void WaveManager::sleep(void) {
  if (imuOk_) imu_.shutdownIMU();
  gps_manager.shutdownGPS();
}

// Wait for the receiver to produce a valid solution, and report whether it did.
// Reports ONLY that - whether a missing fix ends the capture is
// wave_measurement_require_gps, and that decision belongs to takeReading. This
// function is also what fills gps_fix_at_start in ses.csv, so it must answer the
// factual question even in the builds that carry on without one.
//
// Called before any file is opened, so an abort leaves no session directory, no
// reading ID and no ~28 MB reservation behind.
//
// "Valid" is a FRESH gnssFixOK solution (UBX_PVT::valid, i.e. fixType >= 2): pvt
// survives across captures, so lastFix() alone would pass instantly on a stale fix
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
    delay(10);  // the poll is rate-limited to GPS_nav_period_ms; do not spin
  }

  if (debug_serial) {
    Serial.print("WaveManager: no GPS fix in ");
    Serial.print(wave_gps_fix_timeout);
    Serial.println(" ms");
  }
  return false;
}

// The current solution in the receiver's own 1e-7 deg, or 0,0 if there is none to
// give. Not freshFix(): that flag is about whether a NEW solution arrived since the
// last poll, and here the question is only "where are we", for which the last valid
// solution is the right answer. valid is still required - an invalid pvt holds
// whatever was last decoded, which may be a fix from the previous deployment.
WaveManager::FixE7 WaveManager::currentFixE7(void) const {
  FixE7 p;
  if (enable_GPS && gps_manager.lastFix().valid) {
    p.lat = gps_manager.lastFix().lat_e7;
    p.lng = gps_manager.lastFix().lng_e7;
  }
  IWatchdog.reload();
  return p;
}

// -----------------------------------------------------------------------------
// Row sink: analyse the window, then (optionally) append it to imu.csv.
// -----------------------------------------------------------------------------
void WaveManager::rowSinkTrampoline(const ImuRow &r) {
  if (s_self) s_self->onRow(r);
}

bool WaveManager::rawSinkTrampoline(const uint8_t *data, uint16_t len) {
  return s_self ? s_self->onRawBlock(data, len) : false;
}

// One drain's worth of the raw log. No sync() here on purpose: SdFat's own buffering
// plus the sync in stopSession is enough - a capture cut by a reset loses the tail of
// the raw file, which is the same bargain imu.csv makes.
//
// This does not run inside the FIFO pop loop: RawLogWriter buffers a whole drain and
// calls the sink once the FIFO is empty (kRawBufBytes in wave_config.h). The write can
// still stall the card - it just no longer does so with words waiting.
//
// The return value is not decoration. write() reports a SHORT write (sd-card full, I/O
// error) by returning fewer bytes, and discarding that was the difference between a
// file that is missing a block and a file that LOOKS fine while every byte after the
// gap is misparsed. RawLogWriter turns a false into kRawFlagWriteFail on the next sync.
bool WaveManager::onRawBlock(const uint8_t *data, uint16_t len) {
  if (!rawFile_) return false;
  return rawFile_.write(data, len) == (int)len;
}

// Runs inside the FIFO pop loop (from ImuSampler::closeWindow), so everything here is
// on the drain's clock.
void WaveManager::onRow(const ImuRow &r) {
  analyzer_.ingest(r);  // the row arrives complete; the analyzer only consumes it
  rowCount_++;

  if (!imuCsvActive_) return;   // Raw-only mode: the analyzer above still ran
  appendImuCsvRow(r);           // see wave_session_log.cpp

  // Only a flag here: sync() is the one call in the csv path that is not a plain block
  // write, and it is deferred to syncImuCsvIfPending() below. The prints stay - they
  // land in SdFat's 512-byte cache and cost an ordinary single-block write, which is
  // not the stall worth moving.
  if (++rowsSinceSync_ >= wave_csv_sync_rows) {
    imuSyncPending_ = true;
    rowsSinceSync_ = 0;
  }
}

// The deferred half of the cadence above. Called from the capture loop right after
// imu_.update() returns, i.e. with the FIFO just drained - the same reasoning that
// moved the raw-log write out of the pop loop (kRawBufBytes in wave_config.h).
//
// A stall here can still overrun the FIFO; what it cannot do any more is start with
// half the budget already spent. FIFO_OVR is latched, so the loss is picked up by the
// next drain's status read either way and nothing goes unreported.
void WaveManager::syncImuCsvIfPending(void) {
  if (!imuSyncPending_) return;
  imuSyncPending_ = false;
  if (imuCsvActive_) imuFile_.sync();
}


// -----------------------------------------------------------------------------
// Capture: stream the IMU FIFO for wave_measurement_duration.
// -----------------------------------------------------------------------------
uint8_t WaveManager::takeReading(void) {
  if (!imuOk_) return 1;

  // Deliberately the FIRST thing after the IMU check: an abort must not consume a
  // reading ID, create a session folder or reserve the clusters, so a run that never
  // sees the sky leaves the sd-card exactly as it was. The wait runs whatever
  // wave_measurement_require_gps says - the answer is logged as gps_fix_at_start
  // either way, and only the ABORT is conditional. enable_GPS wins over the
  // requirement: demanding a fix from a build with no receiver would abort forever.
  gpsFixAtStart_ = waitForGpsFix();
  if (!gpsFixAtStart_ && wave_measurement_require_gps && enable_GPS) {
    if (debug_serial) {
      Serial.println("WaveManager: capture skipped - wave_measurement_require_gps");
    }

    return 2;
  }

  seedReadingId();  // no-op once done; covers a card that was not up at begin()
  readingID_++;
  rowCount_ = 0;
  gpsRowsWritten_ = 0;
  // Zeroed with the other per-capture counters, so the block in ses.csv describes THIS
  // capture and carries nothing over from the previous one.
  wave_timing.resetCapture();
  captureStart_ = now();
  captureStartPos_ = currentFixE7();
  // The end position is only assigned if a fix is actually held then, so it has to be
  // cleared here - otherwise a capture that ends without one would report the PREVIOUS
  // capture's position as its own.
  captureEndPos_ = FixE7{};
  IWatchdog.reload();

  analyzer_.begin();

  // Open the session directory BEFORE starting the FIFO stream: the mkdir, the file
  // opens and the headers/anchor/config take tens of ms of SD activity, and a FIFO
  // already streaming would overflow before the first drain. csvActive_ spans
  // take+process: spec/ana are added and the session file closed in processReading ->
  // stopSession.
  csvActive_ = false;
  // startSession sets these; a failed open must not leave them set.
  imuCsvActive_ = false;
  gpsCsvActive_ = false;
  if (wave_log_csv && sd_writer.active) {
    // Not a threat to the FIFO - the stream starts below - but it IS dead time inside the
    // measurement window, and preAllocate of a reservation this size is the one call here
    // that can run into hundreds of ms. Recorded as a single value: it happens once.
    const uint32_t tStart = timeStart();
    csvActive_ = startSession();
    if (wave_timing_enabled) wave_timing.startSessionUs = micros() - tStart;
  }

  IWatchdog.reload();

  // Init done. Flush whatever the FIFO holds, THEN start the stream: from here on the
  // loop below is what keeps it drained, and it is the only thing that does. The two
  // calls belong together and in this order - resetFifo leaves the FIFO idle, so
  // starting the stream anywhere but immediately before the drain loop reopens the
  // window this split was made to close.
  imu_.resetWindowing(millis());
  imu_.resetFifo();
  imu_.startStreaming();

  uint32_t start = millis();
  while (millis() - start < wave_measurement_duration) {
    const uint32_t elapsed = millis() - start;
    // TIM_LOOP covers the body but NOT the delay below, so it measures work rather than
    // the pause. It is the bucket that catches what the others do not: a drain that
    // arrives late with the INT1 flag already set spent that time somewhere in here, and
    // if TIM_LOOP's max exceeds the sum of the buckets inside it, the time went to
    // something none of them measure.
    const uint32_t tLoop = timeStart();
    // The countdown rides along on the IMU report rather than printing on its own:
    // the two answer one question together - whether the capture is still running,
    // and whether the drain is keeping up while it does.
    imu_.update(Serial, wave_measurement_duration - elapsed);  // drain the FIFO (stay tight)
    const uint32_t tSync = timeStart();
    syncImuCsvIfPending();          // deferred from onRow: never with the FIFO half full
    timeAdd(TIM_SYNCCSV, tSync);
    // Third of the three deferrals, and the largest: ~88 ms of FFT once every 25.6 s.
    // Same placement argument as the two above and as the raw-log flush - update() has
    // just returned, so the FIFO is empty and the whole depth is available. A no-op on
    // the ~99.9 % of iterations with nothing pending.
    //
    // Timed only when it RAN: charging every flag test to the bucket would report
    // thousands of 2 us calls instead of the one 88 ms segment it exists to show. n is
    // the segment count, and it must equal welch_segments in ana.csv.
    const uint32_t tWelch = timeStart();
    if (analyzer_.processPendingSegment()) timeAdd(TIM_WELCH, tWelch);
    // The drift track, and the only GPS work inside the loop. Compiled out entirely
    // when wave_gps_track_in_capture is off, which leaves tim_gps at n = 0; the
    // positions at each end of the capture come from outside this loop either way.
    if constexpr (wave_gps_track_in_capture) {
      const uint32_t tGps = timeStart();
      serviceGps(elapsed);          // non-blocking GPS poll -> one gps.csv row per fix
      timeAdd(TIM_GPS, tGps);
    }
    IWatchdog.reload();
    timeAdd(TIM_LOOP, tLoop);
    delay(2);  // let the FIFO refill; keeps the drain loop from spinning hot
  }

  if (csvActive_) {
    // Timed per FILE, not as one total: handing back a reservation is the expensive part,
    // and the three files hold wildly different ones (the raw log reserves tens of MB, the
    // gps log a couple). A single number would say that shutdown was slow without saying
    // which file made it so. Same reason these are plain values and not a TimeStat: one
    // sample each, and a max across all three would answer the wrong question.
    const uint32_t tStop = timeStart();

    // truncate() at the current position hands back the clusters pre-allocation
    // reserved but the capture did not use, and sets the directory entry to the real
    // length. Without it every session folder would claim its full reservation and
    // the tail would read as garbage. Safe to call whether or not preAllocate
    // succeeded: with no reservation the position already is the end of the file.
    if (imuCsvActive_) {
      const uint32_t t0 = timeStart();
      imuFile_.truncate();  IWatchdog.reload();
      imuFile_.sync();      IWatchdog.reload();
      imuFile_.close();     IWatchdog.reload();
      if (wave_timing_enabled) wave_timing.stopImuUs = micros() - t0;}

    if (gpsCsvActive_) {
      const uint32_t tStopGps = timeStart();
      gpsFile_.truncate();  IWatchdog.reload();
      gpsFile_.sync();      IWatchdog.reload();
      gpsFile_.close();     IWatchdog.reload();
      if (wave_timing_enabled) wave_timing.stopGpsUs = micros() - tStopGps;
    }

    // The raw log's partial block has to be pushed BEFORE truncate(), or the tail is
    // cut at the last full block and the final records are lost. Detaching the sink
    // first stops a late drain from appending past the truncation point.
    if (rawFile_) {
      // Flush BEFORE detaching, not after: flush() writes through the sink and does
      // nothing without one, so the other order would silently discard the last
      // partial buffer - up to kRawBufBytes, a drain's worth of data plus the sync
      // record describing it. No drain can slip in between the two lines: the INT1
      // routine only sets a flag, it drains nothing.
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

  // Taken here, before the fix below: this is when the capture actually ended, and it is
  // what ses.csv and the 'W' message carry.
  captureEnd_ = now();

  /*
    The end position. With the loop polling, lastFix() is already fresh and this is a
    read. Without it, lastFix() still holds the solution decoded BEFORE the capture -
    and returning that would report a 30-minute drift of zero, which looks like a
    measurement rather than a missing one. So re-poll.

    waitForGpsFix is the right primitive rather than a bare gps_manager.update(): it
    requires freshFix() && valid, so it cannot pass on the stale start fix, and it keeps
    polling until one decodes - which matters because the receiver's DDC buffer has been
    left unread for the whole capture. On failure the position stays 0,0, i.e. unknown.

    No FIFO is at risk here: the drain loop has finished and nothing is left to lose.
  */
  if (wave_gps_track_in_capture || waitForGpsFix()) {
    captureEndPos_ = currentFixE7();
  }
  return 0;
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
  // session summary. Only when startSession succeeded (csvActive_). These files are
  // what a reader checks to tell a finished capture from an interrupted one.
  if (csvActive_ && sd_writer.active) {
    writeSpecCsv();
    writeAnaCsv(params);
    writeSessionSummary(ok, params);   // closes out the session file (anchor + summary)
    stopSession();
  }

  // Summary to the console (mirrors ORB_test StreamAnalyzer::finalize).
  if (debug_serial) {
    Serial.print("[wave] processReading #"); Serial.println(readingID_);
    Serial.print("  brake_windows: ");    Serial.print(analyzer_.brakeRows());
    Serial.print(" / ");                  Serial.println(analyzer_.rows());
    Serial.print("  vacc10Hz_samples: "); Serial.println(analyzer_.samples10Hz());
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
