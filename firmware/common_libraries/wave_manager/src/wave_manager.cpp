#include "wave_manager.h"

#include <math.h>
#include <string.h>
#include <TimeLib.h>
#include "IWatchdog.h"
#include "parser_utils.h"
#include "sd_writer.h"

WaveManager wave_manager;
WaveManager *WaveManager::s_self = nullptr;

// imu.csv column contract. Naming convention, so a capture can be read without
// consulting the firmware to find out which orientation a column came from:
//   unsuffixed  = the SELECTED filter (WaveAhrs, or SFLP when wave_use_sflp)
//   _sflp       = the on-chip SFLP rotation fusion, or ZERO throughout when
//                 kEnableSflp is false and the block never ran - cfg.csv's
//                 sflp_enabled is what separates that from a still buoy
//   _fir        = FIR-decimated; without it, the unfiltered value at the same instant
static const char *kImuCsvHeader =
    "win_start_ms,n,ax_mg,ay_mg,az_mg,ax_ned_sflp,ay_ned_sflp,az_ned_sflp,"
    "gx_mdps,gy_mdps,gz_mdps,qw_sflp,qx_sflp,qy_sflp,qz_sflp,braking,"
    "qw,qx,qy,qz,vacc,vacc_sflp,sflp_nan,fifo_ovf,vacc_fir,vacc_sflp_fir";

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
// Since 2026-08-14 this no longer runs inside the FIFO pop loop; the sampler buffers a
// whole drain and calls the sink once the FIFO is empty (kRawBufBytes in wave_config.h).
// The write can still stall the card - it just no longer does so with words waiting.
//
// The return value is not decoration. write() reports a SHORT write (sd-card full, I/O
// error) by returning fewer bytes, and discarding that was the difference between a
// file that is missing a block and a file that LOOKS fine while every byte after the
// gap is misparsed. The sampler turns a false into kRawFlagWriteFail on the next sync.
bool WaveManager::onRawBlock(const uint8_t *data, uint16_t len) {
  if (!rawFile_) return false;
  return rawFile_.write(data, len) == (int)len;
}

// Self-describing header, so a capture can be decoded without the firmware that wrote
// it - including the sensitivities, which is what turns the int16 payloads back into
// mg and mdps. kRawHeaderBytes is fixed; the tail is reserved and zeroed.
void WaveManager::writeRawHeader(void) {
  uint8_t h[kRawHeaderBytes] = {0};
  uint8_t o = 0;
  auto put32 = [&](uint32_t v) {
    h[o++] = (uint8_t)v;         h[o++] = (uint8_t)(v >> 8);
    h[o++] = (uint8_t)(v >> 16); h[o++] = (uint8_t)(v >> 24);
  };
  auto put16 = [&](uint16_t v) { h[o++] = (uint8_t)v; h[o++] = (uint8_t)(v >> 8); };
  auto putf  = [&](float f) { uint32_t b; memcpy(&b, &f, 4); put32(b); };

  put32(kRawMagic);
  h[o++] = kRawFormatVersion;
  h[o++] = kRawWordBytes;
  put16(kImuOdrHz);
  put16((uint16_t)kSflpOdrHz);
  putf(kAccSensMgPerLsb);          // int16 LSB -> mg
  putf(kGyrSensMdpsPerLsb);        // int16 LSB -> mdps
  put32((uint32_t)captureStart_);  // capture t=0 in UTC epoch seconds
  put16(readingID_);
  put16(kRawSyncBytes);
  // Remaining bytes stay zero: reserved, and a reader must skip to kRawHeaderBytes
  // rather than assume the fields end where this version stopped writing.
  rawFile_.write(h, kRawHeaderBytes);
}

void WaveManager::onRow(const ImuRow &r) {
  analyzer_.ingest(r);  // the row arrives complete; the analyzer only consumes it
  rowCount_++;

  if (!imuCsvActive_) return;   // Raw-only mode: the analyzer above still ran
  imuFile_.print(r.winStartMs); imuFile_.print(',');
  imuFile_.print(r.n);          imuFile_.print(',');
  imuFile_.print(r.ax, 3); imuFile_.print(','); imuFile_.print(r.ay, 3); imuFile_.print(','); imuFile_.print(r.az, 3); imuFile_.print(',');
  imuFile_.print(r.axnSflp, 3); imuFile_.print(','); imuFile_.print(r.aynSflp, 3); imuFile_.print(','); imuFile_.print(r.aznSflp, 3); imuFile_.print(',');
  imuFile_.print(r.gx, 3); imuFile_.print(','); imuFile_.print(r.gy, 3); imuFile_.print(','); imuFile_.print(r.gz, 3); imuFile_.print(',');
  imuFile_.print(r.qwSflp, 5); imuFile_.print(','); imuFile_.print(r.qxSflp, 5); imuFile_.print(','); imuFile_.print(r.qySflp, 5); imuFile_.print(','); imuFile_.print(r.qzSflp, 5); imuFile_.print(',');
  imuFile_.print(r.braking); imuFile_.print(',');
  imuFile_.print(r.qw, 5); imuFile_.print(','); imuFile_.print(r.qx, 5); imuFile_.print(','); imuFile_.print(r.qy, 5); imuFile_.print(','); imuFile_.print(r.qz, 5); imuFile_.print(',');
  imuFile_.print(r.vacc, 5); imuFile_.print(','); imuFile_.print(r.vaccSflp, 5); imuFile_.print(',');
  imuFile_.print(r.sflpNan); imuFile_.print(',');
  imuFile_.print(r.fifoOvf); imuFile_.print(',');
  imuFile_.print(r.vaccFir, 5); imuFile_.print(','); imuFile_.println(r.vaccSflpFir, 5);

  // Only a flag here. This function is called from closeWindow(), which runs inside
  // the FIFO pop loop - see imu_sampler.cpp - so the sync itself is deferred to
  // syncImuCsvIfPending() below. The prints above stay: they land in SdFat's 512-byte
  // cache and cost an ordinary single-block write, which is not the stall worth moving.
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
// Reading ID continuity.
//
// readingID_ lives in RAM only, so before this it restarted at 0 on every reboot -
// and a watchdog reset is a reboot. Two captures then went out with the same
// reading_ID, which is why readings.h says the join key is (buoy, ts_start).
//
// The fix is the one sd_writer already plays with logCount and readings/: count what
// is on the card at boot and continue from there. The card is the state that survives
// the reset. Each capture owns exactly one folder under waves/, so the folder count IS
// the number of captures taken - subject to the same caveat as logCount, that emptying
// the card restarts the numbering.
// -----------------------------------------------------------------------------
uint16_t WaveManager::countSessionDirs(void) {
  SdFat &card = sd_writer.card();
  if (!card.exists(wave_log_dir)) return 0;   // first capture on a fresh card

  File dir;
  if (!dir.open(wave_log_dir, O_RDONLY)) return 0;

  uint16_t n = 0;
  File entry;
  // Directories only: results.csv sits in this same folder, and counting it would
  // shift every ID by one from the first capture that wrote it onwards.
  while (entry.openNext(&dir, O_RDONLY)) {
    if (entry.isDir() && !entry.isHidden()) n++;
    entry.close();
    IWatchdog.reload();  // a card with hundreds of sessions must not trip the watchdog
  }
  dir.close();
  return n;
}

// One-shot. Called from begin(), and retried at the next capture: begin() runs after
// sd_writer.begin(), but if the card did not come up there the count would silently
// read 0 and restart the numbering - exactly the failure this replaces. Leaving the
// flag clear means the next capture with a working card still picks up where the
// card left off.
void WaveManager::seedReadingId(void) {
  if (readingIdSeeded_ || !sd_writer.active) return;

  readingID_ = countSessionDirs();
  readingIdSeeded_ = true;
  if (debug_serial) {
    Serial.print("WaveManager: "); Serial.print(readingID_);
    Serial.println(" sessions on card - next reading_ID is one past that");
  }
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
  IWatchdog.reload();

  analyzer_.begin();

  // Open the session directory (imu/gps/ses/cfg) BEFORE starting the FIFO stream: the
  // mkdir + opening 4 files + writing headers/anchor/config + syncs take tens of ms of SD
  // activity, and if the FIFO were already streaming it would overflow before the
  // first drain. csvActive_ spans take+process: spec/ana are added and the session
  // file closed in processReading -> stopSession.
  csvActive_ = false;
  imuCsvActive_ = false;   // startSession sets it; a failed open must not leave it set
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
    // Same placement argument as the two above it and as the raw-log flush - update()
    // has just returned, so the FIFO is empty and the whole depth is available. Inside
    // the pop loop, where this used to run, it started with up to kFifoWatermark - 1
    // words already standing. A no-op on the ~99.9% of iterations with nothing pending.
    // Timed only when it RAN. The call itself is a flag test on ~99.9% of iterations,
    // and charging those to the bucket made tim_welch report 6112 calls at 2 us instead
    // of the one 88 ms segment it exists to show. n is the segment count, and it must
    // equal welch_segments in ana.csv.
    const uint32_t tWelch = timeStart();
    if (analyzer_.processPendingSegment()) timeAdd(TIM_WELCH, tWelch);
    const uint32_t tGps = timeStart();
    serviceGps(elapsed);            // non-blocking GPS poll -> one gps.csv row per fix
    timeAdd(TIM_GPS, tGps);
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

    const uint32_t tStopGps = timeStart();
    gpsFile_.truncate();    IWatchdog.reload();
    gpsFile_.sync();        IWatchdog.reload();
    gpsFile_.close();       IWatchdog.reload();
    if (wave_timing_enabled) wave_timing.stopGpsUs = micros() - tStopGps;

    // The raw log's partial block has to be pushed BEFORE truncate(), or the tail is
    // cut at the last full block and the final records are lost. Detaching the sink
    // first stops a late drain from appending past the truncation point.
    if (rawFile_) {
      // Flush FØR sinken kobles fra, ikke etter: flushRaw() skriver gjennom rawSink_
      // og gjør ingenting uten den, så den gamle rekkefølgen kastet stille det siste
      // delvise bufferet - opptil kRawBufBytes, altså en drenerings verdi av data og
      // sync-posten som beskriver den. Ingen drenering kan smyge seg inn mellom de to
      // linjene: INT1-rutinen setter bare et flagg, den tømmer ingenting.
      const uint32_t tStopRaw = timeStart();
      imu_.flushRaw(true);      IWatchdog.reload();
      imu_.setRawSink(nullptr); IWatchdog.reload();
      rawFile_.truncate();      IWatchdog.reload();
      rawFile_.sync();          IWatchdog.reload();
      rawFile_.close();         IWatchdog.reload();
      if (wave_timing_enabled) wave_timing.stopRawUs = micros() - tStopRaw;
    }
    if (wave_timing_enabled) wave_timing.stopTotalUs = micros() - tStop;
    // sessionFile_ stays open: the summary is appended in processReading.
  }

  captureEnd_ = now();
  captureEndPos_ = currentFixE7();
  return 0;
}

// -----------------------------------------------------------------------------
// Session logging (ORB_test Logger style): one timestamped directory per capture,
// created under its final name "<stamp>" and left there.
//
// Whether a capture ran to completion is DERIVED, not marked: writeSessionSummary
// and the spec/ana files are written by processReading, so a capture that died
// mid-stream leaves a ses.csv holding nothing but the anchor keys, and no ana.csv
// beside it. stop_utc_epoch in ses.csv is the discriminator - present means the
// capture reached processReading, absent means it did not. Renaming the folder
// added a second, redundant copy of that fact, and one that a reset could leave
// disagreeing with the files inside it.
// -----------------------------------------------------------------------------
bool WaveManager::startSession(void) {
  SdFat &card = sd_writer.card();

  // Stamp from the RTC (set from GPS in setup). Without a valid clock this falls
  // back to the 1970 epoch stamp; readingID_ in the anchor still disambiguates.
  sprintf(logStamp_, "%04d%02d%02d_%02d%02d%02d",
          year(captureStart_), month(captureStart_), day(captureStart_),
          hour(captureStart_), minute(captureStart_), second(captureStart_));
  
  snprintf(sessionDir_, sizeof(sessionDir_), "%s/%s", wave_log_dir, logStamp_);

  // mkdir creates the missing "waves/" parent too.
  if (!card.exists(sessionDir_) && !card.mkdir(sessionDir_)) {
    if (debug_serial) { Serial.print("WaveManager: mkdir failed "); Serial.println(sessionDir_); }
    return false;
  }

  char nm[64];
  if (wave_mode_imu_csv()) {
    snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_IMU_PREFIX);
    imuFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
    imuCsvActive_ = (bool)imuFile_;
  }
  snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_GPS_PREFIX);
  gpsFile_     = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_SESSION_PREFIX);
  sessionFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  if ((wave_mode_imu_csv() && !imuCsvActive_) || !gpsFile_ || !sessionFile_) {
    if (debug_serial) Serial.println("WaveManager: could not open session files");
    return false;
  }

  // Reserve both streaming files contiguously BEFORE the first byte goes in -
  // preAllocate() refuses once a cluster exists. See wave_config.h for why this is
  // what keeps the FIFO alive. A failure is not fatal: the file simply falls back to
  // growing cluster by cluster, which is the behaviour this replaces, so the capture
  // still runs and only the overflow risk returns.
  //
  // Every reservation must cover the WHOLE capture. Outgrowing the extent mid-stream is
  // not the graceful fallback a failed preAllocate is: the allocator then has to find
  // free clusters past this file's neighbours - raw.bin's 15 MB extent sits right behind
  // gps.csv - while the drain loop is blocked in the write, and a single call that takes
  // longer than the watchdog kills the capture. That is what happened on 2026-08-12:
  // gpsBytes assumed one fix per second, so the extent ran out ~7 min in and every other
  // capture died there. serviceGps writes one row per FRESH fix, so the rate is the
  // receiver's nav rate, not one per second.
  const uint32_t durationS = wave_measurement_duration / s_2_ms;
  const uint32_t imuBytes  = (uint32_t)kRowOdrHz * durationS * wave_imu_row_bytes_max;
  const uint32_t gpsBytes  = durationS * GPS_nav_rate_hz * wave_gps_row_bytes_max;
  if (imuCsvActive_ && !imuFile_.preAllocate(imuBytes) && debug_serial) {
    Serial.print("WaveManager: imu preAllocate failed, "); Serial.print(imuBytes);
    Serial.println(" B - sd-card may be full or fragmented");
  }
  if (!gpsFile_.preAllocate(gpsBytes) && debug_serial) {
    Serial.println("WaveManager: gps preAllocate failed");
  }

  // Raw FIFO log. Opened last and treated as optional throughout: if it fails, the
  // capture still runs and only the undecimated record is lost - the wave chain does
  // not read this file. rawSink_ stays unset in that case, so the emit path costs a
  // null check per word and nothing else.
  if (wave_mode_imu_raw()) {
    snprintf(nm, sizeof(nm), "%s/%s_%s.bin", sessionDir_, logStamp_, WAVE_RAW_PREFIX);
    rawFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
    if (rawFile_) {
      // +20 % on top of the nominal word rate. The nominal figure is what the FIFO
      // batches in a second, and the measured captures land only 4 % under it - too
      // little to absorb a run where the sync records come more often or the SFLP
      // batching shifts. Running out mid-capture is not the graceful fallback a failed
      // preAllocate is; see the gpsBytes comment above for what it costs.
      const uint32_t rawNominal = kRawHeaderBytes + durationS *
                                  (kFifoWordsPerSec * kRawWordBytes + 16u * kRawSyncBytes);
      const uint32_t rawBytes = rawNominal + rawNominal / 5u;
      if (!rawFile_.preAllocate(rawBytes) && debug_serial) {
        Serial.print("WaveManager: raw preAllocate failed, "); Serial.print(rawBytes);
        Serial.println(" B");
      }
      writeRawHeader();
      imu_.setRawSink(&WaveManager::rawSinkTrampoline);
    } else if (debug_serial) {
      Serial.println("WaveManager: could not open raw log - continuing without it");
    }
  }

  if (imuCsvActive_) { imuFile_.println(kImuCsvHeader); imuFile_.sync(); }
  // Decoded SI units, one column per NAV-PVT channel the analysis uses. This is
  // the format postprocess.py and mapplot.py read as they stand - vUp is what
  // the GPS elevation spectrum is built from, and hAccuracy/sats are what the
  // report quotes for fix quality. See serviceGps for the cost this carries.
  // Kolonnene er de gamle minus alt_msl/vN/vE: ingen leser dem. Høyden er GPS-
  // høyde og sier ikke noe om bølgene, og horisontalhastigheten ligger allerede
  // i gspeed. vUp er den ene hastighetskanalen analysen faktisk bygger på.
  gpsFile_.println("rel_ms,utc,lat,lon,gspeed,vUp,head,"
                   "sAccuracy,hAccuracy,vAccuracy,pdop,fix,sats");
  sessionFile_.println("key,value");
  gpsFile_.sync();
  writeSessionAnchor();  // anchor keys (sessionFile_ kept open)

  // cfg.csv: write-once and close - the constants are known at start of session.
  snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_CFG_PREFIX);
  File cfg = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  if (cfg) {
    writeSessionConfig(cfg);
    cfg.sync(); cfg.close();
  } else if (debug_serial) {
    Serial.println("WaveManager: could not open cfg.csv");
  }

  rowsSinceSync_ = 0;
  imuSyncPending_ = false;   // a request left over from the previous session is stale
  if (debug_serial) { Serial.print("WaveManager: logging session to ");Serial.println(sessionDir_); }
  return true;
}

// The session file (build + start time) is written up front so it survives on disk even
// if the capture is interrupted before stopSession. imu/gps use a relative time
// base (ms from start), so these keys alone tie t=0 to real UTC.
void WaveManager::writeSessionAnchor(void) {
  if (!sessionFile_) return;
  char iso[24];
  sprintf(iso, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          year(captureStart_), month(captureStart_), day(captureStart_),
          hour(captureStart_), minute(captureStart_), second(captureStart_));
  sessionFile_.print("reading_id,");      sessionFile_.println(readingID_);
  sessionFile_.print("orientation_name,");sessionFile_.println(analyzer_.orientationName());
  sessionFile_.print("gps_fix_at_start,"); sessionFile_.println(gpsFixAtStart_ ? 1 : 0);
  sessionFile_.print("start_utc_epoch,"); sessionFile_.println((uint32_t)captureStart_);
  sessionFile_.print("start_utc_iso,");   sessionFile_.println(iso);
  sessionFile_.sync();  // do not close: summary is appended at stop
}

// cfg.csv: every constant the capture depends on, so that the session folder
// is self-describing. Enabling postprocessing / testing / verification
void WaveManager::writeSessionConfig(File &f) {
  f.println("key,value");

  // --- capture scheduling ---
  f.print("duration_ms,");        f.println(wave_measurement_duration);
  f.print("period_ms,");          f.println(base_measurement_period_wave_analysis);
  
  // --- AHRS settling window: logged to imu.csv/gps.csv but excluded from Welch/PSD, so ---
  // postprocess must skip the same leading rows to reproduce the on-device Hs.
  f.print("filter_warm_up_ms,");  f.println(wave_measurement_filter_warm_up);

  // GNSS
  f.print("gps_fix_timeout_ms,"); f.println(wave_gps_fix_timeout);
  f.print("gps_fix_required,");   f.println((wave_measurement_require_gps && enable_GPS) ? 1 : 0);

  // --- IMU front end ---
  f.print("imu_odr_hz,");         f.println(kImuOdrHz);
  f.print("accel_odr_hz,");       f.println(kAccelOdrHz);
  f.print("imu_low_power,");      f.println(kImuLowPower);
  f.print("imu_acc_mode,");       f.println((int)kImuAccMode);
  f.print("imu_gyr_mode,");       f.println((int)kImuGyrMode);
  f.print("imu_spi_hz,");         f.println(kImuSpiHz);
  f.print("accel_fs_g,");         f.println((int)kAccelFS);
  f.print("gyro_fs_dps,");        f.println((int)kGyroFS);
  f.print("lpf2_enabled,");       f.println(kUseLpf2 ? 1 : 0);
  f.print("lpf2_bw,");            f.println(kLpf2Bw);        // raw CTRL8 register value
  f.print("lpf2_odr_div,");       f.println(kLpf2Div);
  f.print("lpf2_cutoff_hz,");     f.println(kLpf2CutoffHz, 2);
  // Tells a zero in the _sflp columns apart from a still buoy: 0 here means the fusion
  // block never ran, so those columns carry no information at all.
  f.print("sflp_enabled,");       f.println(kEnableSflp ? 1 : 0);
  f.print("sflp_odr_hz,");        f.println(kSflpOdrHz, 1);
  f.print("sflp_rotation_tag,");  f.println(kSflpRotationTag);
  // "poll" meant something else before 2026-08-15: the drain ran on every loop iteration
  // regardless of level. Both values now describe a watermark-paced drain, and the string
  // is what lets a capture from either side of that change be told apart offline.
  f.print("imu_wake,");           f.println(kImuUseInt1 ? "int1_watermark" : "poll_watermark");
  f.print("fifo_watermark,");     f.println(kFifoWatermark);

  // --- IMU.csv - windowing (raw ODR -> imu.csv rows) ---
  f.print("output_rate_hz,");     f.println(kRowOdrHz);
  f.print("window_ms,");          f.println(kRowPeriodMs);
  f.print("csv_sync_rows,");      f.println(wave_csv_sync_rows);
  f.print("imu_prealloc_bytes,");
  f.println((uint32_t)kRowOdrHz * (wave_measurement_duration / s_2_ms) *
            wave_imu_row_bytes_max);

  // --- decimation ---
  f.print("row_decimation,fir");  f.println();
  f.print("fir_ntap,");           f.println(kFirNtap);
  f.print("fir_s1_cutoff_hz,");   f.println(kFirS1CutoffHz, 3);
  f.print("fir_s2_cutoff_hz,");   f.println(kFirS2CutoffHz, 3);
  f.print("fir_s1_delay_s,");     f.println(kFirS1DelayS, 6);
  f.print("fir_s2_delay_s,");     f.println(kFirS2DelayS, 6);
  f.print("fir_s1_center_ms,");   f.println(kFirS1CenterMs);
  f.print("fir_s2_center_ms,");   f.println(kFirS2CenterMs);
  // The logged series lag by the group delays above. 
  // Offline comparisons must compensate for that
  f.print("fir_compensate,0");    f.println();

  // --- breaking wave detection ---
  f.print("brake_g_thresh,");     f.println(kBrakeGThreshold, 3);
  f.print("brake_thresh_mg2,");   f.println((float)kBrakeThresholdMg2, 1);
  f.print("brake_min_ms,");       f.println(kBrakeMinMs);
  f.print("brake_min_samples,");  f.println(kBrakeMinSamples);

  // --- orientation / vertical acceleration ---
  f.print("orientation_name,");   f.println(analyzer_.orientationName());

  f.print("ahrs_rate_hz,");       f.println(kAhrsInputOdrHz, 2);
  f.print("ahrs_rate_cap_hz,");   f.println(kAhrsInputOdrCapHz);

  f.print("quat_decimation,hold"); f.println();
  f.print("quat_delay_s,");       f.println(kFirS1DelayS, 6);
  f.print("quat_delay_steps,");   f.println(kQuatDelaySteps);

  // --- Madgwick ---
  f.print("madgwick_beta,");      f.println(kMadgwickBeta, 4);

  // --- Kalman ---
  f.print("kalman_sigma_g,");     f.println(kKalmanParams.sigmaG, 6);
  f.print("kalman_sigma_b,");     f.println(kKalmanParams.sigmaB, 8);
  f.print("kalman_r0,");          f.println(kKalmanParams.r0, 8);
  f.print("kalman_dt_ref,");      f.println(kKalmanParams.dtRef, 4);
  f.print("kalman_lambda_a,");    f.println(kKalmanParams.lambdaA, 3);
  f.print("kalman_lambda_w,");    f.println(kKalmanParams.lambdaW, 3);
  f.print("kalman_w0,");          f.println(kKalmanParams.w0, 3);
  f.print("kalman_p0_angle,");    f.println(kKalmanParams.p0Angle, 5);
  f.print("kalman_p0_bias,");     f.println(kKalmanParams.p0Bias, 5);
  f.print("gravity,");            f.println(kGravity, 5);
  f.print("mg_to_ms2,");          f.println(kMg2Ms2, 8);
  f.print("mdps_to_rads,");       f.println(kMdps2Rads, 8);
  f.print("log_mode,");           f.println((uint8_t)wave_log_mode);  // 0=csv 1=raw 2=both
  f.print("raw_format_version,"); f.println(kRawFormatVersion);
  f.print("vacc_bucket_ms,");     f.println(kWelchInputPeriodMs);
  f.print("vacc_fs_hz,");         f.println((float)kWelchInputOdrHz, 3);

  // --- Welch spectrum + acc->elevation taper ---
  f.print("welch_seglen,");       f.println(kWelchSegLen);
  f.print("welch_overlap_div,");  f.println(kWelchOverlapDiv);
  f.print("welch_step,");         f.println(kWelchSegLen / kWelchOverlapDiv);
  f.print("welch_window,");       f.println(kWelchWindow == WindowType::Hann ? "Hann" : "Hamming");
  f.print("psd_df_hz,");          f.println(kPsdDfHz, 6);
  f.print("wave_fmax_hz,");       f.println(kWaveFMax, 3);
  f.print("psd_min_freq_hz,");    f.println(kPsdMinFreq, 3);
  f.print("psd_max_freq_hz,");    f.println(kPsdMaxFreq, 3);
  // The wire message has no version byte, so an archived capture cannot otherwise be
  // told apart from one taken before the companding change - and reading a sqrt payload
  // linearly produces a plausible-looking spectrum rather than an obvious failure.
  // A capture without this key is "linear"; postprocessing keys off exactly that.
  f.print("psd_wire_encoding,");  f.println("sqrt");
  f.print("taper_f1_hz,");        f.println(kTaperF1, 3);
  f.print("taper_f2_hz,");        f.println(kTaperF2, 3);
  f.print("welch_bin_min,");      f.println((uint32_t)welch_bin_min);
  f.print("welch_bin_max,");      f.println((uint32_t)welch_bin_max);
  f.print("welch_bins,");         f.println((uint32_t)welch_bins);
  f.print("spec_sent,");          f.println(kSendPsd ? 1 : 0);
  f.print("spec_tx_bins,");       f.println((uint32_t)kSpecTxBins);
  f.print("spec_n_bins,");        f.println((uint32_t)kSpecNBins);
  f.print("spec_bin_group,");     f.println((uint32_t)kSpecBinGroup);
  f.print("spec_bin_width_hz,");  f.println(kSpecBinWidthHz, 6);
  f.print("spec_f_min_hz,");      f.println(kSpecFMinHz, 5);
  f.print("spec_f_max_hz,");      f.println(kSpecFMaxHz, 5);
  f.print("spec_quantity,");      f.println("acc");   // acc | eta
  f.print("spec_taper_applied,"); f.println(0);
  f.print("spec_band_min_hz,");   f.println(kSpecBandMinHz, 5);
  f.print("spec_band_max_hz,");   f.println(kSpecBandMaxHz, 5);
  f.print("scale_factor,");       f.println(scale_factor);
}

// Append the summary (known only at capture end) to the still-open session file.
void WaveManager::writeSessionSummary(bool ok, const WaveParams &params) {
  if (!sessionFile_) return;
  sessionFile_.print("stop_utc_epoch,"); sessionFile_.println((uint32_t)captureEnd_);
  sessionFile_.print("duration_ms,");    sessionFile_.println(wave_measurement_duration);
  sessionFile_.print("imu_rows,");       sessionFile_.println(analyzer_.rows());
  sessionFile_.print("gps_rows,");       sessionFile_.println(gpsRowsWritten_);
  sessionFile_.print("welch_segments,"); sessionFile_.println(analyzer_.segments());
  sessionFile_.print("usable_spectrum,");sessionFile_.println(ok ? 1 : 0);
  if (ok) {
    sessionFile_.print("Hs,"); sessionFile_.println(params.hs, 3);
    sessionFile_.print("Tz,"); sessionFile_.println(params.tz, 2);
    sessionFile_.print("Tc,"); sessionFile_.println(params.tc, 2);
    sessionFile_.print("Tp,"); sessionFile_.println(params.tp, 2);
  }
  writeTimingBlock();
  sessionFile_.sync();
}

/*
  The timing buckets, in the same key,value form as the rest of ses.csv. This is the half
  of the reporting that survives a field deployment: the [TIM] serial line needs someone
  watching the monitor, and an 18-minute capture in the water has no one.

  n, mean AND max for every bucket. The max is not decoration - a stall is a tail event,
  and a mean over thousands of drains divides a 300 ms outlier down into the noise. The
  mean says what the loop normally costs; the max says whether anything in it could have
  emptied the FIFO budget. Both are per CAPTURE here (the *Cap accumulators), where the
  serial line reports per print interval.

  Written from processReading, so an interrupted capture gets no block - the same way it
  gets no stop_utc_epoch and no ana.csv. Absence is the signal, as everywhere else here.
*/
void WaveManager::writeTimingBlock(void) {
  if (!wave_timing_enabled || !sessionFile_) return;

  for (uint8_t i = 0; i < TIM_COUNT; i++) {
    const TimeStat &s = wave_timing.b[i];
    sessionFile_.print("tim_"); sessionFile_.print(kTimingNames[i]);
    sessionFile_.print("_n,");       sessionFile_.println(s.nCap);
    sessionFile_.print("tim_"); sessionFile_.print(kTimingNames[i]);
    sessionFile_.print("_us_mean,"); sessionFile_.println(s.meanUsCap());
    sessionFile_.print("tim_"); sessionFile_.print(kTimingNames[i]);
    sessionFile_.print("_us_max,");  sessionFile_.println(s.maxUsCap);
  }

  // Turns tim_flush into us/kB rather than us/drain, which is what tells a slow card
  // apart from a big write.
  sessionFile_.print("tim_flush_bytes,");  sessionFile_.println(wave_timing.flushBytesCap);

  // The one-shot costs outside the loop. No FIFO risk, but they are dead time in the
  // capture window - and stop_raw is where a multi-MB reservation is handed back.
  sessionFile_.print("tim_start_session_us,"); sessionFile_.println(wave_timing.startSessionUs);
  sessionFile_.print("tim_stop_imu_us,");      sessionFile_.println(wave_timing.stopImuUs);
  sessionFile_.print("tim_stop_gps_us,");      sessionFile_.println(wave_timing.stopGpsUs);
  sessionFile_.print("tim_stop_raw_us,");      sessionFile_.println(wave_timing.stopRawUs);
  sessionFile_.print("tim_stop_total_us,");    sessionFile_.println(wave_timing.stopTotalUs);

  // The budget every number above is measured against, so the file can be read without
  // the firmware that wrote it. See kFifoFillMs in wave_config.h - this is that same
  // quantity in microseconds, and kDrainIntervalMs is the fraction of it the drain is
  // allowed to use.
  // Computed here in microseconds rather than as kFifoFillMs * 1000: the constant is
  // integer milliseconds, so going through it would report 213000 where the budget is
  // 213333, and this number exists to be measured against.
  sessionFile_.print("tim_fifo_budget_us,");
  sessionFile_.println((uint32_t)kFifoDepthWords * 1000000UL / kFifoWordsPerSec);
}

// Close the session file. The folder already carries its final name, so there is
// nothing to rename: what marks a capture as complete is the summary that
// writeSessionSummary just put in ses.csv, on the card, inside the folder it
// describes. A directory rename could not be made to agree with that under a reset
// - it is a second write, of the same fact, that can land or not land on its own.
void WaveManager::stopSession(void) {
  if (sessionFile_) { sessionFile_.sync(); sessionFile_.close(); }
  csvActive_ = false;
}

// GPS drift track: drive the non-blocking poll and append one gps.csv row per fresh
// fix. update() never blocks long enough to starve the IMU FIFO (see gps_manager).
void WaveManager::serviceGps(uint32_t relMs) {
  gps_manager.update();
  if (!csvActive_ || !gpsFile_ || !gps_manager.freshFix()) return;
  const UBX_PVT &f = gps_manager.lastFix();
  // Scaled to SI here, in the writer, and not kept scaled in UBX_PVT: the module's
  // integers stay exact for everything else that reads lastFix() (the radio message
  // carries lat/lng_e7 as they are), and only the CSV pays for the conversion.
  //
  // The decimals are not cosmetic. 6 on lat/lon is ~0.1 m, one digit finer than the
  // receiver resolves; 4 on the velocities keeps mm/s, which is the quantum vUp
  // arrives in and the floor of the elevation spectrum built from it. Trimming
  // either would throw away resolution the module actually delivered.
  //
  // COST: 12 float conversions per fix, ~7 fixes/s. That is real work in the drain
  // loop, but it happens between FIFO reads and not inside one, and it is the price
  // of a gps.csv the analysis chain reads without a conversion step in between.
  gpsFile_.print(relMs);                    gpsFile_.print(',');
  // UTC as HHMMSSCC, the receiver's own time-of-day - the date belongs to the
  // session (ses.csv start_utc_iso) and is not repeated on every row.
  gpsFile_.print((uint32_t)f.hour * 1000000UL +
                 (uint32_t)f.minute * 10000UL +
                 (uint32_t)f.second * 100UL);
  gpsFile_.print(',');
  gpsFile_.print(f.lat_e7 * 1e-7, 6);       gpsFile_.print(',');
  gpsFile_.print(f.lng_e7 * 1e-7, 6);       gpsFile_.print(',');
  gpsFile_.print(f.gSpeed_mms / 1000.0, 4); gpsFile_.print(',');
  // vUp, not velD: the analysis works in an up-positive elevation, and the sign
  // flip belongs here - at the one place the column is named - rather than in
  // every reader that has to remember which way NED points.
  gpsFile_.print(-f.velD_mms / 1000.0, 4);  gpsFile_.print(',');
  gpsFile_.print(f.headMot_e5 * 1e-5, 2);   gpsFile_.print(',');
  gpsFile_.print(f.sAcc_mms / 1000.0, 2);   gpsFile_.print(',');
  gpsFile_.print(f.hAcc_mm / 1000.0, 2);    gpsFile_.print(',');
  gpsFile_.print(f.vAcc_mm / 1000.0, 2);    gpsFile_.print(',');
  gpsFile_.print(f.pDOP_e2 * 0.01, 2);      gpsFile_.print(',');
  gpsFile_.print(f.fixType);                gpsFile_.print(',');
  gpsFile_.println(f.numSV);
  gpsRowsWritten_++;
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
    char name[64];
    snprintf(name, sizeof(name), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_SPEC_PREFIX);
    File sf = sd_writer.card().open(name, O_RDWR | O_CREAT | O_TRUNC);
    if (sf) {
      sf.println("f_hz,psd_acc,psd_eta");
      const int N = kWelchSegLen;
      const float df = (float)kWelchInputOdrHz / N;
      const float invSeg = analyzer_.segments() > 0 ? 1.0f / (float)analyzer_.segments() : 0.0f;
      const float *psd = analyzer_.psd();
      for (int k = 1; k <= N / 2; k++) {
        float f = k * df;
        if (f > kWaveFMax) break;
        float w = 2.0f * (float)M_PI * f;
        float w4 = w * w * w * w;
        float taper = (f <= kTaperF1) ? 0.0f : (f >= kTaperF2 ? 1.0f
                     : 0.5f * (1.0f - cosf((float)M_PI * (f - kTaperF1) / (kTaperF2 - kTaperF1))));
        if (taper <= 0.0f) continue;
        float pacc = psd[k] * invSeg;
        sf.print(f, 5); sf.print(',');
        sf.print(pacc, 6); sf.print(',');
        sf.println(pacc / w4 * (taper * taper), 6);
      }
      sf.sync(); sf.close();
    }

    snprintf(name, sizeof(name), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_ANA_PREFIX);
    File af = sd_writer.card().open(name, O_RDWR | O_CREAT | O_TRUNC);
    if (af) {
      af.println("key,value");
      af.print("imu_rows,");         af.println(analyzer_.rows());
      af.print("warmup_rows,");      af.println(analyzer_.warmupRows());
      af.print("brake_windows,");    af.println(analyzer_.brakeRows());
      af.print("vacc10hz_samples,"); af.println(analyzer_.samples10Hz());
      // Windows where no raw sample landed on the centre, so the FIR was read at the
      // window edge instead. Non-zero means FIFO gaps - judge a capture by it.
      af.print("fir_late_eval_windows,"); af.println(imu_.firLateEvalCount());
      // Times the FIFO filled during this capture. In FIFO_MODE each one is a hard
      // gap: collection stopped until update() drained and restarted it, so the time
      // axis is compressed by the whole outage. sampleTms_ counts received samples,
      // not elapsed time, so the gap is smeared across the record rather than left
      // as a hole - which puts false low-frequency energy exactly where omega^-4
      // amplifies it. Treat non-zero as grounds for distrusting Hs, not a footnote.
      af.print("fifo_overflows,"); af.println(imu_.overflowTotal());
      // Blocks raw.bin lost. Distinct from fifo_overflows above: that one says the
      // CAPTURE has holes, this one says the FILE does - and a byte-stream format
      // misparses everything after a hole, so non-zero here condemns the raw log even
      // when the capture itself was clean. Written in every log mode; it stays 0 when
      // no raw log was open, which is what "nothing was lost" should look like.
      af.print("raw_write_failures,"); af.println(imu_.rawWriteFailCount());
      af.print("welch_segments,");   af.println(analyzer_.segments());
      af.print("welch_seglen,");     af.println((int)kWelchSegLen);
      // Times the deferred FFT had to run inside the pop loop after all, because the
      // ring had no free slot. Unlike the two counters above this one does not condemn
      // any data - the segments are identical either way - it says the capture kept the
      // deferral's arithmetic without its timing, so tim_welch_us_max understates what
      // the drain actually carried. Non-zero means kWelchRingSlack no longer holds.
      af.print("welch_ring_full,");  af.println(analyzer_.ringFullCount());
      af.print("Hs,"); af.println(params.hs, 3);
      af.print("Tz,"); af.println(params.tz, 2);
      af.print("Tc,"); af.println(params.tc, 2);
      af.print("Tp,"); af.println(params.tp, 2);
      af.sync(); af.close();
    }

    // Close out the session file (anchor + summary).
    writeSessionSummary(ok, params);
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

// -----------------------------------------------------------------------------
// Serialise the front result. A measurement goes out as TWO messages: the
// parameters in msgB ('W' ... 'E') and the spectrum in psdB ('P' ... 'E'), paired
// by ts_start. See readings.h for the layouts and for why the pair is keyed on the
// timestamp rather than on reading_ID.
// -----------------------------------------------------------------------------

// Physical value -> the uint32 fixed point the wave parameters travel in.
static uint32_t waveToFixed(float v) {
  if (!(v > 0.0f)) return 0;                       // undefined (-1) or negative -> 0
  double scaled = (double)v * (double)scale_factor;
  if (scaled > 4294967295.0) return 0xFFFFFFFFUL;  // clamp to uint32 range
  return (uint32_t)llround(scaled);
}

size_t WaveManager::updateTransmitMessage(void) {
  if (wave_analysis_results.empty()) return 0;
  WaveResult res = wave_analysis_results.front();

  uint8_t offset = 0;
  msgB[offset++] = 'W';
  msg_insert_uint(msgB, res.reading_ID, offset, wave_message_size, offset, true);

  // Full time_t, 8 bytes, as every other message here serialises it - the split
  // left this message small enough that narrowing buys nothing. See readings.h.
  msg_insert_uint(msgB, res.timestamp_start, offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, res.timestamp_end,   offset, wave_message_size, offset, true);

  msg_insert_uint(msgB, waveToFixed(res.Hs), offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, waveToFixed(res.Tc), offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, waveToFixed(res.Tp), offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, waveToFixed(res.Tz), offset, wave_message_size, offset, true);

  // msg_insert_int, so the sign survives: sign-and-magnitude, five bytes each, the
  // same encoding the 'G' message uses for coordinates. They are already 1e-7 deg
  // (gps_coord_scale) straight from the receiver, so nothing is rescaled here.
  // 0,0 is what a window with no fix sends, and it means "unknown".
  msg_insert_int(msgB, res.lat_start_e7, offset, wave_message_size, offset, true);
  msg_insert_int(msgB, res.lng_start_e7, offset, wave_message_size, offset, true);
  msg_insert_int(msgB, res.lat_end_e7,   offset, wave_message_size, offset, true);
  msg_insert_int(msgB, res.lng_end_e7,   offset, wave_message_size, offset, true);

  msgB[offset++] = 'E';

  /*
    The result is deliberately NOT popped here. The caller pops with
    popTransmittedResult() once the radio has confirmed TxDone, and a failure
    leaves the result at the head of the queue for the next window.
  */
  return offset;
}

size_t WaveManager::updatePsdTransmitMessage(void) {
  if (wave_analysis_results.empty() || !kSendPsd) return 0;
  WaveResult res = wave_analysis_results.front();

  uint8_t offset = 0;
  psdB[offset++] = 'P';
  msg_insert_uint(psdB, res.reading_ID, offset, wave_spectrum_message_size, offset, true);

  // The join key. Must be byte-identical to the value the 'W' message carried, so
  // it is the same field at the same width and nothing recomputes it.
  msg_insert_uint(psdB, res.timestamp_start, offset, wave_spectrum_message_size, offset, true);

  // max_value gets wave_psd_scale, not scale_factor. An acceleration PSD is orders of
  // magnitude smaller than a wave height or a period, and at 1e5 a calm-sea peak rounds
  // to 0 - which zeroes the whole spectrum on the far side, since every bin is
  // reconstructed as (value/65535)^2 * max_value. See readings.h for the range.
  auto toPsdFixed = [](float v) -> uint32_t {
    if (!(v > 0.0f)) return 0;                       // undefined (-1) or negative -> 0
    double scaled = (double)v * (double)wave_psd_scale;
    if (scaled > 4294967295.0) return 0xFFFFFFFFUL;  // clamp to uint32 range
    return (uint32_t)llround(scaled);
  };
  msg_insert_uint(psdB, toPsdFixed(res.max_value), offset, wave_spectrum_message_size, offset, true);

  // Frequency axis, so the base station can label the bins it is about to read. Bin
  // CENTRES, and the count immediately before the bins themselves - the receiver
  // needs it to know how many to consume.
  // wave_freq_scale, not scale_factor - see readings.h for why the axis needs the
  // finer scale.
  auto toFreqFixed = [](float f) -> uint32_t {
    return (uint32_t)llround((double)f * (double)wave_freq_scale);
  };
  msg_insert_uint(psdB, toFreqFixed(kSpecFMinHz), offset, wave_spectrum_message_size, offset, true);
  msg_insert_uint(psdB, toFreqFixed(kSpecFMaxHz), offset, wave_spectrum_message_size, offset, true);
  msg_insert_uint(psdB, (uint16_t)kSpecTxBins,    offset, wave_spectrum_message_size, offset, true);

  // Last field: it is the only variable-length one, so stopping short here shortens
  // the message without moving anything the receiver has already read. kSpecTxBins is
  // at most welch_bins - the capacity wave_spectrum_message_size was budgeted for - and
  // is smaller whenever kPsdMaxFreq does not divide evenly into the bin grid.
  for (size_t i = 0; i < kSpecTxBins; i++) {
    msg_insert_uint(psdB, res.wave_spectrum[i], offset, wave_spectrum_message_size, offset, true);
  }
  psdB[offset++] = 'E';
  return offset;
}

void WaveManager::popTransmittedResult(void) {
  if (!wave_analysis_results.empty()) wave_analysis_results.pop_front();
}

#if DEBUG_WAVE_MSG
// Synthetic result for bench testing - see DEBUG_WAVE_MSG in wave_config.h.
void WaveManager::enqueueFakeResult(void) {
  WaveResult res{};
  res.reading_ID = ++readingID_;

  // Deliberately NOT round numbers, and all distinct: if the fixed-point scaling or
  // the field ORDER is wrong on the receiving side, distinct odd values say so
  // immediately, where 1.0/2.0/3.0 could line up plausibly after a swap.
  res.Hs        = 1.37f;   // m
  res.Tc        = 2.53f;   // s
  res.Tp        = 6.91f;   // s
  res.Tz        = 4.29f;   // s
  res.max_value = 0.0842f; // peak acceleration PSD ((m/s^2)^2/Hz)

  // A single smooth peak, encoded exactly as finalize() does: the wire value is
  // sqrt(binAcc/peakAcc) * 65535, so the far side reconstructs
  // (value/65535)^2 * max_value. The sqrt has to be here too - a fixture that encodes
  // linearly would still decode to a plausible-looking gaussian on the receiver and
  // stop testing the one thing it exists to test.
  // Peak placed off-centre so a mirrored or off-by-one bin axis is visible.
  const float peakBin = 0.35f * (float)kSpecNBins;
  const float width   = 0.12f * (float)kSpecNBins;
  for (size_t j = 0; j < kSpecNBins; j++) {
    const float d = ((float)j - peakBin) / width;
    const float norm = expf(-0.5f * d * d);
    res.wave_spectrum[j] = (uint16_t)lroundf(sqrtf(norm) * 65535.0f);
  }

  res.timestamp_end   = now();
  res.timestamp_start = res.timestamp_end -
                        (time_t)(wave_measurement_duration / s_2_ms);

  // A short synthetic drift, and both signs present: a receiver that drops the
  // sign character, or reads the pair in the wrong order, cannot produce these
  // four numbers by accident. West of Greenwich on purpose - a bench test that
  // only ever sees positive coordinates would not exercise the 'N' branch.
  res.lat_start_e7 =  599578000;   //  59.9578 N
  res.lng_start_e7 =  110686000;   //  11.0686 E
  res.lat_end_e7   =  599601000;   //  59.9601 N, drifted north
  res.lng_end_e7   =  -110701000;  // -11.0701, i.e. W: exercises the sign byte

  // Same bound handling as processReading: the deque is fixed-size, and dropping the
  // OLDEST keeps the freshest results when transmit cannot keep up.
  if (wave_analysis_results.full()) wave_analysis_results.pop_back();
  wave_analysis_results.push_front(res);
}

// What is about to go on the air, in physical units - so a scaling or field-order
// mistake in updateTransmitMessage is visible against these numbers rather than only
// after decoding on the base station.
void WaveManager::printPendingResult(Print &out) const {
  if (wave_analysis_results.empty()) return;
  const WaveResult &r = wave_analysis_results.front();

  out.print(F("  wave result #"));  out.println(r.reading_ID);
  out.print(F("    Hs "));  out.print(r.Hs, 3);  out.print(F(" m   Tc "));
  out.print(r.Tc, 3);       out.print(F(" s   Tp "));
  out.print(r.Tp, 3);       out.print(F(" s   Tz "));
  out.print(r.Tz, 3);       out.println(F(" s"));
  out.print(F("    max_value "));  out.print(r.max_value, 9);
  out.print(F(" (m/s^2)^2/Hz   span "));
  out.print((uint32_t)(r.timestamp_end - r.timestamp_start));  out.println(F(" s"));

  // sprintf, not out.print(float): an epoch near 1.77e9 does not survive a float
  // round trip (24-bit mantissa -> rounded to the nearest 128 s), and these are the
  // fields that wrap if the RTC was never set, so they must be exact to be useful.
  char ts[64];  // "    window " + two 10-digit values + " .. " is 39; leave margin
  sprintf(ts, "    window %lu .. %lu",
          (unsigned long)r.timestamp_start, (unsigned long)r.timestamp_end);
  out.println(ts);

  // Position at each end of the window, printed as the receiver will read it:
  // 1e-7 deg back to degrees. 0,0 is what a window without a fix sends, and it is
  // labelled rather than printed as a coordinate off West Africa.
  auto printPos = [&out](const __FlashStringHelper *label, int32_t lat, int32_t lng) {
    out.print(label);
    if (lat == 0 && lng == 0) { out.println(F(" no fix (0,0)")); return; }
    out.print(' ');  out.print((double)lat / gps_coord_scale, 6);
    out.print(F(", ")); out.println((double)lng / gps_coord_scale, 6);
  };
  printPos(F("    pos start"), r.lat_start_e7, r.lng_start_e7);
  printPos(F("    pos end  "), r.lat_end_e7,   r.lng_end_e7);

  // The wire format is a normalised, sqrt-companded uint16 per bin; the base station
  // reconstructs (value/65535)^2 * max_value. Both are printed so the raw payload can
  // be checked against the decoded value without doing the arithmetic by hand - and
  // this line must stay identical to print_wave_analysis_reading() in
  // message_parser.cpp, since disagreeing with it is what would reveal a half-finished
  // format change.
  //
  // The frequency is built from kSpecFMinHz and kSpecBinWidthHz - the two values
  // updateTransmitMessage puts in the message - rather than from welch_bin_min and
  // the group size, so this really is the receiver's arithmetic and not a parallel
  // derivation that could agree here and disagree over the air.
  if (!kSendPsd) {
    out.println(F("    PSD not transmitted (kSendPsd off) - num_bins 0"));
    return;
  }

  out.print(F("    PSD, "));      out.print((uint32_t)kSpecTxBins);
  out.print(F(" bins, f_min "));  out.print(kSpecFMinHz, 4);
  out.print(F(" Hz, f_max "));    out.print(kSpecFMaxHz, 4);
  out.print(F(" Hz, df "));       out.print(kSpecBinWidthHz, 6);
  out.println(F(" Hz (f_hz raw psd_acc):"));
  for (size_t j = 0; j < kSpecTxBins; j++) {
    out.print(F("      "));      out.print(kSpecFMinHz + j * kSpecBinWidthHz, 4);
    out.print(' ');              out.print(r.wave_spectrum[j]);
    out.print(' ');
    const float n = r.wave_spectrum[j] / 65535.0f;
    out.println(n * n * r.max_value, 9);
  }
}
#endif  // DEBUG_WAVE_MSG
