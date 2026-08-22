#include "wave_manager.h"

#include <math.h>
#include <TimeLib.h>
#include "IWatchdog.h"
#include "sd_writer.h"

/*
  Everything WaveManager writes to the sd-card: the session directory and its six CSV
  files, plus the reading-ID continuity that depends on the directory count.

  Split out of wave_manager.cpp, which keeps the capture loop. Same class - these are
  WaveManager members - so nothing about the interface changes; the file boundary is
  there because file format and capture flow are read for different reasons.
*/

// platformio.ini passes the commit and branch UNQUOTED (-DREPO_COMMIT_ID=b402450...),
// so they have to be stringified here. A hash that starts with a digit is a valid
// preprocessing number and nothing else - # is the only thing that can consume it.
// The fallbacks cover a build outside the ini, where the flags are simply absent.
#ifndef REPO_COMMIT_ID
#define REPO_COMMIT_ID unknown
#endif
#ifndef REPO_GIT_BRANCH
#define REPO_GIT_BRANCH unknown
#endif
#define BUILD_STR2(x) #x
#define BUILD_STR(x)  BUILD_STR2(x)

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

// One row, in the column order kImuCsvHeader declares. Called from onRow, i.e. from
// inside the FIFO pop loop - the decimals are budgeted there, not chosen freely.
void WaveManager::appendImuCsvRow(const ImuRow &r) {
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
}// -----------------------------------------------------------------------------
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
  }
  // No drift track means no gps.csv at all, rather than one holding only its header -
  // that is what a failed receiver produces, and the two must stay distinguishable.
  // cfg.csv's gps_track_in_capture is what tells them apart.
  if (wave_gps_track_in_capture) {
    snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_GPS_PREFIX);
    gpsFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  }
  snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_SESSION_PREFIX);
  sessionFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  if ((wave_mode_imu_csv() && !imuFile_) ||
      (wave_gps_track_in_capture && !gpsFile_) || !sessionFile_) {
    if (debug_serial) Serial.println("WaveManager: could not open session files");
    return false;
  }

  // Reserve both streaming files contiguously BEFORE the first byte goes in -
  // preAllocate() refuses once a cluster exists. Allocating clusters mid-capture is
  // what threatens the FIFO. A failure here is not fatal: the file falls back to
  // growing cluster by cluster, so the capture still runs and only the risk returns.
  //
  // Every reservation must cover the WHOLE capture. Outgrowing the extent mid-stream is
  // NOT the graceful fallback a failed preAllocate is: the allocator then has to find
  // free clusters past this file's neighbours - raw.bin's 15 MB extent sits right
  // behind gps.csv - while the drain loop is blocked in the write, and one call longer
  // than the watchdog kills the capture. gpsBytes therefore uses GPS_nav_rate_hz and
  // not one row per second: serviceGps writes one row per FRESH fix, so the rate is the
  // receiver's, and assuming one per second ran the extent out ~7 minutes in.
  const uint32_t durationS = wave_measurement_duration / s_2_ms;
  const uint32_t imuBytes  = (uint32_t)kRowOdrHz * durationS * wave_imu_row_bytes_max;
  const uint32_t gpsBytes  = durationS * GPS_nav_rate_hz * wave_gps_row_bytes_max;
  if (imuFile_ && !imuFile_.preAllocate(imuBytes) && debug_serial) {
    Serial.print("WaveManager: imu preAllocate failed, "); Serial.print(imuBytes);
    Serial.println(" B - sd-card may be full or fragmented");
  }
  if (gpsFile_ && !gpsFile_.preAllocate(gpsBytes) && debug_serial) {
    Serial.println("WaveManager: gps preAllocate failed");
  }

  // Raw FIFO log. Opened last and optional throughout: if it fails the capture still
  // runs and only the undecimated record is lost - the wave chain does not read this
  // file. The sampler's rawLog_ pointer stays null in that case, so the emit path costs
  // a null check per word and nothing else.
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
      // Sink first: writeHeader goes through it, as every other record does.
      rawLog_.reset();
      rawLog_.setSink(&WaveManager::rawSinkTrampoline);
      rawLog_.writeHeader((uint32_t)captureStart_, readingID_);
      imu_.setRawLog(&rawLog_);
    } else if (debug_serial) {
      Serial.println("WaveManager: could not open raw log - continuing without it");
    }
  }

  if (imuFile_) { imuFile_.println(kImuCsvHeader); imuFile_.sync(); }

  // Gps-logging
  if (gpsFile_) {
    gpsFile_.println("rel_ms,utc,itow,lat,lon,gspeed,vN,vE,vUp,head,"
                     "sAccuracy,hAccuracy,vAccuracy,pdop,sats");
    gpsFile_.sync();
  }

  sessionFile_.println("key,value");
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

// One end of the capture, in degrees at the same six decimals gps.csv uses. 0,0 is the
// "no fix" value currentFixE7 returns, not a position.
void WaveManager::writePosition(const char *when, const FixE7 &p) {
  sessionFile_.print("lat_"); sessionFile_.print(when); sessionFile_.print(',');
  sessionFile_.println(p.lat * 1e-7, 6);
  sessionFile_.print("lon_"); sessionFile_.print(when); sessionFile_.print(',');
  sessionFile_.println(p.lng * 1e-7, 6);
}

// The session file (build + start time + start position) is written up front so it
// survives on disk even if the capture is interrupted before stopSession. imu/gps use a
// relative time base (ms from start), so these keys alone tie t=0 to real UTC.
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
  writePosition("start", captureStartPos_);
  sessionFile_.sync();  // do not close: summary is appended at stop
}

// cfg.csv: every constant the capture depends on, so that the session folder
// is self-describing. Enabling postprocessing / testing / verification
void WaveManager::writeSessionConfig(File &f) {
  f.println("key,value");

  // Build date and commit/branch. NB at time of file compilation, so need to do 
  // clean rebuild to refresh.
  f.print("build_date,");         f.println(__DATE__ " " __TIME__);
  f.print("build_commit,");       f.println(BUILD_STR(REPO_COMMIT_ID));
  f.print("build_branch,");       f.println(BUILD_STR(REPO_GIT_BRANCH));

  // --- capture scheduling ---
  f.print("duration_ms,");        f.println(wave_measurement_duration);
  f.print("period_ms,");          f.println(base_measurement_period_wave_analysis);
  
  // --- AHRS settling window: logged to imu.csv/gps.csv but excluded from Welch/PSD, so ---
  // postprocess must skip the same leading rows to reproduce the on-device Hs.
  f.print("filter_warm_up_ms,");  f.println(wave_measurement_filter_warm_up);

  // GNSS
  f.print("gps_fix_timeout_ms,"); f.println(wave_gps_fix_timeout);
  f.print("gps_fix_required,");   f.println((wave_measurement_require_gps && enable_GPS) ? 1 : 0);
  // 0 means the capture deliberately wrote no gps.csv. Without this key a session folder
  // with no drift track is indistinguishable from one where the receiver failed.
  f.print("gps_track_in_capture,"); f.println(wave_gps_track_in_capture ? 1 : 0);

  // --- IMU front end ---
  f.print("imu_odr_hz,");         f.println(kImuOdrHz);
  f.print("accel_odr_hz,");       f.println(kImuOdrHz);
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
  f.print("sflp_rotation_tag,");  f.println(kTagSflpRotation);
  // Which drain trigger the capture used. The strings distinguish it from an older
  // firmware where "poll" meant draining on every loop iteration regardless of level;
  // both values here describe a watermark-paced drain.
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

// spec.csv: the averaged spectrum bin by bin, as acceleration AND as elevation. The
// taper is recomputed here rather than read back from the analyzer, because this file
// is the record of what the analysis SAW, not of what it reported.
void WaveManager::writeSpecCsv(void) {
  char name[64];
  snprintf(name, sizeof(name), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_SPEC_PREFIX);
  File sf = sd_writer.card().open(name, O_RDWR | O_CREAT | O_TRUNC);
  if (!sf) return;

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

// ana.csv: the wave parameters
void WaveManager::writeAnaCsv(bool ok, const WaveParams &params) {
  char name[64];
  snprintf(name, sizeof(name), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_ANA_PREFIX);
  File af = sd_writer.card().open(name, O_RDWR | O_CREAT | O_TRUNC);
  if (!af) return;

  af.println("key,value");
  af.print("imu_rows,");         af.println(analyzer_.rows());
  af.print("warmup_rows,");      af.println(analyzer_.warmupRows());
  af.print("brake_windows,");    af.println(analyzer_.brakeRows());
  af.print("vacc10hz_samples,"); af.println(analyzer_.samples10Hz());

  // Windows where no raw sample landed on the centre, so the FIR was read at the window
  // edge instead. Non-zero means FIFO gaps - judge a capture by it.
  af.print("fir_late_eval_windows,"); af.println(imu_.firLateEvalCount());

  // Times the FIFO overran during this capture. sampleTms_ counts received samples, not
  // elapsed time, so a gap is SMEARED across the record rather than left as a hole -
  // which puts false low-frequency energy exactly where omega^-4 amplifies it. Treat
  // non-zero as grounds for distrusting Hs, not as a footnote.
  af.print("fifo_overflows,"); af.println(imu_.overflowTotal());

  // Blocks raw.bin lost. Distinct from fifo_overflows above: that one says the CAPTURE
  // has holes, this one says the FILE does - and a byte-stream format misparses
  // everything after a hole, so non-zero here condemns the raw log even when the
  // capture itself was clean. Written in every log mode; it stays 0 when no raw log was
  // open, which is what "nothing was lost" should look like.
  af.print("raw_write_failures,"); af.println(rawLog_.writeFailCount());

  af.print("welch_segments,");   af.println(analyzer_.segments());
  af.print("welch_seglen,");     af.println((int)kWelchSegLen);

  // Times the deferred FFT had to run inside the pop loop after all, because the ring
  // had no free slot. Unlike the two counters above this one does not condemn any data
  // - the segments are identical either way - it says the capture kept the deferral's
  // arithmetic without its timing, so tim_welch_us_max understates what the drain
  // actually carried. Non-zero means kWelchRingSlack no longer holds.
  af.print("welch_ring_full,");  af.println(analyzer_.ringFullCount());

  // 0 means finalize() found too few Welch segments to build a spectrum from. The four
  // parameters below are then the -1 sentinels finalize() left in place, written out
  // rather than omitted: a reader that keys off the value sees the failure, where a
  // missing key is indistinguishable from an older firmware that never wrote it.
  af.print("usable_spectrum,"); af.println(ok ? 1 : 0);

  af.print("Hs,"); af.println(params.hs, 3);
  af.print("Tz,"); af.println(params.tz, 2);
  af.print("Tc,"); af.println(params.tc, 2);
  af.print("Tp,"); af.println(params.tp, 2);
  af.sync(); af.close();
}

// Append the summary (known only at capture end) to the still-open session file.
//
// The ANALYSIS belongs to ana.csv and is not repeated here: what the capture ran to,
// how much of it reached the card, and what the loop cost. stop_utc_epoch keeps its
// second job as the completion marker - it is written from the same call as ana.csv,
// so the pair is still what separates a finished capture from an interrupted one.
void WaveManager::writeSessionSummary(void) {
  if (!sessionFile_) return;
  sessionFile_.print("stop_utc_epoch,"); sessionFile_.println((uint32_t)captureEnd_);
  writePosition("stop", captureEndPos_);
  sessionFile_.print("duration_ms,");    sessionFile_.println(wave_measurement_duration);
  // Rows in gps.csv, not an analysis count - the on-board chain never reads the drift
  // track, so this says what was LOGGED and stays on the session side.
  sessionFile_.print("gps_rows,");       sessionFile_.println(gpsRowsWritten_);
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
  // the firmware that wrote it. See kFifoFillMs in imu_config.h - this is that same
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
  sessionActive_ = false;
}

// GPS drift track: drive the non-blocking poll and append one gps.csv row per fresh
// fix. update() never blocks long enough to starve the IMU FIFO (see gps_manager).
void WaveManager::serviceGps(uint32_t relMs) {
  gps_manager.update();
  if (!sessionActive_ || !gpsFile_ || !gps_manager.freshFix()) return;
  const UBX_PVT &f = gps_manager.lastFix();
  // A PVT that is not gnssFixOK leaves every field below at the PREVIOUS solution -
  // the parser only fills them when valid - so such a row is the last position
  // repeated under a new timestamp, not a measurement. It used to go in the file with
  // fix=0 for the reader to drop; without that column it has to be dropped here.
  //
  // 3D and not merely valid (which is fixType >= 2): a 2D solution holds the height
  // fixed, so its velD - the vUp this file exists to provide - is not measured but
  // assumed. mapplot.py applied exactly this cut on the fix column it no longer has.
  if (!f.valid || f.fixType < 3) return;
  // Scaled to SI here, in the writer, and not kept scaled in UBX_PVT: the module's
  // integers stay exact for everything else that reads lastFix() (the radio message
  // carries lat/lng_e7 as they are), and only the CSV pays for the conversion.
  //
  // The decimals are not cosmetic. 6 on lat/lon is ~0.1 m, one digit finer than the
  // receiver resolves; 4 on the velocities keeps mm/s, which is the quantum vUp
  // arrives in and the floor of the elevation spectrum built from it. Trimming
  // either would throw away resolution the module actually delivered.
  //
  // COST: 13 float conversions per fix. That is real work in the drain
  // loop, but it happens between FIFO reads and not inside one, and it is the price
  // of a gps.csv the analysis chain reads without a conversion step in between.
  gpsFile_.print(relMs);                    gpsFile_.print(',');
  // UTC as HHMMSSCC, the receiver's own time-of-day - the date belongs to the
  // session (ses.csv start_utc_iso) and is not repeated on every row.
  gpsFile_.print((uint32_t)f.hour * 1000000UL +
                 (uint32_t)f.minute * 10000UL +
                 (uint32_t)f.second * 100UL);
  gpsFile_.print(',');
  // The receiver's own epoch time, ms into the GPS week. rel_ms is when the firmware got
  // round to reading the frame - up to GPS_ddc_check_ms late, and later still behind an SD
  // stall - while this is when the solution was computed. utc above resolves to a whole
  // second, which cannot separate two 10 Hz epochs, so this is the only column the wave
  // analysis can build an even time base from.
  gpsFile_.print(f.iTOW_ms);                gpsFile_.print(',');
  gpsFile_.print(f.lat_e7 * 1e-7, 6);       gpsFile_.print(',');
  gpsFile_.print(f.lng_e7 * 1e-7, 6);       gpsFile_.print(',');
  gpsFile_.print(f.gSpeed_mms / 1000.0, 4); gpsFile_.print(',');
  gpsFile_.print(f.velN_mms / 1000.0, 4);   gpsFile_.print(',');
  gpsFile_.print(f.velE_mms / 1000.0, 4);   gpsFile_.print(',');
  // vUp, not velD: the analysis works in an up-positive elevation, and the sign
  // flip belongs here - at the one place the column is named - rather than in
  // every reader that has to remember which way NED points.
  gpsFile_.print(-f.velD_mms / 1000.0, 4);  gpsFile_.print(',');
  gpsFile_.print(f.headMot_e5 * 1e-5, 2);   gpsFile_.print(',');
  gpsFile_.print(f.sAcc_mms / 1000.0, 2);   gpsFile_.print(',');
  gpsFile_.print(f.hAcc_mm / 1000.0, 2);    gpsFile_.print(',');
  gpsFile_.print(f.vAcc_mm / 1000.0, 2);    gpsFile_.print(',');
  gpsFile_.print(f.pDOP_e2 * 0.01, 2);      gpsFile_.print(',');
  gpsFile_.println(f.numSV);
  gpsRowsWritten_++;
}