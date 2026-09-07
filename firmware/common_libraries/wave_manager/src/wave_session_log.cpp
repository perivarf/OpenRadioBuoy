#include "wave_manager.h"

#include <math.h>
#include <TimeLib.h>
#include "IWatchdog.h"
#include "sd_writer.h"

/*
  Everything WaveManager writes to the sd-card: 
  
  * _ana.csv (Significant Wave Height, number of measurements, etc)
  * _imu.csv (IMU data, optional, depending on wave_log_mode)
  * _cfg.csv (Configuration data)
  * _gps.csv (GPS data)
  * _ses.csv (Session data, timing, lat/lng)
  * _spec.csv (Welch spectrum)
*/

// platformio.ini passes the commit and branch UNQUOTED (-DREPO_COMMIT_ID=b402450...),
// so they have to be stringified here. 
#ifndef REPO_COMMIT_ID
#define REPO_COMMIT_ID unknown
#endif
#ifndef REPO_GIT_BRANCH
#define REPO_GIT_BRANCH unknown
#endif
#define BUILD_STR2(x) #x
#define BUILD_STR(x)  BUILD_STR2(x)

// imu.csv column header. Naming convention:
//   unsuffixed  = the SELECTED filter (AhrsFilter, or SFLP when wave_use_sflp)
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
// Count number of session directories on the card (for use by seedReadingId)
// -----------------------------------------------------------------------------
uint16_t WaveManager::countSessionDirs(void) {
  SdFat &card = sd_writer.card();
  if (!card.exists(wave_log_dir)) return 0;   // first capture on a fresh card

  File dir;
  if (!dir.open(wave_log_dir, O_RDONLY)) return 0;

  uint16_t n = 0;
  File entry;
  // Count directories only
  while (entry.openNext(&dir, O_RDONLY)) {
    if (entry.isDir() && !entry.isHidden()) n++;
    entry.close();
    IWatchdog.reload();  // a card with hundreds of sessions must not trip the watchdog
  }
  dir.close();
  return n;
}

// Get reading_ID at boot by counting the number of session directories on the card
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
// Session logging: one timestamped directory per capture
//
// Whether a capture ran to completion is not marked, but inferred by 
// the presence of session-file.
// -----------------------------------------------------------------------------
bool WaveManager::startSession(void) {
  SdFat &card = sd_writer.card();

  // Stamp from the RTC (set from GPS in setup). Without a valid clock this falls
  // back to the 1970..
  sprintf(logStamp_, "%04d%02d%02d_%02d%02d%02d",
          year(captureStart_), month(captureStart_), day(captureStart_),
          hour(captureStart_), minute(captureStart_), second(captureStart_));
  
  // Session dir name: "waves/20240101_123456" for a capture that started at 2024-01-01 12:34:56.
  snprintf(sessionDir_, sizeof(sessionDir_), "%s/%s", wave_log_dir, logStamp_);

  // mkdir creates the missing "waves/" parent too.
  if (!card.exists(sessionDir_) && !card.mkdir(sessionDir_)) {
    if (debug_serial) { Serial.print("WaveManager: mkdir failed "); Serial.println(sessionDir_); }
    return false;
  }

  // imu.csv opened if wave_log_mode is Csv or Both
  char nm[64];
  if (wave_mode_imu_csv()) {
    snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_IMU_PREFIX);
    imuFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  }

  // gps.csv opened if wave_gps_track_in_capture is set
  if (wave_gps_track_in_capture) {
    snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_GPS_PREFIX);
    gpsFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  }

  // ses.csv opened always, for session anchors and timing
  snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_SESSION_PREFIX);
  sessionFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  if ((wave_mode_imu_csv() && !imuFile_) ||
      (wave_gps_track_in_capture && !gpsFile_) || !sessionFile_) {
    if (debug_serial) Serial.println("WaveManager: could not open session files");
    return false;
  }

  // raw.bin opened if wave_mode_imu_raw
  if (wave_mode_imu_raw()) {
    snprintf(nm, sizeof(nm), "%s/%s_%s.bin", sessionDir_, logStamp_, WAVE_RAW_PREFIX);
    rawFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
    if (!rawFile_ && debug_serial) {
      Serial.println("WaveManager: could not open raw log - continuing without it");
    }
  }

  // Reserve every streaming file contiguously before the first byte goes in
  // A failure here is not fatal: the file falls back to growing cluster by cluster
  // The preallocation must cover the whole capture, otherwise FIFO may overflow
  // due to SD card takes too long to allocate a new cluster.
  const uint32_t durationS = wave_measurement_duration / s_2_ms;
  const uint32_t imuBytes  = (uint32_t)kRowOdrHz * durationS * wave_imu_row_bytes_max;
  const uint32_t gpsBytes  = GPS_nav_rate_hz *     durationS * wave_gps_row_bytes_max;

  if (imuFile_ && !imuFile_.preAllocate(imuBytes) && debug_serial) {
    Serial.print("WaveManager: imu preAllocate failed, "); Serial.print(imuBytes);
    Serial.println(" B - sd-card may be full or fragmented");
  }

  if (gpsFile_ && !gpsFile_.preAllocate(gpsBytes) && debug_serial) {
    Serial.println("WaveManager: gps preAllocate failed");
  }

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
  }

  // IMU, GPS, Session headers
  if (imuFile_) { imuFile_.println(kImuCsvHeader); imuFile_.sync(); }

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
  imuSyncPending_ = false;
  if (debug_serial) { Serial.print("WaveManager: logging session to ");Serial.println(sessionDir_); }
  return true;
}

// One end of the capture (i.e. when=start/stop), in degrees with six decimals
// 0,0 is the "no fix" value currentFixE7 returns, not a position.
void WaveManager::writePosition(const char *when, const FixE7 &p) {
  sessionFile_.print("lat_"); sessionFile_.print(when); sessionFile_.print(',');
  sessionFile_.println(p.lat * 1e-7, 6);
  sessionFile_.print("lon_"); sessionFile_.print(when); sessionFile_.print(',');
  sessionFile_.println(p.lng * 1e-7, 6);
}

// The session file (build + start time + start position) is written up front so it
// survives on disk even if the capture is interrupted before stopSession. imu/gps use a
// relative time base (ms from start), so these keys tie t=0 to real UTC.
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

// cfg.csv: (hopefully) every constant the capture depends on, so that the session folder
// is self-describing. Enabling postprocessing / testing / verification
void WaveManager::writeSessionConfig(File &f) {
  f.println("key,value");

  // Build date and commit/branch. 
  // NB at time of file compilation, so need to do clean rebuild to refresh.
  f.print("build_date,");         f.println(__DATE__ " " __TIME__);
  f.print("build_commit,");       f.println(BUILD_STR(REPO_COMMIT_ID));
  f.print("build_branch,");       f.println(BUILD_STR(REPO_GIT_BRANCH));

  // Capture duration etc
  f.print("duration_ms,");        f.println(wave_measurement_duration);
  f.print("period_ms,");          f.println(base_measurement_period_wave_analysis);
  
  // AHRS settling window: logged to imu.csv/gps.csv but excluded from Welch/PSD, so
  // postprocess must skip the same leading rows to reproduce the on-device Hs.
  f.print("filter_warm_up_ms,");  f.println(wave_measurement_filter_warm_up);

  // GNSS
  // The nav rate the receiver is SET to against which the achieved fix rate from the track is read
  f.print("gps_rate_hz,");        f.println(GPS_nav_rate_hz);
  f.print("gps_fix_timeout_ms,"); f.println(wave_gps_fix_timeout);
  f.print("gps_fix_required,");   f.println((wave_measurement_require_gps && enable_GPS) ? 1 : 0);
  f.print("gps_track_in_capture,"); f.println(wave_gps_track_in_capture ? 1 : 0);

  // IMU
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
  f.print("lpf2_cutoff_hz,");     f.println(kLpf2CutoffHz, 2);
  f.print("sflp_enabled,");       f.println(kEnableSflp ? 1 : 0);
  f.print("sflp_odr_hz,");        f.println(kSflpOdrHz, 1);
  f.print("sflp_rotation_tag,");  f.println(kTagSflpRotation);
  f.print("imu_wake,");           f.println(kImuUseInt1 ? "int1_watermark" : "poll_watermark");
  f.print("fifo_watermark,");     f.println(kFifoWatermark);

  // --- IMU.csv - windowing (raw ODR -> imu.csv rows) ---
  f.print("output_rate_hz,");     f.println(kRowOdrHz);
  f.print("window_ms,");          f.println(kRowPeriodMsF, 4);
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
  f.print("fir_s1_center_ms,");   f.println(0.5f * kRowPeriodMsF, 4);
  f.print("fir_s2_center_ms,");   f.println(kFirS2CenterMs);
  f.print("fir_compensate,0");    f.println();

  // breaking wave detection
  f.print("brake_g_thresh,");     f.println(kBrakeGThreshold, 3);
  f.print("brake_thresh_mg2,");   f.println((float)kBrakeThresholdMg2, 1);
  f.print("brake_min_ms,");       f.println(kBrakeMinMs);
  f.print("brake_min_samples,");  f.println(kBrakeMinSamples);

  // orientation / vertical acceleration
  f.print("orientation_name,");   f.println(analyzer_.orientationName());
  f.print("ahrs_rate_hz,");       f.println(kAhrsInputOdrHz, 2);
  f.print("quat_decimation,hold"); f.println();
  f.print("quat_delay_s,");       f.println(kFirS1DelayS, 6);
  f.print("quat_delay_steps,");   f.println(kQuatDelaySteps);

  // Madgwick
  f.print("madgwick_beta,");      f.println(kMadgwickBeta, 4);

  // Kalman
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

  // Welch spectrum + acc->elevation taper
  f.print("welch_seglen,");       f.println(kWelchSegLen);
  f.print("welch_overlap_div,");  f.println(kWelchOverlapDiv);
  f.print("welch_step,");         f.println(kWelchSegLen / kWelchOverlapDiv);
  f.print("welch_window,");       f.println(kWelchWindow == WindowType::Hann ? "Hann" : "Hamming");
  f.print("psd_df_hz,");          f.println(kPsdDfHz, 6);
  f.print("wave_fmax_hz,");       f.println(kWaveFMax, 3);
  f.print("psd_min_freq_hz,");    f.println(kPsdMinFreq, 3);
  f.print("psd_max_freq_hz,");    f.println(kPsdMaxFreq, 3);
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
// taper is recomputed here rather than read back from the analyzer
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
  af.print("vacc_welch_samples,"); af.println(analyzer_.samplesWelch());

  // Windows where no raw sample landed on the centre, so the FIR was read at the window
  // edge instead. Non-zero means FIFO gaps
  af.print("fir_late_eval_windows,"); af.println(imu_.firLateEvalCount());

  // Times the FIFO overran during this capture
  af.print("fifo_overflows,"); af.println(imu_.overflowTotal());

  // Blocks raw.bin lost
  af.print("raw_write_failures,"); af.println(rawLog_.writeFailCount());

  // Welch and calculated wave parameters
  af.print("welch_segments,");   af.println(analyzer_.segments());
  af.print("welch_seglen,");     af.println((int)kWelchSegLen);
  af.print("welch_ring_full,");  af.println(analyzer_.ringFullCount()); // Should always be zero..
  af.print("usable_spectrum,"); af.println(ok ? 1 : 0);
  af.print("Hs,"); af.println(params.hs, 3);
  af.print("Tz,"); af.println(params.tz, 2);
  af.print("Tc,"); af.println(params.tc, 2);
  af.print("Tp,"); af.println(params.tp, 2);
  af.sync(); af.close();
}

// Append the capture-end summary to the session file.
// Stop, lat/long, duration, gps_rows, timing buckets
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
  The timing buckets are written to the session file at the end of a capture
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

  // Turns tim_flush into us/kB rather than us/drain
  sessionFile_.print("tim_flush_bytes,");  sessionFile_.println(wave_timing.flushBytesCap);

  // The one-shot costs outside the loop. 
  // No FIFO risk, so not so important.
  sessionFile_.print("tim_start_session_us,"); sessionFile_.println(wave_timing.startSessionUs);
  sessionFile_.print("tim_stop_imu_us,");      sessionFile_.println(wave_timing.stopImuUs);
  sessionFile_.print("tim_stop_gps_us,");      sessionFile_.println(wave_timing.stopGpsUs);
  sessionFile_.print("tim_stop_raw_us,");      sessionFile_.println(wave_timing.stopRawUs);
  sessionFile_.print("tim_stop_total_us,");    sessionFile_.println(wave_timing.stopTotalUs);

  // Approximate FIFO budget in microseconds.
  sessionFile_.print("tim_fifo_budget_us,");
  sessionFile_.println((uint32_t)kFifoDepthWords * 1000000UL / kFifoWordsPerSec);
}

// Close the session file and mark the session inactive
void WaveManager::stopSession(void) {
  if (sessionFile_) { sessionFile_.sync(); sessionFile_.close(); }
  sessionActive_ = false;
}

// GPS-file, if open, is written to at the nav rate
void WaveManager::serviceGps(uint32_t relMs) {
  gps_manager.update();
  if (!sessionActive_ || !gpsFile_ || !gps_manager.freshFix()) return;

  // Get (copy of) last position fix from the GPS manager
  const UBX_PVT &f = gps_manager.lastFix();

  // Not interested if we do not have a 3D fix or if f invalid
  if (!f.valid || f.fixType < 3) return;
  
  gpsFile_.print(relMs);                    gpsFile_.print(',');

  // UTC as HHMMSSCC
  gpsFile_.print((uint32_t)f.hour * 1000000UL +
                 (uint32_t)f.minute * 10000UL +
                 (uint32_t)f.second * 100UL); gpsFile_.print(',');

  //iTOW: Interval Time Of Week (milliseconds since start of current GPS week)
  gpsFile_.print(f.iTOW_ms);                gpsFile_.print(','); 

  // Latitude, Longitude, Speed, Velocity N/E/Up, Heading
  gpsFile_.print(f.lat_e7 * 1e-7, 6);       gpsFile_.print(',');
  gpsFile_.print(f.lng_e7 * 1e-7, 6);       gpsFile_.print(',');
  gpsFile_.print(f.gSpeed_mms / 1000.0, 4); gpsFile_.print(',');
  gpsFile_.print(f.velN_mms / 1000.0, 4);   gpsFile_.print(',');
  gpsFile_.print(f.velE_mms / 1000.0, 4);   gpsFile_.print(',');
  gpsFile_.print(-f.velD_mms / 1000.0, 4);  gpsFile_.print(','); //vUp
  gpsFile_.print(f.headMot_e5 * 1e-5, 2);   gpsFile_.print(',');

  // Accuracy (speed (m/s), horizontal (m), vertical (m))
  // PDOP (Position Dilution Of Precision 1 great, 10 poor), Satellites
  gpsFile_.print(f.sAcc_mms / 1000.0, 2);   gpsFile_.print(',');
  gpsFile_.print(f.hAcc_mm / 1000.0, 2);    gpsFile_.print(',');
  gpsFile_.print(f.vAcc_mm / 1000.0, 2);    gpsFile_.print(',');
  gpsFile_.print(f.pDOP_e2 * 0.01, 2);      gpsFile_.print(',');
  gpsFile_.println(f.numSV);

  gpsRowsWritten_++;
}