#include "wave_manager.h"

#include <math.h>
#include <TimeLib.h>
#include "IWatchdog.h"
#include "parser_utils.h"
#include "sd_writer.h"

WaveManager wave_manager;
WaveManager *WaveManager::s_self = nullptr;

// imu.csv column contract, identical to ORB_test Logger so postprocess.py can read
// captures back offline.
static const char *kImuCsvHeader =
    "win_start_ms,n,ax_mg,ay_mg,az_mg,ax_ned,ay_ned,az_ned,gx_mdps,gy_mdps,gz_mdps,"
    "qw,qx,qy,qz,braking,mqw,mqx,mqy,mqz,vacc_madgwick,vacc_sflp,sflp_nan";

void WaveManager::begin(void) {
  s_self = this;
  imu_.setRowSink(&WaveManager::rowSinkTrampoline);
  imuOk_ = imu_.begin(Serial);
  if (imuOk_) {
    imu_.resetFifo();
  } else if (debug_serial) {
    Serial.println("WaveManager: IMU init failed - wave analysis disabled");
  }
}

void WaveManager::wake(void) {
  if (imuOk_) imu_.resetFifo();
}

void WaveManager::sleep(void) {
  // The LSM6DSV keeps streaming into its FIFO; nothing to actively power down here.
  // Between captures the FIFO is simply left to overwrite (reset at next capture).
}

// -----------------------------------------------------------------------------
// Row sink: analyse the window, then (optionally) append it to imu.csv.
// -----------------------------------------------------------------------------
void WaveManager::rowSinkTrampoline(ImuRow &r) {
  if (s_self) s_self->onRow(r);
}

void WaveManager::onRow(ImuRow &r) {
  analyzer_.ingest(r);  // fills r.mq*/r.vacc* before we log the row
  rowCount_++;

  if (!csvActive_) return;
  imuFile_.print(r.winStartMs); imuFile_.print(',');
  imuFile_.print(r.n);          imuFile_.print(',');
  imuFile_.print(r.ax, 3); imuFile_.print(','); imuFile_.print(r.ay, 3); imuFile_.print(','); imuFile_.print(r.az, 3); imuFile_.print(',');
  imuFile_.print(r.axn, 3); imuFile_.print(','); imuFile_.print(r.ayn, 3); imuFile_.print(','); imuFile_.print(r.azn, 3); imuFile_.print(',');
  imuFile_.print(r.gx, 3); imuFile_.print(','); imuFile_.print(r.gy, 3); imuFile_.print(','); imuFile_.print(r.gz, 3); imuFile_.print(',');
  imuFile_.print(r.qw, 5); imuFile_.print(','); imuFile_.print(r.qx, 5); imuFile_.print(','); imuFile_.print(r.qy, 5); imuFile_.print(','); imuFile_.print(r.qz, 5); imuFile_.print(',');
  imuFile_.print(r.braking); imuFile_.print(',');
  imuFile_.print(r.mqw, 5); imuFile_.print(','); imuFile_.print(r.mqx, 5); imuFile_.print(','); imuFile_.print(r.mqy, 5); imuFile_.print(','); imuFile_.print(r.mqz, 5); imuFile_.print(',');
  imuFile_.print(r.vaccMadgwick, 5); imuFile_.print(','); imuFile_.print(r.vaccSflp, 5); imuFile_.print(',');
  imuFile_.println(r.sflpNan);

  if (++rowsSinceSync_ >= wave_csv_sync_rows) {
    imuFile_.sync();  // periodic flush; NOT per row (that would stall the FIFO)
    rowsSinceSync_ = 0;
  }
}

// -----------------------------------------------------------------------------
// Capture: stream the IMU FIFO for wave_measurement_duration.
// -----------------------------------------------------------------------------
uint8_t WaveManager::takeReading(void) {
  if (!imuOk_) return 1;

  readingID_++;
  rowCount_ = 0;
  gpsRowsWritten_ = 0;
  captureStart_ = now();

  analyzer_.begin();

  // Open the session directory (imu/gps/ses/cfg) BEFORE starting the FIFO stream: the
  // mkdir + opening 4 files + writing headers/anchor/config + syncs take tens of ms of SD
  // activity, and if the FIFO were already streaming it would overflow before the
  // first drain. csvActive_ spans take+process: spec/ana are added and the folder
  // renamed (_tmp -> final) in processReading -> stopSession.
  csvActive_ = false;
  if (wave_log_csv && sd_writer.active) {
    csvActive_ = startSession();
  }

  IWatchdog.reload();

  // Init done: start the FIFO stream and drain it immediately (no overflow window).
  imu_.resetWindowing(millis());
  imu_.resetFifo();
  imu_.startStreaming();

  uint32_t start = millis();
  while (millis() - start < wave_measurement_duration) {
    imu_.update(Serial);            // drain the IMU FIFO (must stay tight)
    serviceGps(millis() - start);   // non-blocking GPS poll -> one gps.csv row per fix
    IWatchdog.reload();
    delay(2);  // let the FIFO refill; keeps the drain loop from spinning hot
  }

  if (csvActive_) {
    imuFile_.sync();  imuFile_.close();
    gpsFile_.sync();  gpsFile_.close();
    // sessionFile_ stays open: summary + rename happen in processReading.
  }
  captureEnd_ = now();
  return 0;
}

// -----------------------------------------------------------------------------
// Session logging (ORB_test Logger style): one timestamped directory per capture,
// created as "<stamp>_tmp" and renamed to "<stamp>" on a clean stop.
// -----------------------------------------------------------------------------
bool WaveManager::startSession(void) {
  SdFat &card = sd_writer.card();

  // Stamp from the RTC (set from GPS in setup). Without a valid clock this falls
  // back to the 1970 epoch stamp; readingID_ in the anchor still disambiguates.
  sprintf(logStamp_, "%04d%02d%02d_%02d%02d%02d",
          year(captureStart_), month(captureStart_), day(captureStart_),
          hour(captureStart_), minute(captureStart_), second(captureStart_));
  snprintf(sessionDir_, sizeof(sessionDir_), "%s/%s_tmp", wave_log_dir, logStamp_);

  // mkdir creates the missing "waves/" parent too.
  if (!card.exists(sessionDir_) && !card.mkdir(sessionDir_)) {
    if (debug_serial) { Serial.print("WaveManager: mkdir failed "); Serial.println(sessionDir_); }
    return false;
  }

  char nm[64];
  snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_IMU_PREFIX);
  imuFile_     = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_GPS_PREFIX);
  gpsFile_     = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_SESSION_PREFIX);
  sessionFile_ = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  if (!imuFile_ || !gpsFile_ || !sessionFile_) {
    if (debug_serial) Serial.println("WaveManager: could not open session files");
    return false;
  }

  imuFile_.println(kImuCsvHeader);
  gpsFile_.println("rel_ms,utc_epoch,lat_e7,lng_e7,gspeed_mms,head_e5,hacc_mm,fix,sats");
  sessionFile_.println("key,value");
  imuFile_.sync(); gpsFile_.sync();
  writeSessionAnchor();  // anchor keys (sessionFile_ kept open)

  // cfg.csv: write-once and close - the constants never change during a capture.
  // A failure here is not fatal: the run is still valid, only less self-describing.
  snprintf(nm, sizeof(nm), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_CFG_PREFIX);
  File cfg = card.open(nm, O_RDWR | O_CREAT | O_TRUNC);
  if (cfg) {
    writeSessionConfig(cfg);
    cfg.sync(); cfg.close();
  } else if (debug_serial) {
    Serial.println("WaveManager: could not open cfg.csv");
  }

  rowsSinceSync_ = 0;
  if (debug_serial) { Serial.print("WaveManager: logging session to "); Serial.println(sessionDir_); }
  return true;
}

// The anchor (build + start time) is written up front so it survives on disk even
// if the capture is interrupted before stopSession. imu/gps use a relative time
// base (ms from start), so these keys alone tie t=0 to real UTC.
void WaveManager::writeSessionAnchor(void) {
  if (!sessionFile_) return;
  char iso[24];
  sprintf(iso, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          year(captureStart_), month(captureStart_), day(captureStart_),
          hour(captureStart_), minute(captureStart_), second(captureStart_));
  sessionFile_.print("build_seq,");       sessionFile_.println(wave_build_seq);
  sessionFile_.print("reading_id,");      sessionFile_.println(readingID_);
  sessionFile_.print("orientation,");     sessionFile_.println((int)wave_orientation_method);
  sessionFile_.print("orientation_name,");sessionFile_.println(orientationName(wave_orientation_method));
  sessionFile_.print("start_utc_epoch,"); sessionFile_.println((uint32_t)captureStart_);
  sessionFile_.print("start_utc_iso,");   sessionFile_.println(iso);
  sessionFile_.sync();  // do not close: summary is appended at stop
}

// cfg.csv: every constant the capture depends on, so a session folder is
// self-describing - postprocess.py can rebuild the exact pipeline (scaling,
// bucketing, Welch, taper) from the folder alone, without matching it against a
// firmware revision. Kept out of ses.csv because these are per-BUILD values, while
// ses.csv holds the per-RUN anchor + summary; mixing them buried the ~12 lines you
// actually want to eyeball under ~50 lines of constants.
//
// Written and closed inside startSession (before the FIFO stream starts, so the SD
// activity cannot starve the drain loop) and never reopened - so it costs no
// long-lived File handle, which matters with RAM at ~87%.
void WaveManager::writeSessionConfig(File &f) {
  f.println("key,value");

  // --- capture scheduling ---
  f.print("duration_ms,");        f.println(wave_measurement_duration);
  f.print("period_ms,");          f.println(base_measurement_period_wave_analysis);

  // --- IMU front end ---
  f.print("imu_odr_hz,");         f.println(kImuOdrHz);
  f.print("accel_odr_hz,");       f.println(kAccelOdrHz);
  f.print("imu_low_power,");      f.println(kImuLowPower);
  f.print("imu_acc_mode,");       f.println((int)kImuAccMode);
  f.print("imu_gyr_mode,");       f.println((int)kImuGyrMode);
  f.print("imu_spi_hz,");         f.println(kImuSpiHz);
  f.print("accel_fs_g,");         f.println((int)kAccelFS);
  f.print("gyro_fs_dps,");        f.println((int)kGyroFS);
  f.print("acc_sens_mg_lsb,");    f.println(kAccSensMgPerLsb, 4);
  f.print("gyr_sens_mdps_lsb,");  f.println(kGyrSensMdpsPerLsb, 4);
  f.print("lpf2_enabled,");       f.println(kUseLpf2 ? 1 : 0);
  f.print("lpf2_bw,");            f.println(kLpf2Bw);        // raw CTRL8 register value
  f.print("lpf2_odr_div,");       f.println(kLpf2Div);
  f.print("lpf2_cutoff_hz,");     f.println(kLpf2CutoffHz, 2);
  f.print("sflp_odr_hz,");        f.println(kSflpOdrHz, 1);
  f.print("sflp_game_rot_tag,");  f.println(kSflpGameRotationTag);
  f.print("fifo_watermark,");     f.println(kFifoWatermark);

  // --- windowing (raw ODR -> imu.csv rows) ---
  f.print("output_rate_hz,");     f.println(kOutputRateHz);
  f.print("window_ms,");          f.println(kWindowMs);
  f.print("csv_sync_rows,");      f.println(wave_csv_sync_rows);

  // --- brake detection ---
  f.print("brake_g_thresh,");     f.println(kBrakeGThreshold, 3);
  f.print("brake_thresh_mg2,");   f.println((float)kBrakeThresholdMg2, 1);
  f.print("brake_min_ms,");       f.println(kBrakeMinMs);
  f.print("brake_min_samples,");  f.println(kBrakeMinSamples);

  // --- orientation / vertical acceleration ---
  // orientation is echoed from ses.csv (same constexpr, so they cannot disagree):
  // without it the tuning below is ambiguous, since only one method actually ran.
  f.print("orientation,");        f.println((int)wave_orientation_method);
  f.print("orientation_name,");   f.println(orientationName(wave_orientation_method));
  f.print("madgwick_beta,");      f.println(kMadgwickBeta, 4);
  f.print("kalman_q_angle,");     f.println(kKalmanQAngle, 6);
  f.print("kalman_q_bias,");      f.println(kKalmanQBias, 6);
  f.print("kalman_r,");           f.println(kKalmanR, 6);
  f.print("gravity,");            f.println(kGravity, 5);
  f.print("mg_to_ms2,");          f.println(kMg2Ms2, 8);
  f.print("mdps_to_rads,");       f.println(kMdps2Rads, 8);
  f.print("vacc_bucket_ms,");     f.println(kVacc10HzBucketMs);
  f.print("vacc_fs_hz,");         f.println(kVaccFsHz, 3);

  // --- Welch spectrum + acc->elevation taper ---
  f.print("welch_seglen,");       f.println(kWelchSegLen);
  f.print("welch_overlap_div,");  f.println(kWelchOverlapDiv);
  f.print("welch_step,");         f.println(kWelchSegLen / kWelchOverlapDiv);
  f.print("welch_window,");       f.println(kWelchWindow == WindowType::Hann ? "Hann" : "Hamming");
  f.print("psd_df_hz,");          f.println(kPsdDfHz, 6);
  f.print("wave_fmax_hz,");       f.println(kWaveFMax, 3);
  f.print("taper_f1_hz,");        f.println(kTaperF1, 3);
  f.print("taper_f2_hz,");        f.println(kTaperF2, 3);

  // --- transmitted spectrum slice ---
  // The bin range is drifter-side, so these keys are the only record of which
  // frequencies the base station's wave_spectrum[] actually covers.
  f.print("welch_bin_min,");      f.println((uint32_t)welch_bin_min);
  f.print("welch_bin_max,");      f.println((uint32_t)welch_bin_max);
  f.print("welch_bins,");         f.println((uint32_t)welch_bins);
  f.print("spec_f_min_hz,");      f.println(welch_bin_min * kPsdDfHz, 5);
  f.print("spec_f_max_hz,");      f.println((welch_bin_max - 1) * kPsdDfHz, 5);
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
  sessionFile_.sync();
}

// Close the session file and drop the "_tmp" suffix (rename to the final folder).
// Only reached after a full capture completed, so a leftover "_tmp" directory
// always marks an interrupted/crashed capture.
void WaveManager::stopSession(void) {
  if (sessionFile_) { sessionFile_.sync(); sessionFile_.close(); }
  csvActive_ = false;
  size_t n = strlen(sessionDir_);
  if (n <= 4) return;  // no "_tmp" suffix to strip
  char finalDir[40];
  strncpy(finalDir, sessionDir_, n - 4);
  finalDir[n - 4] = '\0';
  SdFat &card = sd_writer.card();
  if (card.exists(finalDir)) return;  // name clash (same-second capture): keep _tmp
  if (!card.rename(sessionDir_, finalDir) && debug_serial) {
    Serial.print("WaveManager: rename failed "); Serial.println(sessionDir_);
  }
}

// GPS drift track: drive the non-blocking poll and append one gps.csv row per fresh
// fix. update() never blocks long enough to starve the IMU FIFO (see gps_manager).
void WaveManager::serviceGps(uint32_t relMs) {
  gps_manager.update();
  if (!csvActive_ || !gpsFile_ || !gps_manager.freshFix()) return;
  const UBX_PVT &f = gps_manager.lastFix();
  gpsFile_.print(relMs);            gpsFile_.print(',');
  gpsFile_.print((uint32_t)now());  gpsFile_.print(',');  // RTC UTC epoch
  gpsFile_.print(f.lat_e7);         gpsFile_.print(',');
  gpsFile_.print(f.lng_e7);         gpsFile_.print(',');
  gpsFile_.print(f.gSpeed_mms);     gpsFile_.print(',');
  gpsFile_.print(f.headMot_e5);     gpsFile_.print(',');
  gpsFile_.print(f.hAcc_mm);        gpsFile_.print(',');
  gpsFile_.print(f.fixType);        gpsFile_.print(',');
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

  bool ok = analyzer_.finalize(params, res.wave_spectrum);

  res.Hs = params.hs;
  res.Tc = params.tc;
  res.Tp = params.tp;
  res.Tz = params.tz;
  res.max_value = params.maxValue;

  // spec.csv + ana.csv into the same session directory as imu/gps/ses, then the
  // session summary + folder rename. Only when startSession succeeded (csvActive_).
  if (csvActive_ && sd_writer.active) {
    char name[64];
    snprintf(name, sizeof(name), "%s/%s_%s.csv", sessionDir_, logStamp_, WAVE_SPEC_PREFIX);
    File sf = sd_writer.card().open(name, O_RDWR | O_CREAT | O_TRUNC);
    if (sf) {
      sf.println("f_hz,psd_acc,psd_eta");
      const int N = kWelchSegLen;
      const float df = kVaccFsHz / N;
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
      af.print("brake_windows,");    af.println(analyzer_.brakeRows());
      af.print("vacc10hz_samples,"); af.println(analyzer_.samples10Hz());
      af.print("welch_segments,");   af.println(analyzer_.segments());
      af.print("welch_seglen,");     af.println((int)kWelchSegLen);
      af.print("Hs,"); af.println(params.hs, 3);
      af.print("Tz,"); af.println(params.tz, 2);
      af.print("Tc,"); af.println(params.tc, 2);
      af.print("Tp,"); af.println(params.tp, 2);
      af.sync(); af.close();
    }

    // Close out the session file (anchor + summary) and rename the folder.
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
// Serialise the front result into msgB ('W' ... 'E'), matching
// parse_wave_analysis_message. Floats are fixed-point scaled by scale_factor.
// -----------------------------------------------------------------------------
size_t WaveManager::updateTransmitMessage(void) {
  if (wave_analysis_results.empty()) return 0;
  WaveResult res = wave_analysis_results.front();

  auto toFixed = [](float v) -> uint32_t {
    if (!(v > 0.0f)) return 0;                     // undefined (-1) or negative -> 0
    double scaled = (double)v * (double)scale_factor;
    if (scaled > 4294967295.0) return 0xFFFFFFFFUL;  // clamp to uint32 range
    return (uint32_t)llround(scaled);
  };

  uint8_t offset = 0;
  msgB[offset++] = 'W';
  msg_insert_uint(msgB, res.reading_ID, offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.Hs),        offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.Tc),        offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.Tp),        offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.Tz),        offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.max_value), offset, wave_message_size, offset, true);
  for (size_t i = 0; i < welch_bins; i++) {
    msg_insert_uint(msgB, res.wave_spectrum[i], offset, wave_message_size, offset, true);
  }
  msg_insert_uint(msgB, (uint32_t)res.timestamp_start, offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, (uint32_t)res.timestamp_end,   offset, wave_message_size, offset, true);
  msgB[offset++] = 'E';

  wave_analysis_results.pop_front();
  return offset;
}
