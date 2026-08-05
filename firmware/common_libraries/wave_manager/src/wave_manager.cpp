#include "wave_manager.h"

#include <math.h>
#include <TimeLib.h>
#include "IWatchdog.h"
#include "parser_utils.h"
#include "sd_writer.h"

WaveManager wave_manager;
WaveManager *WaveManager::s_self = nullptr;

// imu.csv column contract. Naming convention, so a capture can be read without
// consulting the firmware to find out which orientation a column came from:
//   unsuffixed  = the SELECTED filter (WaveAhrs, or SFLP when wave_use_sflp)
//   _sflp       = the on-chip SFLP game-rotation fusion
//   _fir        = FIR-decimated; without it, the unfiltered value at the same instant
// The filter's NAME belongs in cfg.csv (wave_orientation_name), not in a column name:
// WaveAhrs is a compile-time choice, so "vacc_madgwick" was a lie the moment it was
// set to KalmanAhrs.
//
// RENAMED at wave_build_seq 3 (columns kept their ORDER, only names changed):
//   ax_ned..az_ned -> ax_ned_sflp..az_ned_sflp  (they always were SFLP-rotated)
//   qw..qz         -> qw_sflp..qz_sflp
//   mqw..mqz       -> qw..qz                    (the selected filter)
//   vacc_madgwick  -> vacc
// The absence of "mqw" is what tells a reader it has a build_seq 3+ file; postprocess
// looks columns up by name and branches on exactly that. Note the one hazard this
// rename creates and an append never did: an OLD postprocess reading a NEW capture
// silently takes the selected filter's quaternion for the SFLP one (tilt diagnostics
// only - the vertical accel it uses comes from az_ned*, which it will not find at all).
//
// Semantics changed at wave_build_seq 2: ax..gz and the NED triple are the
// FIR-decimated value at the window centre, not the window mean, and the quaternions
// are delayed to match (see ImuRow). vacc/vacc_sflp are the UNFILTERED values at that
// same instant; vacc_fir/vacc_sflp_fir are the filtered ones. cfg.csv carries
// row_decimation/ahrs_rate_hz so an old and a new capture cannot be confused.
static const char *kImuCsvHeader =
    "win_start_ms,n,ax_mg,ay_mg,az_mg,ax_ned_sflp,ay_ned_sflp,az_ned_sflp,"
    "gx_mdps,gy_mdps,gz_mdps,qw_sflp,qx_sflp,qy_sflp,qz_sflp,braking,"
    "qw,qx,qy,qz,vacc,vacc_sflp,sflp_nan,fifo_ovf,vacc_fir,vacc_sflp_fir";

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
void WaveManager::rowSinkTrampoline(const ImuRow &r) {
  if (s_self) s_self->onRow(r);
}

void WaveManager::onRow(const ImuRow &r) {
  analyzer_.ingest(r);  // the row arrives complete; the analyzer only consumes it
  rowCount_++;

  if (!csvActive_) return;
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
    // truncate() at the current position hands back the clusters pre-allocation
    // reserved but the capture did not use, and sets the directory entry to the real
    // length. Without it every session folder would claim its full reservation and
    // the tail would read as garbage. Safe to call whether or not preAllocate
    // succeeded: with no reservation the position already is the end of the file.
    imuFile_.truncate();  imuFile_.sync();  imuFile_.close();
    gpsFile_.truncate();  gpsFile_.sync();  gpsFile_.close();
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

  // Reserve both streaming files contiguously BEFORE the first byte goes in -
  // preAllocate() refuses once a cluster exists. See wave_config.h for why this is
  // what keeps the FIFO alive. A failure is not fatal: the file simply falls back to
  // growing cluster by cluster, which is the behaviour this replaces, so the capture
  // still runs and only the overflow risk returns.
  const uint32_t durationS = wave_measurement_duration / s_2_ms;
  const uint32_t imuBytes  = (uint32_t)kOutputRateHz * durationS * wave_imu_row_bytes_max;
  const uint32_t gpsBytes  = durationS * wave_gps_row_bytes_max;  // <= 1 row per fix per second
  if (!imuFile_.preAllocate(imuBytes) && debug_serial) {
    Serial.print("WaveManager: imu preAllocate failed, "); Serial.print(imuBytes);
    Serial.println(" B - card may be full or fragmented");
  }
  if (!gpsFile_.preAllocate(gpsBytes) && debug_serial) {
    Serial.println("WaveManager: gps preAllocate failed");
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
  sessionFile_.print("orientation_name,");sessionFile_.println(analyzer_.orientationName());
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
  // AHRS settling window: logged to imu.csv/gps.csv but excluded from Welch/PSD, so
  // postprocess must skip the same leading rows to reproduce the on-device Hs.
  f.print("filter_warm_up_ms,");  f.println(wave_measurement_filter_warm_up);

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
  // fifo_watermark only means something when INT1 drives the drain - it described
  // nothing at all until the interrupt path was ported, so record which one ran.
  f.print("imu_wake,");           f.println(kImuUseInt1 ? "int1_watermark" : "poll");
  f.print("fifo_watermark,");     f.println(kFifoWatermark);

  // --- windowing (raw ODR -> imu.csv rows) ---
  f.print("output_rate_hz,");     f.println(kOutputRateHz);
  f.print("window_ms,");          f.println(kWindowMs);
  f.print("csv_sync_rows,");      f.println(wave_csv_sync_rows);
  // What the imu file reserved up front. Read this next to fifo_ovf in imu.csv: a
  // capture that overflows despite a successful reservation has a stall that is not
  // cluster allocation, which is a different hunt.
  f.print("imu_prealloc_bytes,");
  f.println((uint32_t)kOutputRateHz * (wave_measurement_duration / s_2_ms) *
            wave_imu_row_bytes_max);

  // --- decimation ---
  // row_decimation is the single key that separates a build-1 capture (boxcar means,
  // AHRS stepped on the 100 Hz rows) from a build-2 one (FIR, AHRS on the raw
  // stream). Anything reading these folders should branch on it, not on a date.
  f.print("row_decimation,fir");  f.println();
  f.print("fir_ntap,");           f.println(kFirNtap);
  f.print("fir_s1_cutoff_hz,");   f.println(kFirS1CutoffHz, 3);
  f.print("fir_s2_cutoff_hz,");   f.println(kFirS2CutoffHz, 3);
  f.print("fir_s1_delay_s,");     f.println(kFirS1DelayS, 6);
  f.print("fir_s2_delay_s,");     f.println(kFirS2DelayS, 6);
  f.print("fir_s1_center_ms,");   f.println(kFirS1CenterMs);
  f.print("fir_s2_center_ms,");   f.println(kFirS2CenterMs);
  // The device runs the filters causally, so the logged series lag by the group
  // delays above. Offline comparisons must either shift or use compensate=False.
  f.print("fir_compensate,0");    f.println();

  // --- brake detection ---
  f.print("brake_g_thresh,");     f.println(kBrakeGThreshold, 3);
  f.print("brake_thresh_mg2,");   f.println((float)kBrakeThresholdMg2, 1);
  f.print("brake_min_ms,");       f.println(kBrakeMinMs);
  f.print("brake_min_samples,");  f.println(kBrakeMinSamples);

  // --- orientation / vertical acceleration ---
  // Echoed from ses.csv (same analyzer, so they cannot disagree): without it the
  // tuning below is ambiguous, since only one method actually ran.
  f.print("orientation_name,");   f.println(analyzer_.orientationName());
  // The AHRS runs on the RAW stream now, not on the rows, so the rate it was
  // stepped at is no longer output_rate_hz and has to be recorded separately -
  // offline cannot reproduce the device attitude without it.
  f.print("ahrs_rate_hz,");       f.println(kAhrsRateHz, 2);
  f.print("ahrs_rate_cap_hz,");   f.println(kAhrsRateCapHz);
  // The quaternions are held, not filtered, but they ARE carried back by the stage-1
  // group delay so every column in a row describes one instant. See quat_delay.h.
  f.print("quat_decimation,hold"); f.println();
  f.print("quat_delay_s,");       f.println(kFirS1DelayS, 6);
  f.print("quat_delay_steps,");   f.println(kQuatDelaySteps);
  f.print("madgwick_beta,");      f.println(kMadgwickBeta, 4);
  // The full Kalman tuning, because R is adaptive: r0 alone does not say what the
  // filter did, the lambdas and dt_ref do. Written whichever filter ran, like
  // madgwick_beta above - a capture folder should describe the build, not only
  // the branch it took.
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
  // A wire bin is the average of spec_bin_group PSD bins, so the array length alone
  // no longer determines the frequency axis. These three keys do, and they are the
  // only record of it: f_j = spec_f_min_hz + j * spec_bin_width_hz. Changed meaning
  // at wave_build_seq 2 - spec_f_min/max_hz are now wire-bin CENTRES, and they used
  // to be PSD-bin centres back when the mapping was 1:1.
  f.print("spec_bin_group,");     f.println((uint32_t)kSpecBinGroup);
  f.print("spec_bin_width_hz,");  f.println(kSpecBinWidthHz, 6);
  f.print("spec_f_min_hz,");      f.println(kSpecFMinHz, 5);
  f.print("spec_f_max_hz,");      f.println(kSpecFMaxHz, 5);
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
