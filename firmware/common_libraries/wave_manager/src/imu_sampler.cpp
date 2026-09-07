#include "imu_sampler.h"
#include "config.h"
#include "rotation.h"
#include <math.h>

// -----------------------------------------------------------------------------
// ImuSampler
// -----------------------------------------------------------------------------
void ImuSampler::resetWindowing(uint32_t captureStartMs) {
  sessionStartMs_ = captureStartMs;
  nOverflowTotal_ = 0;       // per-capture, reported in ana.csv
  nOverflow_ = 0;
  lastDrainMs_ = sessionStartMs_;
  logStarted_ = false;
  accelIdx_ = 0;
  sampleTms_ = 0.0;
  samplePeriodMs_ = 1000.0 / kImuOdrHz;
  curWinIdx_ = -1;
  winNAcc_ = 0;
  winBraking_ = false;
  winSflpNan_ = false;
  winFifoOvf_ = false;
  winFirDone_ = false;
  brakeRun_ = 0;
  latestQw_ = 1; latestQx_ = latestQy_ = latestQz_ = 0;
  latestGx_ = latestGy_ = latestGz_ = 0;

  
  ahrs_.reset(); // Orientation filter reset (is initialized from the first accel sample). 
  ahrsSeeded_ = false;
  fir_.reset();
  
  // Debug counters
  dbgLastPrint_ = captureStartMs;
  nAccDbg_ = nGyrDbg_ = 0;
  dbgGpsPrev_ = 0;  // matches WaveManager::gpsRowsWritten_, which is zeroed per session
  nUnknownDbg_ = 0;
  lastUnknownTag_ = 0;
}

// Evaluate the decimation filters and read the delay-matched quaternions into the
// row being assembled (to imu.csv / input to FIR Stage 2). 
// Everything refers to the centre of the current window
void ImuSampler::latchRowValues() {

  // Time the FIR evaluation and the quaternion reads
  WAVE_TIME(TIM_FIR);
  
  fir_.eval(pendingRow_); // Update pendingRow_ with the FIR-decimated values

  float mq[4], sq[4]; //mq = AHRS quaternion, sq = SFLP quaternion
  qDelay_.read(mq, sq); // Read from delay-line to get the quaternions corresponding to the center of the current window

  // Setting the quaternions in the pending row
  pendingRow_.qw = mq[0]; pendingRow_.qx = mq[1];
  pendingRow_.qy = mq[2]; pendingRow_.qz = mq[3];
  pendingRow_.qwSflp = sq[0]; pendingRow_.qxSflp = sq[1];
  pendingRow_.qySflp = sq[2]; pendingRow_.qzSflp = sq[3];

  // If any of the filtered values are not finite, set them to 0.0f
  if (!isfinite(pendingRow_.vaccFir)) pendingRow_.vaccFir = 0.0f;
  if (!isfinite(pendingRow_.vaccSflpFir)) pendingRow_.vaccSflpFir = 0.0f;
  if (!isfinite(pendingRow_.vacc)) pendingRow_.vacc = 0.0f;
  if (!isfinite(pendingRow_.vaccSflp)) pendingRow_.vaccSflp = 0.0f;

  winFirDone_ = true;

} // Scope end for TIM_FIR

// Close the current window -> finish the ImuRow and hand it to the row sink.
void ImuSampler::closeWindow() {
  
  if (winNAcc_ == 0) return;
  
  // No raw sample reached the window centre - a FIFO gap. Read the filters at the
  // window edge instead of dropping the row: the filtered value is still valid, it
  // is just centred up to one row period late. Counted so a capture can be judged.
  if (!winFirDone_) {
    latchRowValues();
    nFirLateEval_++;
  }
  
  // Window statistics
  pendingRow_.winStartMs = rowStartMs((uint32_t)curWinIdx_); // Derived to avoid drift due to float additions
  pendingRow_.n = winNAcc_;
  pendingRow_.braking = winBraking_ ? 1 : 0;
  pendingRow_.sflpNan = winSflpNan_ ? 1 : 0;
  pendingRow_.fifoOvf = winFifoOvf_ ? 1 : 0;

  // The sink is the analyzer plus (optionally) the imu.csv row. 
  const uint32_t tSink = timeStart();
  rowSink_(pendingRow_);
  timeAdd(TIM_ROWSINK, tSink);
}

// Drain all pending FIFO words. 
// Accel, gyro and the rotation quaternion arrive interleave.
// The accel sample is used as the clock, driving windowing, the AHRS and decimation filters.
// Every delay line advances exactly once per accel sample
// Gyro and quaternion words only latch their latest value for the accel to pair with.
void ImuSampler::update(Print &dbg, uint32_t captureLeftMs, uint32_t gpsRows) {

  // Watermark: If used, but not triggered, return
  if (kImuUseInt1 && !dev_.fifoReady()) {
    debugPrintStatus(dbg, captureLeftMs, gpsRows);
    return;
  }

  // If not using watermark, check the drain deadline. If not reached, return.
  if (!kImuUseInt1 && (millis() - lastDrainMs_) < kDrainIntervalMs) {
    debugPrintStatus(dbg, captureLeftMs, gpsRows);
    return;
  }

  // Draining everything, so reset fifo flag
  dev_.clearFifoReady();

  // For timing purposes, setting start of drain
  const uint32_t tStatus = timeStart();

  // IMU status and flags (from one register read)
  const ImuFifoStatus st = dev_.status();
  timeAdd(TIM_STATUS, tStatus);

  // TIM_UPDATE measures drains
  WAVE_TIME(TIM_UPDATE);

  // Loss is OVR, not FULL
  //
  // pendingOvrLatched_ belongs to the PREVIOUS drain's pop loop (latched..), so it is reported one
  // drain late. The loss is real, so we report it.
  const bool lost = st.ovr || st.ovrLatched || pendingOvrLatched_;
  pendingOvrLatched_ = false;
  if (lost) {
    nOverflow_++;       // reset by the debug print; nOverflowTotal_ is the per-capture one
    nOverflowTotal_++;
    winFifoOvf_ = true; // flag the window that was open when it happened (fifo_ovf in imu.csv)
  }

  // Add sync record to rawlog (timestamp for when we synced)
  // => the words (acc/gyro/sflp) that follow (until next sync record) belongs to the same window
  if (rawLog_) {
    rawLog_->emitSync((uint32_t)(sampleTms_ * 1000.0), accelIdx_,
                      st.level, lost ? kRawFlagFifoOvf : 0);
  }

  popFifo(st.level);
  finishDrain();

  // Debug print status
  debugPrintStatus(dbg, captureLeftMs, gpsRows);
}

// Drain nSamples words from the FIFO, dispatching each by kind.
void ImuSampler::popFifo(uint16_t nSamples) {

  // TIM_POP spans the whole loop
  // TIM_SPI inside it isolates the bus from the work done on the words
  //,The difference between them is what the maths costs (AHRS, FIR etc)
  const uint32_t tPop = timeStart();

  dev_.resetBurst();   // nothing carries over between drains, but keeping this as a safeguard

  for (uint16_t i = 0; i < nSamples; i++) {

    // TIM_SPI wraps the WHOLE per-word cost, burst refill and decoding included, so n
    // stays the word count and the mean stays comparable across builds. Its max is a
    // whole burst (~320 us at 32 words), not one word.
    ImuFifoWord w;
    const uint32_t tSpi = timeStart();

    // Pop one word. popWord() automatically burst reads.
    dev_.popWord(w, (uint16_t)(nSamples - i));

    // Time SPI read and extraction
    timeAdd(TIM_SPI, tSpi);

    // Everything is written to the raw log (but only periodically to the SD card)
    if (rawLog_) rawLog_->emitWord(w.tag, w.payload);

    // If acceleration: Decode word and feed to AHRS toghether with the latest gyro word
    if (w.kind == ImuSampleKind::Accel) {  // mg
      processAccelWord(w);
    } else if (w.kind == ImuSampleKind::Gyro) {  // mdps
      const float *g = w.v;
      nGyrDbg_++;
      latestGx_ = g[0]; latestGy_ = g[1]; latestGz_ = g[2];
    } else if (w.kind == ImuSampleKind::Quat) {  // [x,y,z,w]
      const float *q = w.v;

      // NaN guard: a corrupt FIFO word decodes to an invalid half-float and the
      // w = sqrt(1 - sumsq) reconstruction yields NaN; one NaN poisons az_ned_sflp and the
      // whole SFLP spectrum. Keep the last valid quaternion instead.
      if (isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) && isfinite(q[3])) {
        latestQx_ = q[0]; latestQy_ = q[1]; latestQz_ = q[2]; latestQw_ = q[3];
      } else {
        winSflpNan_ = true;
      }
    } else {
      nUnknownDbg_++;
      lastUnknownTag_ = w.tag;
    }
  }

  // Time the whole drain
  timeAdd(TIM_POP, tPop);
}

// One accel FIFO word: windowing, AHRS, brake flag, FIR push.
void ImuSampler::processAccelWord(const ImuFifoWord &w) {
  const float *a = w.v;
  nAccDbg_++;   // debug print counter
  uint32_t tms = (uint32_t)sampleTms_;
  int32_t widx = (int32_t)rowIndexAt(tms);

  // Capture start
  if (!logStarted_) { logStarted_ = true; curWinIdx_ = widx; }

  // If the window index has changed, close the previous window and start a new one
  if (widx != curWinIdx_) {
    closeWindow();
    curWinIdx_ = widx;
    winNAcc_ = 0;
    winBraking_ = false;
    winSflpNan_ = false;
    winFifoOvf_ = false;
    winFirDone_ = false;
  }

  winNAcc_++;

  // Extracting from array
  const float ax = (float)a[0], ay = (float)a[1], az = (float)a[2];

  // ---- AHRS on the raw stream ----
  // Accel and gyro arrive as separate FIFO tags, so the update is paired with
  // the freshest gyro word: at most one sample of skew
  const float axS = ax * kMg2Ms2, ayS = ay * kMg2Ms2, azS = az * kMg2Ms2;
  const float sflpQ[4] = {latestQw_, latestQx_, latestQy_, latestQz_};

  // AHRS is initialized from gravity on the first accel sample.
  if (!ahrsSeeded_) {
    ahrs_.initFromAccel(axS, ayS, azS);
    ahrsSeeded_ = true;
    qDelay_.reset(ahrs_.quaternion(), sflpQ);
  } else {

    // One update per raw sample. dt comes from samplePeriodMs_,
    // which self-calibrates against the wall clock
    const uint32_t tAhrs = timeStart();

    ahrs_.update(latestGx_ * kMdps2Rads, latestGy_ * kMdps2Rads, latestGz_ * kMdps2Rads,
                 axS, ayS, azS, (float)samplePeriodMs_ * 1.0e-3f);
    timeAdd(TIM_AHRS, tAhrs);

    // The attitude is not filtered, but it is carried back by the same group
    // delay the FIR imposes on ax..gz, so the whole row describes one instant.
    // At one step per sample that delay is kFirHalf pushes exactly.
    qDelay_.push(ahrs_.quaternion(), sflpQ);
  }

  // Gravity frame linear accel from the SFLP quaternion, gravity removed.
  float wX = 0.0f, wY = 0.0f, wZ = 0.0f;
  if (kEnableSflp) {
    float w[3];
    rotateBodyToWorld(sflpQ, ax, ay, az, w);
    wX = w[0]; wY = w[1]; wZ = w[2] - 1000.0f;   // remove 1 g, mg units
  }

  // Brake flag: LINEAR accel (gravity removed) must stay over
  // threshold for kBrakeMinSamples in a row.

  float bX = wX, bY = wY, bZ = wZ;
  if (!wave_use_sflp) {
    float b[3];
    rotateBodyToWorld(ahrs_.quaternion(), ax, ay, az, b);
    bX = b[0]; bY = b[1]; bZ = b[2] - 1000.0f;
  }

  const double aMag2 = (double)bX * bX + (double)bY * bY + (double)bZ * bZ;

  // If linear accel magnitude exceeds threshold for kBrakeMinSamples in a row, the window is flagged as braking.
  if (aMag2 > kBrakeThresholdMg2) {
    if (brakeRun_ < 0xFFFF) brakeRun_++;
    if (brakeRun_ >= kBrakeMinSamples) winBraking_ = true;
  } else {
    brakeRun_ = 0;
  }

  // The vertical accel is recomputed on every raw sample from the latest
  // quaternion
  const float vacc = wave_use_sflp
                         ? (wZ * kMg2Ms2)
                         : verticalAccel(ahrs_.quaternion(), axS, ayS, azS, kGravity);

  // Adding acceleration (ax, ay, az - sensor frame), acceleration (wZ, wY, wZ - SFLP gravity frame),
  // gyro (latestGx... - sensor frame) and vertical accel (vacc - gravity frame) to the FIR decimation filters.
  fir_.push(ax, ay, az, wX, wY, wZ,
            latestGx_, latestGy_, latestGz_, vacc);

  // Output sample: the first raw sample to reach the window centre.
  // The value sits in the middle of the window it represents.
  // The centre falls between raw samples (and, at 120 Hz, between whole ms), so the
  // residual jitter is at most half a raw period.
  if (!winFirDone_ && tms >= rowCenterMs((uint32_t)curWinIdx_)) {
    latchRowValues();
  }

  sampleTms_ += samplePeriodMs_;
  accelIdx_++;
}

// Post-drain bookkeeping: recalibrate the sample period, flush the raw log.
void ImuSampler::finishDrain() {

  // Calibrate the accel sample period from real elapsed time / total samples, so
  // the time axis tracks the wall clock and the last sample lands on real elapsed time.
  // TODO -> Should have been from previous FIFO drain?
  if (accelIdx_ > 0) {
    samplePeriodMs_ = (double)(millis() - sessionStartMs_) / (double)accelIdx_;
  }

  // Updating last drain
  lastDrainMs_ = millis();

  // FIFO just been emptied, so flushing the raw log where it is least likely
  // to overflow the FIFO buffer
  if (rawLog_) {

    // Timer start
    const uint32_t tFlush = timeStart();
    const uint16_t wroteBytes = rawLog_->flush();
    if (wroteBytes > 0) {

      // Timer end
      timeAdd(TIM_FLUSH, tFlush);

      // Add number of bytes written to the wave_timing stats.
      wave_timing.addFlushBytes(wroteBytes);
    }
  }
}

// Print the effective accel/gyro sample rate + mean magnitudes
void ImuSampler::debugPrintStatus(Print &dbg, uint32_t captureLeftMs, uint32_t gpsRows) {
  if (!debug_serial || imu_debug_print_period == 0) return;

  // Return if the print period has not elapsed since the last print
  uint32_t now = millis();
  if (now - dbgLastPrint_ < imu_debug_print_period) return;

  // Timing the debugprint also
  const uint32_t tDbg = timeStart();

  double elapsedS = (now - dbgLastPrint_) / 1000.0;
  double accHz = elapsedS > 0 ? nAccDbg_ / elapsedS : 0.0;
  double gyrHz = elapsedS > 0 ? nGyrDbg_ / elapsedS : 0.0;

  dbg.print("[IMU] ");              dbg.print(captureLeftMs / 1000UL);
  dbg.print(" s left  |  Accel: "); dbg.print(accHz, 0);
  dbg.print(" Hz, Gyro: ");         dbg.print(gyrHz, 0);

  // GPS effective rate (if logged)
  if constexpr (wave_gps_track_in_capture) {
    const uint32_t dGps = gpsRows - dbgGpsPrev_;
    dbgGpsPrev_ = gpsRows;
    dbg.print(" Hz, GPS: ");
    dbg.print(elapsedS > 0 ? dGps / elapsedS : 0.0, 1);
  }

  if (nUnknownDbg_ > 0) {
    dbg.print("  [WARN] unk "); dbg.print(nUnknownDbg_);
    dbg.print(" (tag 0x"); dbg.print(lastUnknownTag_, HEX); dbg.print(")");
  }
  
  if (nOverflow_ > 0) {
    dbg.print("  [WARN] FIFO overrun x"); dbg.print(nOverflow_);
    nOverflow_ = 0;
  }
  
  dbg.println();
  nUnknownDbg_ = 0;
  nAccDbg_ = nGyrDbg_ = 0;
  dbgLastPrint_ = now;

  if (wave_timing_enabled) {
    /*
      Every bucket as n and mean/max in microseconds
      For instance: 
      update 10x 100/200us  
      status 10x 50/100us  
      pop 10x 500/800us ...
    */
    for (uint8_t i = 0; i < TIM_COUNT; i++) {
      const TimeStat &s = wave_timing.b[i];
      if (i % 4 == 0) dbg.print(i == 0 ? "[TIM] " : "\n      ");
      dbg.print(kTimingNames[i]); dbg.print(' ');
      dbg.print(s.n);             dbg.print("x ");
      dbg.print(s.meanUs());      dbg.print('/');
      dbg.print(s.maxUs);         dbg.print("us  ");
    }
    dbg.println();

    // The raw log's write cost per byte
    const TimeStat &flush = wave_timing.b[TIM_FLUSH];
    dbg.print("      raw ");   dbg.print(wave_timing.flushBytes / 1024.0, 1);
    dbg.print(" kB");
    if (wave_timing.flushBytes > 0) {
      dbg.print(" at ");
      dbg.print((float)flush.sumUs / (float)wave_timing.flushBytes, 2);
      dbg.print(" us/B");
    }
    dbg.println();

    // Reset the window,
    wave_timing.resetInterval();

    // Timing of debugprint - end
    timeAdd(TIM_DBGPRINT, tDbg);
  }
}
