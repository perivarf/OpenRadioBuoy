#ifndef WAVE_CONFIG_H
#define WAVE_CONFIG_H

#include <Arduino.h>
#include "config.h"

/*
  IMU + wave-analysis tuning, split by concern. This header is the single include
  point - nothing includes the three below directly.

    imu_config.h       the sensor: rates, ranges, filters, FIFO depth and watermark,
                       register addresses, and the drain timing budget everything else
                       is measured against
    analysis_config.h  the rate chain, both FIR stages, the AHRS selection, Welch and
                       the transmitted spectrum
    log_config.h       what a capture writes: log mode, CSV widths, the raw binary
                       format, and the hot-path timing switch

  They chain in that order, each including the one above it, because the derivations
  run the same way: the Welch ring slack needs the FIFO word rate, and the raw log's
  buffer needs the FIFO depth.
*/

#include "imu_config.h"
#include "analysis_config.h"
#include "log_config.h"

// -----------------------------------------------------------------------------
// Capture scheduling
// -----------------------------------------------------------------------------

// How long takeReading() waits for a valid GNSS solution before giving up. Used for the
// fix at each end of the capture. A ceiling, not a cost: waitForGpsFix returns on the
// first fresh solution, which is ~100-200 ms with a lock (GPS_nav_period_ms is 100).
static constexpr uint32_t wave_gps_fix_timeout {120000};  // ms

// Whether that timeout ABORTS the capture or merely ends the wait.
static constexpr bool wave_measurement_require_gps {true};

// Whether the capture LOOP polls the receiver and logs the drift track to gps.csv.
// The positions at each end of the window are taken either way - they come from calls
// outside the loop, and the wave analysis reads nothing else from the GPS.
//
// What this costs when on: 383 us mean and 19.2 ms max per loop iteration, ~12 % of CPU
// over a capture, and gps.csv's ~2 MB reservation. With it off there is no gps.csv at
// all, and cfg.csv's gps_track_in_capture is what tells that apart from a receiver that
// failed.
static constexpr bool wave_gps_track_in_capture = false;

#endif  // WAVE_CONFIG_H
