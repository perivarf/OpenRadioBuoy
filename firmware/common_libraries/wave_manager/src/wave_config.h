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

// How long takeReading() waits for a valid GNSS solution before giving up.
static constexpr uint32_t wave_gps_fix_timeout {120000};  // ms

// Whether that timeout ABORTS the capture or merely ends the wait.
static constexpr bool wave_measurement_require_gps {true};

#endif  // WAVE_CONFIG_H
