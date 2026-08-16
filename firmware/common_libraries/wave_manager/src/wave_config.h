#ifndef WAVE_CONFIG_H
#define WAVE_CONFIG_H

#include <Arduino.h>
#include "config.h"

// IMU + wave-analysis config, split by concern
#include "imu_config.h"
#include "analysis_config.h"
#include "log_config.h"

// GPS timeout for start and end location of wave measurement.
static constexpr uint32_t wave_gps_fix_timeout {120000};  // ms

// Whether that timeout aborts the capture or merely ends the wait.
static constexpr bool wave_measurement_require_gps {true};

// Whether the capture loop logs GPS fixes to gps.csv
// Only use when postprocessing of GPS data is needed, as it adds some overhead.
static constexpr bool wave_gps_track_in_capture = true;

#endif  // WAVE_CONFIG_H
