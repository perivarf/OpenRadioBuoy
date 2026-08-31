#ifndef CONFIG_H
#define CONFIG_H
#include "common_config.h"   // shared pins, modes, Serial alias, welch, watchdog, ...

// OLB MODE PARAMETERS - DO NOT ADJUST LIGHTLY
static constexpr uint8_t WIO_MODE {BUOY_MODE};

/*
 Useful constants
 Do note, base time unit in arduino is milliseconds.
 These denote where we convert from seconds to milliseconds, or
 minutes to seconds to milliseconds.
*/
static constexpr uint32_t s_2_ms                         {1000};
static constexpr uint32_t min_2_s                          {60};
static constexpr float    mps_2_kmph                       {3.6};


// Radio parameters
static           uint32_t minimal_transmission_period    {2*s_2_ms};
static           float    LoRa_freq_receive              {863};
static constexpr uint8_t  LoRa_power                     {15};

static constexpr uint8_t  packet_count_send_treshold     {0}; // Default 2 -> this is expensive
static constexpr int16_t  transmission_grace_period      {5*s_2_ms};
static constexpr uint32_t max_radio_fix_look_time        {90*s_2_ms};
static constexpr uint32_t beacon_ping_period             {2*min_2_s*s_2_ms};
static constexpr uint32_t send_delay_after_handshake     {1000};

// Sensor reading parameters
static constexpr uint8_t  readings_per_measurement            {15};
static constexpr uint8_t  max_number_of_measurements         {40};
static constexpr uint32_t max_GPS_read_time                  {1*min_2_s*s_2_ms}; //Default 3 min
static constexpr uint32_t max_sensor_read_time               {40*s_2_ms};
static constexpr float    outlier_discard_tolerance          {2};
static constexpr uint16_t GPS_baud                           {9600};
static constexpr uint32_t minimal_measurement_period         {10*s_2_ms}; //Default 10 min
static           uint32_t base_measurement_period            {10*s_2_ms}; //Default 10 min
static constexpr uint32_t maximal_measurement_period         {30*min_2_s*s_2_ms}; //Default 30 min
static constexpr uint32_t thermometre_pause_between_readings {30};


// Enable or disable parameters
static constexpr bool remove_outliers                       {true};
static constexpr bool enable_GPS                            {true};
static constexpr bool debug_SD                              {false};
static constexpr bool transmitDeploymentMessage             {false};
static constexpr bool debug_LED_enabled                     {false};
static constexpr bool sleep_GPS                             {true};
static constexpr bool perform_handshake                     {true}; //TRUE default
static constexpr bool enable_baseStation_parameter_updates  {false};
static constexpr bool enable_recovery_beacon                {true};
static constexpr bool log_every_reading                     {true};
static constexpr bool resync_RTC_using_GPS                  {true};
static constexpr bool enable_bootloader_menu                {true};
static constexpr uint32_t bootloader_menu_window            {5*s_2_ms};

/*
  Take the first GPS/thermistor reading and start the first wave capture on the first
  loop iteration, rather than after one full period of silence. Set false to let both
  gates wait out their period after boot (useful when a bench unit should not start a
  half-hour capture the moment it is powered).
*/
static constexpr bool measure_immediately_after_deployment  {true};

/*
  Run the transmission check before the wave capture as well as after it.

  A capture blocks loop() for wave_measurement_duration - half an hour as configured
  - so with the check only after it, a freshly booted buoy shows no sign of life at
  the base station, and hence on Notehub, until the first capture has finished.
  Checking first as well gets whatever is already queued out the door immediately,
  which is what you want when you are standing next to the buoy wondering whether it
  came up at all.

  Diagnostic convenience, not a measurement requirement: it costs one extra handshake
  per loop whenever something is queued (connectToBaseStation is up to
  max_radio_fix_look_time + max_radio_wait_time). Set false for a deployment where
  radio power matters more than seeing the buoy quickly.
*/
static constexpr bool transmit_before_wave_capture          {true};

// Motion parameters
static bool     enable_motion_detection                  {false};
static float    motion_treshold                          {0.5};
static uint32_t target_reading_distance                  {30};

// -----------------------------------------------------------------------------
// Wave capture scheduling (the between-captures period + enable flag are shared and
// live in common_config.h; this is the length of one blocking capture window).
// -----------------------------------------------------------------------------
static constexpr uint8_t  max_number_of_wave_measurements {5};
static constexpr uint32_t wave_measurement_duration       {30*min_2_s*s_2_ms}; // 30 min capture

// AHRS settling time at the start of a capture. The filter (Madgwick/Kalman) starts
// from a single accel sample and needs a while to converge on the true orientation;
// the vertical acceleration it produces before then is biased and would pollute the
// PSD. Rows inside the warm-up are still fed to the filter (that is the point) and
// still logged to imu.csv/gps.csv - they are only kept out of the 10 Hz bucketing,
// the Welch accumulation and hence Hs/Tz/Tc/Tp. Set to 0 to disable.
static constexpr uint32_t wave_measurement_filter_warm_up {30*s_2_ms};
static_assert(wave_measurement_filter_warm_up < wave_measurement_duration,
              "AHRS warm-up must be shorter than the capture, or nothing is analysed");

/*
  base_measurement_period_wave_analysis is the interval between the START of one
  capture and the start of the next (task_measure_waves anchors its timer up front),
  so it has to leave room for the capture itself. If it does not, the gate is already
  due when the capture returns and the buoy captures back-to-back with no GPS/temp
  measurement or transmission in between.
*/
static_assert(base_measurement_period_wave_analysis > wave_measurement_duration,
              "Wave capture period must exceed wave_measurement_duration, or captures run back-to-back");

// Power parameters
static constexpr uint32_t sleep_time                     {9*s_2_ms};
#endif
