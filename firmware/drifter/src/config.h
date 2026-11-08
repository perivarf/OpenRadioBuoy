#ifndef CONFIG_H
#define CONFIG_H
#include "common_config.h"   // shared pins, modes, Serial alias, welch, watchdog, ...

// ORB MODE PARAMETERS - DO NOT ADJUST LIGHTLY
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
static           uint32_t minimal_transmission_period    {5*min_2_s*s_2_ms};
static           float    LoRa_freq_receive              {863};
static constexpr uint8_t  LoRa_power                     {15};

static constexpr uint8_t  packet_count_send_treshold     {2}; // Default 2 -> this is expensive
static constexpr int16_t  transmission_grace_period      {5*s_2_ms};
static constexpr uint32_t max_radio_fix_look_time        {90*s_2_ms};
static constexpr uint32_t beacon_ping_period             {2*min_2_s*s_2_ms};
static constexpr uint32_t send_delay_after_handshake     {1000};

// Sensor reading parameters
static constexpr uint8_t  readings_per_measurement            {15};
static constexpr uint8_t  max_number_of_measurements         {40};
static constexpr uint32_t max_GPS_read_time                  {3*min_2_s*s_2_ms}; //Default 3 min
static constexpr uint32_t max_sensor_read_time               {40*s_2_ms};
static constexpr float    outlier_discard_tolerance          {2};
static constexpr uint16_t GPS_baud                           {9600};
static constexpr uint32_t minimal_measurement_period         {10*s_2_ms}; //Default 10 min
static           uint32_t base_measurement_period            {10*min_2_s*s_2_ms}; //Default 10 min
static constexpr uint32_t maximal_measurement_period         {30*min_2_s*s_2_ms}; //Default 30 min
static constexpr uint32_t thermometre_pause_between_readings {300};


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
  Take the first readings and start the first wave capture on the first loop iteration
  instead of waiting one period. Set false on the bench to avoid an immediate capture.
*/
static constexpr bool measure_immediately_after_deployment  {true};

/*
  Also run the transmission check before the wave capture, so a freshly booted buoy
  shows up at the base station without waiting for the first capture to finish.
  Costs one extra handshake per capture when something is queued.
*/
static constexpr bool transmit_before_wave_capture          {false};

// Motion parameters
static bool     enable_motion_detection                  {false};
static float    motion_treshold                          {0.5};
static uint32_t target_reading_distance                  {30};

// -----------------------------------------------------------------------------
// Wave capture scheduling (the between-captures period + enable flag are shared and
// live in common_config.h; this is the length of one blocking capture window).
// -----------------------------------------------------------------------------
static constexpr uint8_t  max_number_of_wave_measurements {5};
static constexpr uint32_t wave_capture_duration           {1230*s_2_ms};       // 1230 s: 30 s warm-up + 1200 s (20 min) analysed

// AHRS settling time at the start of a capture. Rows in the warm-up feed the filter and
// are logged, but are kept out of the Welch PSD and Hs/Tz/Tc/Tp. 0 disables.
static constexpr uint32_t wave_capture_warmup             {30*s_2_ms};
static_assert(wave_capture_warmup < wave_capture_duration,
              "AHRS warm-up must be shorter than the capture, or nothing is analysed");

/*
  base_wave_capture_period runs from the start of one capture to the start of the
  next, so it must be longer than the capture itself.
*/
static_assert(base_wave_capture_period > wave_capture_duration,
              "Wave capture period must exceed wave_capture_duration, or captures run back-to-back");

// Power parameters
static constexpr uint32_t sleep_time                     {9*s_2_ms};
#endif
