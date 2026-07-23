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
static constexpr uint8_t  packet_count_send_treshold     {2};
static constexpr int16_t  transmission_grace_period      {5*s_2_ms};
static constexpr uint32_t max_radio_fix_look_time        {90*s_2_ms};
static constexpr uint32_t beacon_ping_period             {2*min_2_s*s_2_ms};
static constexpr uint32_t send_delay_after_handshake     {1000};

// Sensor reading parameters
static constexpr uint8_t  readings_per_measurement            {15};
static constexpr uint8_t  max_number_of_measurements         {40};
static constexpr uint32_t max_GPS_read_time                  {3*min_2_s*s_2_ms};
static constexpr uint32_t max_sensor_read_time               {40*s_2_ms};
static constexpr float    outlier_discard_tolerance          {2};
static constexpr uint16_t GPS_baud                           {9600};
static constexpr uint32_t minimal_measurement_period         {10*s_2_ms};
static           uint32_t base_measurement_period            {10*s_2_ms};
static constexpr uint32_t maximal_measurement_period         {30*min_2_s*s_2_ms};
static constexpr uint32_t thermometre_pause_between_readings {300};


// Enable or disable parameters
static constexpr bool remove_outliers                       {true};
static constexpr bool enable_GPS                            {true};
static constexpr bool debug_SD                              {false};
static constexpr bool transmitDeploymentMessage             {false};
static constexpr bool debug_LED_enabled                     {false};
static constexpr bool sleep_GPS                             {true};
static constexpr bool perform_handshake                     {true};
static constexpr bool enable_baseStation_parameter_updates  {false};
static constexpr bool enable_recovery_beacon                {true};
static constexpr bool log_every_reading                     {true};
static constexpr bool resync_RTC_using_GPS                  {true};
static constexpr bool enable_bootloader_menu                {true};
static constexpr uint32_t bootloader_menu_window            {5*s_2_ms};

// Motion parameters
static bool     enable_motion_detection                  {false};
static float    motion_treshold                          {0.5};
static uint32_t target_reading_distance                  {30};

// Power parameters
static constexpr uint32_t sleep_time                     {9*s_2_ms};
#endif
