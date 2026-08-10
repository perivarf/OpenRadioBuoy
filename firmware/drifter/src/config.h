#ifndef CONFIG_H
#define CONFIG_H
#include "Arduino.h"

/*
  The console is the Serial instance in main.cpp (USART1 on PB6/PB7). The
  shared libraries print to `Serial`, which on this variant is an unused
  LPUART1 on PA2/PA3, so their output would be lost. Alias it here - config.h
  resolves to each project's own copy, so this does not affect basestation.
*/

// OLB MODE PARAMETERS - DO NOT ADJUST LIGHTLY
static constexpr uint8_t BUOY_MODE {0};
static constexpr uint8_t BST_MODE {1};
static constexpr uint8_t MOORED_MODE {2};
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
static constexpr float    LoRa_bw                        {125.0};
static constexpr uint8_t  LoRa_sf                        {8};
static constexpr uint8_t  LoRa_cr                        {6};
static constexpr uint8_t  LoRa_power                     {15};
static constexpr uint8_t  packet_count_send_treshold     {2};
static constexpr int16_t  transmission_grace_period      {5*s_2_ms};
static constexpr uint32_t max_radio_fix_look_time        {90*s_2_ms};
static constexpr uint32_t max_radio_wait_time            {40*s_2_ms};
static constexpr uint8_t  max_message_length             {255};
static constexpr uint32_t beacon_ping_period             {2*min_2_s*s_2_ms};
static constexpr float    LoRa_freq_beacon               {868};
static constexpr uint32_t send_delay_after_handshake     {1000};

// Sensor reading parameters
static constexpr uint8_t  readings_per_measurement            {15};
static constexpr uint8_t  max_number_of_measurements         {40};
static constexpr uint32_t max_GPS_read_time                  {3*min_2_s*s_2_ms};
static constexpr uint32_t max_sensor_read_time               {40*s_2_ms};
static constexpr float    outlier_discard_tolerance          {2};
static constexpr uint16_t GPS_baud                           {9600};
static constexpr uint8_t  max_number_of_thermometres         {1};
static constexpr uint32_t minimal_measurement_period         {10*s_2_ms};
static           uint32_t base_measurement_period            {10*s_2_ms};
static constexpr uint32_t maximal_measurement_period         {30*min_2_s*s_2_ms};
static constexpr uint32_t scale_factor                       {100000};
/*
  Latitude/longitude keep the receiver's native 1e-7 deg
*/
static constexpr int32_t  gps_coord_scale                    {10000000};
static constexpr uint32_t thermometre_pause_between_readings {300};

/*
  Number of spectrum bins in a wave-analysis ('W') message. This is part of the
  wire contract, not a local tuning knob: readings.h sizes
  wave_analysis_Reading::wave_spectrum and wave_message_size from it, so drifter
  and basestation MUST carry the same value. A mismatch does not fail to
  compile - it silently shifts every field after the spectrum.
  Keep in sync with basestation/src/config.h.
*/
static constexpr size_t welch_bins {51};


// Enable or disable parameters
static constexpr bool remove_outliers                       {true};
static constexpr bool debug_serial                          {true};
static constexpr bool enable_GPS                            {true};
static constexpr bool enable_watchdog                       {true};
static constexpr bool debug_SD                              {false};
static constexpr int  serial_baud                           {115200};
static           bool enable_motion_detection               {false};
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

/*
  Bench tests of the transmit messages. Set from the build, not here: the
  orb_drifter_test_* environments in platformio.ini pass -DDEBUG_WAVE_MSG=1,
  -DDEBUG_GPS_MSG=1 and/or -DDEBUG_TEMP_MSG=1.

  Each flag adds one synthetic message to every transmission cycle - 'W' wave
  analysis, 'G' GPS, 'T' temperature - so the wire format and the base station's
  parser can be exercised indoors, with no fix, no wave capture and no
  thermistor attached. The flags are independent and combine freely;
  orb_drifter_test_msgs turns on all three.

  All default to 0, so the normal orb_drifter build compiles exactly as before.
*/
#ifndef DEBUG_WAVE_MSG
#define DEBUG_WAVE_MSG 0
#endif
#ifndef DEBUG_GPS_MSG
#define DEBUG_GPS_MSG 0
#endif
#ifndef DEBUG_TEMP_MSG
#define DEBUG_TEMP_MSG 0
#endif

/*
  True in any bench-test build. The hardware is skipped per build, not per
  message: a unit on a desk gets no fix, so the wait-for-fix loop in setup()
  would spin forever and the message under test would never be reached - and
  that is just as true for a GPS-only or temperature-only test as for a wave
  one. The handshake and the rest of the transmit path are unchanged.
*/
#define DEBUG_TEST_MSG (DEBUG_WAVE_MSG || DEBUG_GPS_MSG || DEBUG_TEMP_MSG)

/*
  How many transmission cycles send test messages before the bench build goes
  quiet. Once the handshake has succeeded and the base station has acknowledged
  a cycle, the format is proven - repeating it forever only fills the SD card
  and the Notehub queue. Every enabled message type goes out once per cycle.

  0 means no limit, for when you do want a continuous stream (a link-margin
  walk-around, say).
*/
static constexpr uint16_t max_test_transmissions {2};

/*
  Bus wiring on this PCB. Do not change unless you have rewired the ORB.
*/
static constexpr uint32_t SPI_MOSI_PIN                   {PA10};
static constexpr uint32_t SPI_MISO_PIN                   {PB14};
static constexpr uint32_t SPI_SCK_PIN                    {PB13};
static constexpr uint32_t SPI_CS_SD_PIN                  {PB9};
static constexpr uint32_t I2C_SDA_PIN                    {PIN_WIRE_SDA};
static constexpr uint32_t I2C_SCL_PIN                    {PIN_WIRE_SCL};


// Motion parameters
static float    motion_treshold                          {0.5};
static uint32_t target_reading_distance                  {30};

// Watchdog and power parameters
static constexpr uint32_t watchdog_wait_time              {32*s_2_ms};
static constexpr uint32_t sleep_time                      {9*s_2_ms};
#endif