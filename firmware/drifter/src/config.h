#ifndef CONFIG_H
#define CONFIG_H
#include "Arduino.h"

/*
  The console is the mySerial instance in main.cpp (USART1 on PB6/PB7). The
  shared libraries print to `Serial`, which on this variant is an unused
  LPUART1 on PA2/PA3, so their output would be lost. Alias it here - config.h
  resolves to each project's own copy, so this does not affect basestation.
*/
extern HardwareSerial mySerial;
#undef Serial          // WSerial.h has already aliased it to SerialLP1
#define Serial mySerial


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
static constexpr uint8_t  max_message_length             {128};
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
static constexpr uint32_t thermometre_pause_between_readings {300};


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

// Debug console on USART1, the header pins on this PCB. The ROM bootloader
// listens on the same pair, so the menu and the flashing use one cable.
// USART1 supports only this orientation: PB6 is in PinMap_UART_TX, PB7 in
// PinMap_UART_RX. Swapping them leaves the line undriven.
static constexpr uint32_t DEBUG_SERIAL_TX_PIN               {PB6};
static constexpr uint32_t DEBUG_SERIAL_RX_PIN               {PB7};

/*
  Bus wiring on this PCB. Do not change unless you have rewired the ORB.

  SPI1 is shared by the SD card and the LSM6DSVTR IMU, so both chip selects
  must be driven high before either slave is addressed.
*/
static constexpr uint32_t SPI_MOSI_PIN                   {PA7};
static constexpr uint32_t SPI_MISO_PIN                   {PA6};
static constexpr uint32_t SPI_SCK_PIN                    {PA5};
static constexpr uint32_t SPI_CS_SD_PIN                  {PA4};
static constexpr uint32_t SPI_CS_IMU_PIN                 {PB3};
static constexpr uint32_t I2C_SDA_PIN                    {PA11};
static constexpr uint32_t I2C_SCL_PIN                    {PA12};

// Motion parameters
static float    motion_treshold                          {0.5};
static uint32_t target_reading_distance                  {30};

// Watchdog and power parameters
static constexpr uint32_t watchdog_wait_time              {32*s_2_ms};
static constexpr uint32_t sleep_time                      {9*s_2_ms};
#endif