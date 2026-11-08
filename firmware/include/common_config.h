#ifndef COMMON_CONFIG_H
#define COMMON_CONFIG_H
#include "Arduino.h"

/*
  Shared configuration for the drifter and the basestation; each config.h includes it.

  The console is mySerial (USART1, PB6 TX / PB7 RX, the debug header). The variant's
  Serial is an unused LPUART1, so Serial is aliased to mySerial.
*/
extern HardwareSerial mySerial;
#undef Serial          // WSerial.h has already aliased it to SerialLP1
#define Serial mySerial

static constexpr uint32_t DEBUG_SERIAL_TX_PIN           {PB6};
static constexpr uint32_t DEBUG_SERIAL_RX_PIN           {PB7};

// WiO operating modes
static constexpr uint8_t BUOY_MODE   {0};
static constexpr uint8_t BST_MODE    {1};

/*
  Bus wiring on this PCB (same board for drifter and basestation).

  SPI1 is shared by the SD card and the LSM6DSVTR IMU, so both chip selects must
  be driven high before either slave is addressed.
*/
static constexpr uint32_t SPI_MOSI_PIN   {PA7};
static constexpr uint32_t SPI_MISO_PIN   {PA6};
static constexpr uint32_t SPI_SCK_PIN    {PA5};
static constexpr uint32_t SPI_CS_SD_PIN  {PA4};
static constexpr uint32_t SPI_CS_IMU_PIN {PB3};
// INT1 on the IMU (sensor pin 4). Not part of the SPI bus - the sensor drives it to
// say the FIFO has reached its watermark, so the drain does not have to poll for it.
// Same net as ORB_test's kLsmInt1; the rest of this pinout matches that board too.
// Moves to PB5 in the next ORB board revision - change it here when that board is used.
static constexpr uint32_t INT1_IMU_PIN   {PB12};
static constexpr uint32_t I2C_SDA_PIN    {PA11};
static constexpr uint32_t I2C_SCL_PIN    {PA12};

// Shared radio parameters. LoRa_bw, LoRa_sf and LoRa_cr must match between TX
// and RX for the link to work; the per-device ones (LoRa_power, frequencies)
// live in each config.h.
static constexpr float    LoRa_bw            {125.0};
static constexpr uint8_t  LoRa_sf            {8};
static constexpr uint8_t  LoRa_cr            {6};
static constexpr float    LoRa_freq_beacon   {868.0};
static constexpr uint32_t max_radio_wait_time {40000};

// Point-to-point message buffer size (receiver buffer must be >= the largest
// message either side builds; a wave-analysis message is ~150 bytes)
static constexpr uint8_t max_message_length {255};

// Measurement scaling
static constexpr uint32_t scale_factor              {100000};
static constexpr int32_t gps_coord_scale            {10000000};
static constexpr uint8_t  max_number_of_thermometres {1};

// Wave analysis message wire format: the largest number of uint16 spectrum values a
// wave message can carry so that the message is still less than max_message_length
// Actual bin count is kSpecNBins in drifter, with kSpecNBins<=welch_bins
// and the actual bin count is sent in the wave message to basestation
// In case of this being set to high, a compile time assertion will be through in wave_manager.cpp
static constexpr size_t welch_bins {102};

// Shared wave-analysis run parameters
static constexpr bool     base_enable_wave_analysis           {true};
// Interval between the START of one wave capture and the start of the next, i.e. a
// capture every n'th minute. Must be longer than wave_capture_duration (asserted
// in drifter/src/config.h); the idle time between captures is n - duration.
static constexpr uint32_t base_wave_capture_period            {120UL * 60UL * 1000UL}; // capture starts every 120 min

// Watchdog
static constexpr bool     enable_watchdog    {true};
static constexpr uint32_t watchdog_wait_time {32000};

// Debug / logging
static constexpr int  serial_baud  {115200};
static constexpr bool debug_serial {false};

/*
  Print a received 'P' message one line per bin. Off by default: every print syncs the
  debug file, and the dump blocks the receive loop long enough to miss the next 'W'.
*/
static constexpr bool debug_print_psd_bins {false};

#endif
