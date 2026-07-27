#ifndef COMMON_CONFIG_H
#define COMMON_CONFIG_H
#include "Arduino.h"

/*
  Shared configuration for both the drifter and the basestation. Each project's
  config.h includes this first, then defines its own project-specific values and
  the constants whose value genuinely differs between the two targets.

  The console is the mySerial instance in main.cpp (USART1 on PB6/PB7, the debug
  header on this PCB). The variant's own `Serial` is an unused LPUART1 on PA2/PA3,
  so alias it here and every existing Serial.print lands on the header. USART1
  supports only this orientation: PB6 is in PinMap_UART_TX, PB7 in PinMap_UART_RX.
  Swapping them leaves the line undriven.
*/
extern HardwareSerial mySerial;
#undef Serial          // WSerial.h has already aliased it to SerialLP1
#define Serial mySerial

static constexpr uint32_t DEBUG_SERIAL_TX_PIN           {PB6};
static constexpr uint32_t DEBUG_SERIAL_RX_PIN           {PB7};

// WiO operating modes
static constexpr uint8_t BUOY_MODE   {0};
static constexpr uint8_t BST_MODE    {1};
static constexpr uint8_t MOORED_MODE {2};

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
static constexpr uint32_t I2C_SDA_PIN    {PA11};
static constexpr uint32_t I2C_SCL_PIN    {PA12};

// Shared radio parameters. LoRa_bw, LoRa_sf and LoRa_cr must match between TX
// and RX for the link to work; the per-device ones (LoRa_power, frequencies)
// live in each config.h.
static constexpr float    LoRa_bw            {125.0};
static constexpr uint8_t  LoRa_sf            {12};
static constexpr uint8_t  LoRa_cr            {6};
static constexpr float    LoRa_freq_beacon   {868.0};
static constexpr uint32_t max_radio_wait_time {40000};

// Point-to-point message buffer size (receiver buffer must be >= the largest
// message either side builds; a wave-analysis message is ~150 bytes)
static constexpr uint8_t max_message_length {255};

// Measurement scaling
static constexpr uint32_t scale_factor              {100000};
static constexpr uint8_t  max_number_of_thermometres {1};

// Wave analysis message wire format: how many uint16 spectrum values a wave message
// carries. This is the whole shared contract - it sizes wave_spectrum[] in
// wave_analysis_Reading, drives the parse loop and fixes wave_message_size.
//
// WHICH frequencies those bins cover is drifter-side physics and lives next to the
// Welch settings in wave_manager/wave_config.h (welch_bin_min/max): a bin index means
// nothing without the segment length and sample rate behind f = k*fs/N, and the base
// station has neither and needs neither. wave_config.h static_asserts its bin range
// against this count, so the two cannot drift apart silently.
static constexpr size_t welch_bins {57};

// Shared wave-analysis run parameters
static constexpr bool     base_enable_wave_analysis           {true};
static constexpr uint32_t base_measurement_period_wave_analysis {0UL * 60UL * 1000UL}; // 0 min between wave captures

// Watchdog
static constexpr bool     enable_watchdog    {true};
static constexpr uint32_t watchdog_wait_time {32000};

// Debug / logging
static constexpr int  serial_baud  {115200};
static constexpr bool debug_serial {true};

#endif
