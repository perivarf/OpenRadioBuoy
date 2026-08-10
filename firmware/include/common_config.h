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
static constexpr uint32_t base_measurement_period_wave_analysis {0UL * 60UL * 1000UL}; // 0 min between wave captures

// Watchdog
static constexpr bool     enable_watchdog    {true};
static constexpr uint32_t watchdog_wait_time {32000};

// Debug / logging
static constexpr int  serial_baud  {115200};
static constexpr bool debug_serial {true};

#endif
