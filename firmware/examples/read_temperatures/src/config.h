#ifndef CONFIG_H
#define CONFIG_H
#include "Arduino.h"

/*
  The console is the mySerial instance in main.cpp (USART1 on PB6/PB7, the debug
  header). The variant's own Serial is an unused LPUART1, so alias it here and
  every Serial.print, in this sketch and in the libraries, lands on the header.
*/
extern HardwareSerial mySerial;
#undef Serial
#define Serial mySerial

static constexpr uint8_t BUOY_MODE {0};
static constexpr uint8_t BST_MODE {1};

static constexpr uint8_t WIO_MODE {BUOY_MODE}; 

// Useful constants
static constexpr uint32_t s_2_ms                         {1000};
static constexpr uint32_t min_2_s                          {60};

// Sensor reading parameters
static constexpr uint8_t  readings_per_measurement            {15};
static constexpr uint8_t  max_number_of_measurements         {80};
static constexpr uint32_t max_sensor_read_time               {40*s_2_ms};
static constexpr float    outlier_discard_tolerance          {2};
static           uint32_t base_measurement_period            {5*min_2_s*s_2_ms};
static constexpr uint32_t scale_factor                       {100000};
static constexpr uint8_t  max_number_of_thermometres          {1};
static constexpr uint32_t thermometre_pause_between_readings  {300};  // ms, DS18B20 conversion wait

// Pin config. Do not change unless you have rewired the ORB
static constexpr uint32_t DEBUG_SERIAL_TX_PIN            {PB6};
static constexpr uint32_t DEBUG_SERIAL_RX_PIN            {PB7};
// SPI1 is shared by the SD card and the IMU, so both chip selects are driven high
static constexpr uint32_t SPI_MOSI_PIN                   {PA7};
static constexpr uint32_t SPI_MISO_PIN                   {PA6};
static constexpr uint32_t SPI_SCK_PIN                    {PA5};
static constexpr uint32_t SPI_CS_SD_PIN                  {PA4};
static constexpr uint32_t SPI_CS_IMU_PIN                 {PB3};

// Enable or disable parameters
static constexpr bool remove_outliers           {true};
static constexpr bool debug_serial              {true};
static constexpr int  serial_baud               {115200};
static constexpr bool debug_SD                  {false};

#endif