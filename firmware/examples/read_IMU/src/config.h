#ifndef CONFIG_H
#define CONFIG_H
#include "Arduino.h"

// Pin config. Do not change unless you have rewired the ORB
// Console on the debug header (USART1). The variant's own Serial is an unused LPUART1.
static constexpr uint32_t DEBUG_SERIAL_TX_PIN            {PB6};
static constexpr uint32_t DEBUG_SERIAL_RX_PIN            {PB7};
// SPI1 is shared by the SD card and the IMU, so the SD chip select is held high
static constexpr uint32_t SPI_MOSI_PIN                   {PA7};
static constexpr uint32_t SPI_MISO_PIN                   {PA6};
static constexpr uint32_t SPI_SCK_PIN                    {PA5};
static constexpr uint32_t SPI_CS_SD_PIN                  {PA4};
static constexpr uint32_t SPI_CS_IMU_PIN                 {PB3};

// IMU parameters
static constexpr uint32_t imu_spi_hz                     {8000000};  // sensor is rated max 10 MHz
static constexpr float    imu_odr_hz                     {120};      // output data rate, accel and gyro
static constexpr int32_t  imu_accel_fs_g                 {4};        // +-2/4/8/16 g
static constexpr int32_t  imu_gyro_fs_dps                {500};      // +-125/250/500/1000/2000/4000 dps
static constexpr uint32_t print_period                   {100};      // ms between printed readings

static constexpr int      serial_baud                    {115200};
#endif
