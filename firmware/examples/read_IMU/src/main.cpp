#include <Arduino.h>
#include <SPI.h>
#include <LSM6DSV16XSensor.h>
#include "config.h"
/*
  This sketch showcase the minimum needed to read the onboard IMU (LSM6DSVTR),
  using only the built-in methods of the STM32duino LSM6DSV16X library.

  Accelerometer (mg) and gyroscope (mdps) are polled
  and printed as CSV on the debug header every print_period ms.
  No FIFO, no interrupt and no SD card - see wave_manager for that.
*/

static HardwareSerial mySerial{DEBUG_SERIAL_RX_PIN, DEBUG_SERIAL_TX_PIN};
static LSM6DSV16XSensor imu(&SPI, (int)SPI_CS_IMU_PIN, imu_spi_hz);

static uint32_t print_timer;

void setup() {

  // We start serial output at 115200 baudrate
  mySerial.begin(serial_baud);
  mySerial.println("Serial initialized!");
  delay(100);

  // The SD card shares the SPI bus, so it must be deselected before the IMU is addressed
  pinMode(SPI_CS_SD_PIN, OUTPUT);
  digitalWrite(SPI_CS_SD_PIN, HIGH);

  // Route SPI1 to the pins on this board. Must be set before SPI.begin()
  SPI.setMOSI(SPI_MOSI_PIN);
  SPI.setMISO(SPI_MISO_PIN);
  SPI.setSCLK(SPI_SCK_PIN);
  SPI.begin();

  // begin() configures the chip select and puts the sensor in a known state
  uint8_t id = 0;
  if (imu.begin() != LSM6DSV16X_OK || imu.ReadID(&id) != LSM6DSV16X_OK || id != LSM6DSV16X_ID) {
    mySerial.print("IMU not found, WHO_AM_I = 0x");
    mySerial.println(id, HEX);
    while (true) { delay(1000); }
  }
  mySerial.println("IMU found");

  // Range and data rate, then start both sensors
  imu.Set_X_FS(imu_accel_fs_g);
  imu.Set_G_FS(imu_gyro_fs_dps);
  imu.Set_X_ODR(imu_odr_hz);
  imu.Set_G_ODR(imu_odr_hz);
  imu.Enable_X();
  imu.Enable_G();

  mySerial.println("t_ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps");
  print_timer = millis();
}

void loop() {
  if (millis() - print_timer >= print_period) {
    print_timer += print_period;

    // The library converts to physical units using the configured full scale
    int32_t acc[3], gyr[3];
    imu.Get_X_Axes(acc);
    imu.Get_G_Axes(gyr);

    char line[96];
    snprintf(line, sizeof(line), "%lu,%ld,%ld,%ld,%ld,%ld,%ld",
             (unsigned long)millis(),
             (long)acc[0], (long)acc[1], (long)acc[2],
             (long)gyr[0], (long)gyr[1], (long)gyr[2]);
    mySerial.println(line);
  }
}
