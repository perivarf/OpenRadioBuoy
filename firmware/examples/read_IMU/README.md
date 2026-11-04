# Read IMU example
This code example is a minimal starting point for the onboard IMU (LSM6DSVTR). It uses nothing but the built-in methods of the
[STM32duino LSM6DSV16X](https://github.com/stm32duino/LSM6DSV16X) library, and is self-contained from the other OLB libraries.
The accelerometer and gyroscope are polled and printed as CSV on the debug header (USART1, PB6/PB7, 115200 baud):

```
t_ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps
```

At rest one axis should read about ±1000 mg and the gyro close to 0. There is no FIFO, interrupt or SD card logging here.
For continuous, gap-free sampling at a high rate, see the FIFO-based [```wave_manager```](../../common_libraries/wave_manager/).

Build and flash from this folder with ```pio run -t upload```, and open the console with ```pio device monitor```.
The script can be tuned with the ```config.h``` file that can be found in the ```src``` directory.

## Configuration parameters

| Parameter | Default value | Explanation |
| --------- | ------------- | ----------- |
| ```imu_spi_hz```      | 8000000 | SPI clock in Hz. The sensor is rated for max 10 MHz |
| ```imu_odr_hz```      | 120     | Output data rate of both accelerometer and gyroscope, in Hz |
| ```imu_accel_fs_g```  | 4       | Accelerometer full scale, ±2/4/8/16 g |
| ```imu_gyro_fs_dps``` | 500     | Gyroscope full scale, ±125/250/500/1000/2000/4000 dps |
| ```print_period```    | 100     | Time in ms between each printed reading |
| ```serial_baud```     | 115200  | Baud rate of the console |

The pin assignments (SPI1 shared with the SD card, IMU chip select on PB3) match the ORB board, and should only be changed if the board is rewired.
