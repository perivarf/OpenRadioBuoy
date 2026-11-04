# ORB firmware
The firmware for the ORB is different applications bulit around a set of sometimes shared libraries. In fact, each application folder, currently ```drifter``` and ```basestation``` each only consist of an unique ```main.cpp``` and ```config.h```. All modules that relate to one or more component is located in the ```common_libraries``` folder, and is included in the applications through the ```platformio.ini``` file.

## Common modules
The current modules are, with descriptions and  listed dependencies (with the exception of the excellent [Embedded Template Library (ETL)](https://www.etlcpp.com/) which is omitted due to it being a dependency for all the modules more or less)

| Module | Description | Dependencies |
| ------ | ----------- | ------------ |
| [```etl_error_manager```](common_libraries/etl_error_manager/) | Here we specify how ETL is supposed to handle different errors | None |
| [```gps_manager```](common_libraries/gps_manager/) | Handling of the onboard GPS and its measurements | [```etl_error_manager```](common_libraries/etl_error_manager/), [```stats```](common_libraries/stats/), [```sd_writer```](common_libraries/sd_writer/), [```TinyGPSPlus```](https://github.com/mikalhart/TinyGPSPlus), [```parser_utils```](common_libraries/parser_utils/)  |
| [```gsm```](common_libraries/gsm/) | All operations when it comes to the GSM module on the base station. Do note all supported message structures are defined in the [```message_tools```](common_libraries/message_tools/). | [```etl_error_manager```](common_libraries/etl_error_manager/), [```sd_writer```](common_libraries/sd_writer/), [```NoteCard.h```](https://github.com/blues/note-c) |
| [```lora_transceiver```](common_libraries/lora_transceiver/) | Radio communication manager. Functionality for both buoy and BST. | [```etl_error_manager```](common_libraries/etl_error_manager/), [```sd_writer```](common_libraries/sd_writer/), [```message_tools```](common_libraries/message_tools/), [```parser_utils```](common_libraries/parser_utils/), [```RadioLib```](https://github.com/jgromes/RadioLib)  |
| [```message_tools```](common_libraries/message_tools/) | Data structures and parsing tools for the different kind of ORB LoRa messages | None |
| [```parser_utils```](common_libraries/parser_utils/) | Templates for inserting and extracting integers from byte arrays | None | 
| [```sd_writer```](common_libraries/sd_writer/) | All operations when it comes to file management and file writing to the onboard sd card | [```etl_error_manager```](common_libraries/etl_error_manager/), [```SdFat```](https://github.com/greiman/SdFat) |
| [```stats```](common_libraries/stats/) | Measurement filtering and post-processing | None | 
| [```thermo_manager```](common_libraries/thermo_manager/) | Library to manage everything related to the onboard thermometres. More or less a compatability port of the excellent [OpenMetBuoy thermistor library](https://github.com/jerabaul29/OpenMetBuoy-v2021a/blob/main/legacy_firmware/firmware/plain_gps_drifter/thermistors_manager.h) | [```stats```](common_libraries/stats/), [```sd_writer```](common_libraries/sd_writer/), [```parser_utils```](common_libraries/parser_utils/), [```OneWire```](https://github.com/PaulStoffregen/OneWire) | 
| [```wave_manager```](common_libraries/wave_manager/) | IMU sampling and wave analysis on the drifter. Streams the LSM6DSVTR FIFO, estimates vertical acceleration with an AHRS (the on-chip SFLP fusion, or Madgwick or Kalman in software, chosen at compile time), and computes the wave spectrum (Welch) and sea state parameters (Hs, Tz, Tc, Tp). Logs each capture to the SD card, and builds the wave message sent to the base station | [```sd_writer```](common_libraries/sd_writer/), [```gps_manager```](common_libraries/gps_manager/), [```STM32duino LSM6DSV16X```](https://github.com/stm32duino/LSM6DSV16X), [```SdFat```](https://github.com/greiman/SdFat) |

## The Drifter firmware
The drifter platformio project is the full software package used by the ORB drifters, and the configuration file is currently set in a default configuration for a drifter where power usage is a priority. A parameter table is given in the drifter folder README

## Building
All applications are environments in the [```platformio.ini```](platformio.ini) in this folder, and are built from here:

| Environment | Description |
| ----------- | ----------- |
| ```orb_drifter``` | The drifter firmware |
| ```orb_drifter_test_wave``` | The drifter firmware with ```DEBUG_WAVE_MSG``` enabled, for testing the wave message |
| ```orb_basestation``` | The base station firmware |

E.g. ```pio run -e orb_drifter -t upload```. The build also writes a ```.hex``` file (```compile_to_hex.py```), which can be flashed over UART with ```upload_hex.sh```.

## Examples
The [```examples```](examples/) folder holds small, separate PlatformIO projects showing how to use a single component. Each is built and flashed from its own folder.

| Example | Description |
| ------- | ----------- |
| [```read_GPS```](examples/read_GPS/) | Read and process GPS data with ```gps_manager```, logging to the SD card |
| [```read_IMU```](examples/read_IMU/) | Minimal polling of the IMU accelerometer and gyroscope, using only the built-in methods of the LSM6DSV16X library |
| [```read_temperatures```](examples/read_temperatures/) | Read the thermistors with ```thermo_manager``` |

## Tools
Post-processing scripts for wave captures are found in [```../tools```](../tools/). [```raw_to_csv.py```](../tools/raw_to_csv.py) decodes a capture's ```<stamp>_raw.bin``` to CSV (IMU samples, and the GPS track as ```<stamp>_gps.csv``` when logged).
