#include <Arduino.h>
#include "config.h"
#include "etl_error_manager.h"
#include "messages.h"
#include "lora_transceiver.h"
#include "gps_manager.h"
#include "sd_writer.h"
#include "thermo_manager.h"
#include "wave_manager.h"
#include "IWatchdog.h"
#include "STM32LowPower.h"
#include "TimeLib.h"
/*
  TODO: 
  Get a good algorithm for getting a date stamp from GPS.
  This feature is currently disabled. 
*/

HardwareSerial mySerial{DEBUG_SERIAL_RX_PIN, DEBUG_SERIAL_TX_PIN};


/* Workaround for millis not increasing during sleep cycles */
uint32_t measurement_timer;
uint32_t wave_measurement_timer;
uint32_t beacon_timer = 0;
static uint32_t sleep_cycles_measurement  = 0;
static uint32_t sleep_cycles_wave_measurement = 0;
static uint32_t sleep_cycles_transmission = 0;
static uint32_t sleep_cycles_beacon       = 0;
static uint32_t iterations_counter = 0;


uint32_t millis_time_corrected(uint32_t sleep_cycles){
  return millis() + sleep_cycles*sleep_time;
}

/*
  Jump to the system bootloader in ROM (0x1FFF0000 on STM32WLxx). After the
  jump the ROM bootloader listens on USART1 (PB6/PB7), the same pins as the
  serial console. Follows ST's official recipe: interrupts off, clocks back to
  reset state, NVIC cleared, MSP loaded from the bootloader vector table, then
  jump to its reset handler.
*/
static void jumpToBootloader(void){
  const uint32_t bootAddr = 0x1FFF0000UL;  // system memory, STM32WLE5 (AN2606)
  __disable_irq();
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;
  HAL_RCC_DeInit();
  for (uint8_t i = 0; i < 8; i++){  // disable and clear every NVIC interrupt
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }
  __enable_irq();  // the bootloader needs interrupts for UART reception
  void (*bootJump)(void) = (void (*)(void))(*((uint32_t *)(bootAddr + 4)));
  __set_MSP(*(uint32_t *)bootAddr);
  bootJump();  // never returns
}

/*
  Startup menu: press 'b' in the serial monitor within bootloader_menu_window
  to jump to the UART bootloader and flash without an ST-Link. One dot per
  second shows the window is still open. Runs before the watchdog is started,
  so the wait cannot trigger a reset.
*/
static void bootloaderWindow(void){
  mySerial.print(F("Press 'b' within "));
  mySerial.print(bootloader_menu_window/s_2_ms);
  mySerial.println(F(" s for bootloader mode..."));
  uint32_t windowStart = millis();
  uint32_t lastDot = windowStart;
  while (millis() - windowStart < bootloader_menu_window){
    if (mySerial.available() && mySerial.read() == 'b'){
      mySerial.println(F("\nJumping to bootloader - ready for UART flashing"));
      mySerial.flush();
      delay(50);  // let the line drain before the UART is torn down
      jumpToBootloader();
    }
    if (millis() - lastDot >= s_2_ms){
      mySerial.print(F("."));
      lastDot = millis();
    }
  }
  mySerial.println();
}

void setup() {

  // Serial debugging can be turned on or off with the debug_serial flag.
  // The bootloader menu needs the console too, so it also brings Serial up.
  if (debug_serial || enable_bootloader_menu){
    mySerial.begin(serial_baud);
    delay(100);
  }
  if (debug_serial){
    mySerial.println("Serial initialized!");
    delay(100);
  }

  if (enable_bootloader_menu){
    bootloaderWindow();
  }

  // We try to enable rtc timing
  setSyncInterval(1);

  // Watchdog begin
  if (enable_watchdog){
    IWatchdog.begin(1000*watchdog_wait_time);
  }
  /*
    Note, watchdog has .isReset member which allows us to resume operations in case of 
    shutdown. Look into implementation of restore from SD/RAM
  */

  // Start SD
  bool SD_fail = sd_writer.begin();  
  int8_t status = sd_writer.startLogging("boot_info.txt");
  if (!SD_fail && debug_serial){
    mySerial.println("SD opened");
    mySerial.print("Status code: ");
    mySerial.println(status);
  }
  IWatchdog.reload();


  /*
    Buoy will not start without working radio.
  */
  LORA.beginRadio(LoRa_freq_receive, LoRa_bw, LoRa_sf, LoRa_cr, LoRa_power);
  LORA.getWiOID();
  if (LORA.state == RADIOLIB_ERR_NONE){
    sd_writer.logString("Radio functional");
  } else {
    sd_writer.logString("Radio failed to start. Freezing sensor.");
    while(1);
  }

  if (SD_fail){
    if (debug_serial){
      mySerial.println(F("SD card mount failed. Proceeding with Radio only"));
    }
    delay(100);
  }
  
  /*
    We monitor possible overflow in ETL storage containers
  */
  etl::error_handler::set_callback<etl_error_func>();

#if DEBUG_WAVE_MSG
  /*
    Bench mode: the GPS is never brought up. A unit on a desk gets no fix, so the
    wait-for-fix loop below would spin forever and the message path under test
    would never be reached. Nothing in this build reads a position: the wave
    capture is replaced by a synthetic result and task_measure_gps_temp is not
    called, so timestamps stay boot-relative and that is fine.
  */
  sd_writer.logString("DEBUG_WAVE_MSG: GPS not started");
  if (debug_serial){
    mySerial.println(F("DEBUG_WAVE_MSG: skipping GPS init and fix wait"));
  }
#else
  sd_writer.logString("Beginning GPS");

  IWatchdog.reload();
  gps_manager.begin();

  int8_t  nofix = 1;

  /*
    Buoy will not advance until it has gotten a fix
  */
  while (nofix){
    gps_manager.updateTimestamp(10*max_GPS_read_time, true);
    nofix = gps_manager.performNReadings(1,10*max_GPS_read_time, true);
    delay(100);
    IWatchdog.reload();
  }

  gps_manager.processReadings(false);

  /*
    Set the system clock (RTC/TimeLib) from GPS now that we have a fix. The loop
    above can miss the in-loop clock sync if the module's time-valid flag flips only
    after that iteration's updateTimestamp ran, leaving now() boot-relative (1970)
    and every log/session timestamp wrong. A valid PVT always carries UTC time, so
    set it explicitly here.
  */
  if (gps_manager.updateTimestamp(max_GPS_read_time, true) == 0){
    if (debug_serial){ mySerial.print("RTC set from GPS, epoch "); mySerial.println((uint32_t)now()); }
  } else if (debug_serial){
    mySerial.println("WARNING: RTC not set - no valid GPS time yet (timestamps boot-relative)");
  }
#endif

  /*
    We need to read the attached thermistors for a buoy ID. Kept in the bench build
    too: it is what sets the thermo power pin, so the sleep() at the end of setup
    would otherwise drive an unconfigured pin.
  */
  thermo_manager.begin(THERMO_DATA_PIN, THERMO_POWER_PIN);

  /*
    Bring up the IMU for wave analysis. Runs after sd_writer.begin() (which set up
    the shared SPI1 bus), and degrades gracefully to no wave analysis if the sensor
    does not answer.
  */
  wave_manager.begin();

  // The deployment message is built from the GPS fix, so it has no meaning in the
  // bench build where the receiver is never started.
#if !DEBUG_WAVE_MSG
  gps_manager.getDeploymentMessage(LORA.WiO_ID);
  LORA.startup_timestamp = gps_manager.timestamp;
  IWatchdog.reload();
  if (debug_serial){
    mySerial.println("Sending deployment message");
  }
  if (IWatchdog.isReset() == false && transmitDeploymentMessage){
    /*
      We wait until we get a fix from a base station before proceeding.
      Allows us to make sure the buoy works before tossing it into the water. 
    */
    while (!LORA.connectToBaseStation(max_radio_fix_look_time, max_radio_fix_look_time)) {
      IWatchdog.reload();
    }
    
    LORA.transmitB(gps_manager.deploymentMessage, deployment_message_size);
    sd_writer.logString(gps_manager.deploymentMessage, deployment_message_size);
    LORA.waitUntilReady();
  }
#endif

  LORA.lastTransmission = millis();
  if (IWatchdog.isReset() == true){
    //We check to see if the buoy crashed
    sd_writer.logString("Watchdog crashed during execution of loop. This is a reboot.");
  }


  /*
    Arm both measurement gates so the first loop iteration runs them at once instead
    of waiting out a whole period: a buoy that has just been thrown in the water
    should produce a reading and start its first wave capture immediately, and the
    wave cadence is then counted from that first capture.

    The subtraction is deliberate unsigned wrap-around - the gates compare
    millis_time_corrected() - timer against the period in uint32 modular arithmetic,
    so a timer "before" millis() = 0 gives the correct elapsed time regardless.
  */
  if (measure_immediately_after_deployment){
    measurement_timer      = millis() - LORA.measurement_period - 1;
    wave_measurement_timer = millis() - LORA.measurement_period_wave_analysis - 1;
  } else {
    measurement_timer      = millis();
    wave_measurement_timer = millis();
  }
  if (enable_recovery_beacon){
    beacon_timer = millis();
  }
  /*
    Start low power management. As low power requires all connections 
    to be off, we turn them off here, before
    enabling them again in the loop. 
  */ 
  LowPower.begin();
#if !DEBUG_WAVE_MSG
  gps_manager.shutdownGPS();  // nothing to shut down when begin() never ran
#endif
  sd_writer.closeLog();
  thermo_manager.sleep();
  LORA.sleep();

  
  // We try to recover a GPS measurement:
  //char targetFile[64];
  //sprintf(targetFile, "readings/reading%05d.txt", 3);
  //sd_writer.startReading(targetFile);
  //sd_writer.readFile();
  //gps_manager.getMeasurementFromFile();
  //thermo_manager.getMeasurementFromFile();
  //sd_writer.closeRead();


  if (debug_serial){
    mySerial.println("Setup complete, loop beginning");
    delay(100);
    mySerial.end();
  }


}

void task_measure_gps_temp() {

  // Measure time!
  unsigned long measurement_start = millis();

  // Wake up sensors
  gps_manager.begin();
  thermo_manager.wake();
  IWatchdog.reload();

  char logname[25];
  sprintf(logname, "readings/reading%05d.txt", sd_writer.logCount);
  
  if (!sd_writer.active){
    sd_writer.begin();
  }
  sd_writer.startLogging(logname);
  IWatchdog.reload();

  // We read each sensor
  if (debug_serial){
    mySerial.println("Reading sensors");
    mySerial.println(gps_manager.iterations);
    delay(100);
  }
  
  // success_gps_read is 0 if the buoy got a fix
  uint8_t success_gps_read = gps_manager.updateTimestamp(max_GPS_read_time, resync_RTC_using_GPS);
  if (success_gps_read == 0){
    gps_manager.performNReadings(readings_per_measurement,max_GPS_read_time, log_every_reading);
    gps_manager.processReadings(true);

    // We should turn off the GPS post reading due to power consumption
    gps_manager.shutdownGPS();

    thermo_manager.takeReadings(max_sensor_read_time, gps_manager.timestamp, log_every_reading);
    thermo_manager.processReadings();


    IWatchdog.reload();
    // We update the measurements period to reflect the new motion paradigme
    if (enable_motion_detection){
      LORA.updateMeasurementFrequency(gps_manager.current_buoy_velocity, maximal_measurement_period, minimal_measurement_period);
    }
  } else {
    // If no fix was found, we skip this measurement and hope for better luck the next time around. 
    gps_manager.shutdownGPS();
  }
  // Turning off remaining sensors
  sleep_cycles_measurement = 0;
  measurement_timer = millis_time_corrected(sleep_cycles_measurement);
  thermo_manager.sleep();
  sd_writer.closeLog();
  IWatchdog.reload();
}
  

void task_measure_waves() {

  if (debug_serial){
    mySerial.println("Wave measurement starting");
  }

  /*
    The period is anchored to the START of the capture, not its end: a capture blocks
    for wave_measurement_duration, so resetting the timer afterwards would give a
    cycle of duration + period instead of the period itself. With the anchor here, a
    capture begins every measurement_period_wave_analysis - which is exactly why that
    period must be longer than wave_measurement_duration (static_assert in config.h);
    otherwise the gate is already due when the capture returns and they run
    back-to-back.
  */
  sleep_cycles_wave_measurement = 0;
  wave_measurement_timer = millis();

  // wave_manager owns its own per-capture session directory (imu/gps/ses/spec/ana) and
  // wakes/stops the GPS itself for the drift track, so just ensure the SD card is up;
  // do NOT open a log here - takeReading/processReading manage the session.
  if (!sd_writer.active){
    sd_writer.begin();
  }

  IWatchdog.reload();
  wave_manager.wake();  // restarts the GNSS engine for the drift track

  // Blocking capture over wave_measurement_duration; the watchdog is reloaded inside.
  // Waits up to wave_gps_fix_timeout for a fix first and returns 2 if none arrives -
  // a capture with no position is skipped outright, so nothing was sampled and there
  // is no spectrum to finalise.
  uint8_t wave_status = wave_manager.takeReading();
  IWatchdog.reload();

  if (wave_status == 0){
    // Finalise the Welch spectrum -> wave parameters and enqueue a result for transmit.
    wave_manager.processReading();
    IWatchdog.reload();
  } else if (debug_serial){
    mySerial.print("Wave measurement skipped, takeReading status ");
    mySerial.println(wave_status);
  }

  wave_manager.sleep();
}


void task_transmit() {

  if (debug_serial){
    mySerial.println("Looking for base station");
  }
  
  char logname[30];
  sprintf(logname, "messages/transmission%05d.txt", LORA.msgCounter);

  if (!sd_writer.active){
    sd_writer.begin();
  }

  sd_writer.startLogging(logname);
  IWatchdog.reload();
  LORA.wakeUp();
  sd_writer.logString("Looking for base station");

  // Handshake. NOT bypassed in the bench build: DEBUG_WAVE_MSG fakes only the SOURCE
  // of the wave result, never its journey. Everything from here on - handshake,
  // learned send frequency, sendData, sd_writer - is the production path, which is
  // the only reason a pass here says anything about the firmware that ships.
  if (perform_handshake){
    if (LORA.connectToBaseStation(max_radio_fix_look_time, max_radio_wait_time)){
    sd_writer.logString("Hands shaked");
    } else {
    sd_writer.logString("No base station found");
    }
    delay(send_delay_after_handshake);
  } else {
      // We do not check if a base station is present
      LORA.available = true;
      LORA.listenTime = 60*s_2_ms;
  }

  Message_Data message_data;
  sd_writer.logString("Transmitting data\n--------\n");

  // We dump as much information as possible to the base station
  // In the time period alloted to us
  while (LORA.available && (thermo_manager.temperatures.size() > 0)){

    // GPS data
    if (debug_serial){
        mySerial.print("Sending G: "); mySerial.print(GPS_message_size); mySerial.println(" bytes");
    }
    
    gps_manager.updateTransmitMessage();
    message_data = LORA.sendData(gps_manager.msgB, GPS_message_size, 10000);
    delay(500);

    sd_writer.logByteArray(gps_manager.msgB, GPS_message_size);
    sd_writer.logSignalInfo(message_data.RSSI, message_data.SNR);

    // Thermometer data
    if (debug_serial){ mySerial.print("Sending T: "); mySerial.print(thermo_message_size); mySerial.println(" bytes");}

    thermo_manager.updateTransmitMessage();
    message_data = LORA.sendData(thermo_manager.msgB, thermo_message_size, 10000);
    IWatchdog.reload();

    delay(500);
    sd_writer.logByteArray(thermo_manager.msgB, thermo_message_size);
    sd_writer.logSignalInfo(message_data.RSSI, message_data.SNR);


    delay(200);
    LORA.packet_count++;
    IWatchdog.reload();

    if (debug_serial){
      mySerial.print("Available: ");
      mySerial.println(LORA.available);
      mySerial.print("Num packets left: ");
      mySerial.println(thermo_manager.temperatures.size());
    }

    delay(500);
  }

  // Wave analysis data (if any). Independent of the thermo queue: a wave result
  // rides along opportunistically whenever the base station is still listening.
  while (LORA.available && !wave_manager.wave_analysis_results.empty()){
#if DEBUG_WAVE_MSG
    // BEFORE updateTransmitMessage, which pops the result off the queue. mySerial,
    // not Serial: Serial is never begun on the drifter.
    wave_manager.printPendingResult(mySerial);
#endif

    // Wave statistics (Significant wave height, peak period, etc) message
    size_t wave_len = wave_manager.updateTransmitMessage();
    if (wave_len == 0){
      break;  // nothing serialised, so there is nothing to send or to pop
    }
    if (debug_serial){
      mySerial.print("Sending W: "); mySerial.print(wave_len);
      mySerial.print(" bytes on "); mySerial.print(LORA.current_frequency, 3);
      mySerial.println(" MHz");
    }
    message_data = LORA.sendData(wave_manager.msgB, (uint8_t)wave_len, 10000);
    IWatchdog.reload();

    delay(500);
    sd_writer.logByteArray(wave_manager.msgB, wave_len);
    sd_writer.logSignalInfo(message_data.RSSI, message_data.SNR);
    
    delay(200);
    LORA.packet_count++;
    IWatchdog.reload();

    /*
      The spectrum follows as its own message, paired to the one above by ts_start.
    */
    size_t psd_len = wave_manager.updatePsdTransmitMessage();
    if (psd_len > 0){
      if (debug_serial){
        mySerial.print("Sending P: "); mySerial.print(psd_len);
        mySerial.print(" bytes on "); mySerial.print(LORA.current_frequency, 3);
        mySerial.println(" MHz");
      }

      message_data = LORA.sendData(wave_manager.psdB, (uint8_t)psd_len, 10000);
      IWatchdog.reload();

      delay(500);
      sd_writer.logByteArray(wave_manager.psdB, psd_len);
      sd_writer.logSignalInfo(message_data.RSSI, message_data.SNR);

      delay(200);
      LORA.packet_count++;
      IWatchdog.reload();
    }

    wave_manager.popTransmittedResult();
  }

  // Wrap up transmission, wait for base station instructions
  if (LORA.available){

    LORA.transmitFinished(thermo_manager.temperatures.size()+wave_manager.wave_analysis_results.size());

    if (enable_baseStation_parameter_updates){

      // We listen for new parameters
      LORA.receiveInstructions();

      // Then we listen for which measurements to add back to memory from 
      // SD card - TBD
      //LORA.receiveDesiredMeasrements();
      //sd_writer.fetchRequestedMeasurements(LORA.measurementTargets);
    }
    LORA.updateMeasurementFrequency(gps_manager.current_buoy_velocity, maximal_measurement_period, minimal_measurement_period);
  }
  
  sd_writer.logString("Transmission done");
  sd_writer.closeLog();

  sleep_cycles_transmission = 0;
  LORA.lastTransmission = millis();
  LORA.sleep();

}

void task_beacon(){
    LORA.wakeUp();
    gps_manager.updateBeaconMsg(LORA.WiO_ID);
    LORA.transmitBeaconMessage(gps_manager.beaconMsg, beaconMsgSize);
    sleep_cycles_beacon = 0;
    beacon_timer        = millis();
    LORA.sleep();
}

/*
  Transmit if the radio is due and either queue has something in it
*/
static bool transmit_if_due(void){
  const bool transmission_due =
      millis_time_corrected(sleep_cycles_transmission) - LORA.lastTransmission > minimal_transmission_period;
  const bool have_payload =
      gps_manager.GPSReadings.size() > packet_count_send_treshold
      || !wave_manager.wave_analysis_results.empty();

  if (transmission_due && have_payload){
    task_transmit();
    return true;
  }
  return false;
}

void loop() {
  /*
    We start by checking if we should switch on each sensor
  */
  IWatchdog.reload();
  bool disable_sensors = false;

  if (debug_serial){
    mySerial.begin(serial_baud);
    delay(100);
    mySerial.println("Booting up!");

  }

  // Measurement loop (temperature and GPS). Skipped entirely in the bench build:
  // the GPS was never started, and a wave-message test has no use for either sensor.
#if !DEBUG_WAVE_MSG
  if (millis_time_corrected(sleep_cycles_measurement) - measurement_timer > LORA.measurement_period){
      task_measure_gps_temp();
  }
#endif

  /*
    Early transmission window. Require handshake etc (radio uptime), so use for debugging purposes.
  */
  if (transmit_before_wave_capture){
    if (transmit_if_due() && debug_serial){
      mySerial.println(F("Transmitted before wave capture (transmit_before_wave_capture)"));
    }
  }

  // Wave measurement loop (IMU): independent gate, own enable flag and period.
  //Debug print enable wave analysis and measurement period, and time since last measurement


#if DEBUG_WAVE_MSG
  // Bench mode: no capture at all. A synthetic result is queued as soon as the queue
  // runs dry and then travels the PRODUCTION path (updateTransmitMessage ->
  // LORA.sendData -> sd_writer). enable_wave_analysis is deliberately not consulted -
  // the point is to test the message without configuring the buoy for a measurement
  // campaign. There is no timer: the transmit below empties the queue, so refilling it
  // the moment it is empty gives back-to-back messages at the loop's own pace. One
  // pending result is enough - queueing more would only test the queue.
  if (wave_manager.wave_analysis_results.empty()){
      wave_manager.enqueueFakeResult();
      if (debug_serial){
        mySerial.println(F("DEBUG_WAVE_MSG: queued fake result"));
      }
  }
#else
  if (LORA.enable_wave_analysis &&
      millis_time_corrected(sleep_cycles_wave_measurement) - wave_measurement_timer > LORA.measurement_period_wave_analysis){
      task_measure_waves();
  }
#endif

  // Transmission protocol
  transmit_if_due();

  // Recovery protocol
  if ((millis_time_corrected(sleep_cycles_beacon) - beacon_timer > beacon_ping_period) && (enable_recovery_beacon)){
    task_beacon();
  }

  if (debug_serial){
    mySerial.println("\n Succesfull iteration, starting to log\n");
    mySerial.println("Logging finished");
    mySerial.println(F("Shutting down"));
    delay(100);
    mySerial.end();
  }
  
  // We sleep for "sleep_time" before waking up to refresh the watchdog
  IWatchdog.reload();
  sleep_cycles_measurement++;
  sleep_cycles_wave_measurement++;
  sleep_cycles_transmission++;
  if (sd_writer.active){
    sd_writer.shutdown();
  }
  delay(150);
  LowPower.sleep(sleep_time);
}
