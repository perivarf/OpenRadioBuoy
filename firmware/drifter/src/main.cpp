#include <Arduino.h>
#include "config.h"
#include "etl_error_manager.h"
#include "messages.h"
#include "lora_transceiver.h"
#include "gps_manager.h"
#include "sd_writer.h"
#include "thermo_manager.h"
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
uint32_t beacon_timer = 0;
static uint32_t sleep_cycles_measurement  = 0;
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
    wait-for-fix loop below would spin forever and the wave message under test
    would never be reached.
  */
  sd_writer.logString("DEBUG_WAVE_MSG: GPS not started");
  if (debug_serial){
    mySerial.println(F("DEBUG_WAVE_MSG: skipping GPS init and fix wait"));
  }
#else
  sd_writer.logString("Beginning GPS");

  IWatchdog.reload();
  gps_manager.begin(GPS_baud);

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
#endif

  /*
    We need to read the attached thermistors for a buoy ID
  */
  thermo_manager.begin(THERMO_DATA_PIN, THERMO_POWER_PIN);

#if !DEBUG_WAVE_MSG
  // Skipped on the bench: getDeploymentMessage reads GPSReadings.back(), and
  // without a fix that deque is empty.
  gps_manager.getDeploymentMessage(LORA.WiO_ID);
#endif
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
  
  LORA.lastTransmission = millis();
  if (IWatchdog.isReset() == true){
    //We check to see if the buoy crashed
    sd_writer.logString("Watchdog crashed during execution of loop. This is a reboot.");
  }


  measurement_timer = millis();
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
  gps_manager.shutdownGPS();  // never started in bench mode
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
  gps_manager.begin(GPS_baud);
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


#if DEBUG_WAVE_MSG
/*
  Bench test of the wave-analysis ('W') message. There is no wave_manager on this
  PCB yet, so the whole producer side lives here: a synthetic result, the
  serialiser, and a console dump. The point is to exercise the wire format and
  the base station's parser end to end without an IMU or a wave capture.

  The byte layout below is the contract with
  Message_Parser::parse_wave_analysis_message. Note the on-air field order is
  Hs, Tc, Tp, Tz - NOT the Hs/Tz/Tc/Tp order the base station happens to print
  them in. Getting that wrong is the easiest mistake to make here, which is why
  the test values are deliberately distinct and not round.
*/
/*
  Frequency axis this fake spectrum claims to cover, as BIN CENTRES. The real drifter
  derives these from its Welch settings (segment length and decimated sample rate) and
  ships them in the message; this target has no wave analysis, so the values are
  simply the ones that firmware produces for a 1024-sample segment at 10 Hz, grouped
  two PSD bins per wire bin: df = 2 * 10/1024 = 0.01953125 Hz, first centre at half a
  group above DC.

  They only need to be plausible and self-consistent - the point of the test message
  is to exercise the codec, not to describe a real sea state.
*/
static constexpr float test_spec_f_min = 0.0048828125f;
static constexpr float test_spec_f_max = test_spec_f_min + (welch_bins - 1) * 0.01953125f;

struct TestWaveResult {
  uint16_t reading_ID;
  float    Hs;         // significant wave height (m)
  float    Tc;         // crest period (s)
  float    Tp;         // peak period (s)
  float    Tz;         // zero-crossing period (s)
  float    max_value;  // peak elevation PSD (m^2/Hz)
  uint16_t wave_spectrum[welch_bins];  // normalised to the peak, 0-65535
  time_t   timestamp_start;
  time_t   timestamp_end;
};

static TestWaveResult make_test_wave_result(void){
  static uint16_t test_reading_ID = 0;

  TestWaveResult res{};
  res.reading_ID = ++test_reading_ID;

  // Deliberately not round numbers, and all different from each other: if the
  // fixed-point scaling or the field order is wrong on the receiving side,
  // distinct odd values say so immediately, where 1.0/2.0/3.0 could line up
  // plausibly after a swap.
  res.Hs        = 1.37f;
  res.Tc        = 2.53f;
  res.Tp        = 6.91f;
  res.Tz        = 4.29f;
  res.max_value = 0.0842f;

  // A single smooth peak. The wire value is bin/peak * 65535, so the far side
  // reconstructs the absolute PSD as value/65535 * max_value. The peak sits off
  // centre so a mirrored or off-by-one bin axis is visible at a glance.
  const float peakBin = 0.35f * (float)welch_bins;
  const float width   = 0.12f * (float)welch_bins;
  for (size_t j = 0; j < welch_bins; j++){
    const float d    = ((float)j - peakBin) / width;
    const float norm = expf(-0.5f * d * d);
    res.wave_spectrum[j] = (uint16_t)lroundf(norm * 65535.0f);
  }

  /*
    time_t is unsigned long, and the RTC is never synced in this build (no GPS),
    so now() starts at 0 and counts up from there. Deriving the pair as
    now() - 30 min therefore wraps timestamp_start around to ~4.29e9 for the
    first half hour of runtime. Anchor to a fixed plausible instant instead and
    let now() only advance it, so both ends are always sane and ordered.
  */
  static constexpr time_t test_wave_epoch = 1767225600UL;  // 2026-01-01 00:00:00 UTC
  res.timestamp_start = test_wave_epoch + now();
  res.timestamp_end   = res.timestamp_start + 30*min_2_s;

  return res;
}

/*
  Serialise into msgB as 'W' ... 'E', matching parse_wave_analysis_message.
  Floats go out as fixed-point scaled by scale_factor.
*/
static void build_test_wave_message(byte (&msgB)[wave_message_size], const TestWaveResult &res){
  auto toFixed = [](float v) -> uint32_t {
    if (!(v > 0.0f)) return 0;                       // undefined or negative -> 0
    double scaled = (double)v * (double)scale_factor;
    if (scaled > 4294967295.0) return 0xFFFFFFFFUL;  // clamp to uint32 range
    return (uint32_t)llround(scaled);
  };

  uint8_t offset = 0;
  msgB[offset++] = 'W';
  msg_insert_uint(msgB, res.reading_ID, offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, (uint32_t)res.timestamp_start, offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, (uint32_t)res.timestamp_end,   offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.Hs),        offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.Tc),        offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.Tp),        offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.Tz),        offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFixed(res.max_value), offset, wave_message_size, offset, true);

  // Frequency axis, so the receiver can label the bins it is about to read. Bin
  // centres, and the count immediately before the bins themselves.
  // wave_freq_scale, not toFixed's scale_factor - see readings.h for why the axis
  // needs the finer scale.
  auto toFreqFixed = [](float f) -> uint32_t {
    return (uint32_t)llround((double)f * (double)wave_freq_scale);
  };
  msg_insert_uint(msgB, toFreqFixed(test_spec_f_min), offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, toFreqFixed(test_spec_f_max), offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, (uint16_t)welch_bins,         offset, wave_message_size, offset, true);

  // Last field: it is the only variable-length one, so stopping short here shortens
  // the message without moving anything the receiver has already read.
  for (size_t i = 0; i < welch_bins; i++){
    msg_insert_uint(msgB, res.wave_spectrum[i], offset, wave_message_size, offset, true);
  }
  msgB[offset++] = 'E';
}

/*
  Decode the bytes we just serialised and print them with print_wave_analysis_message
  - the very same function the base station calls on reception - so the two
  consoles can be diffed line for line. If they agree, the codec is right and any
  difference is on the air; if they disagree here, the fault is in the serialiser.

  parse_wave_analysis_message is the base station's own parser too, so this is a
  real round trip through the wire format, not a re-print of the source values.
*/
static void print_test_wave_message(byte (&msgB)[wave_message_size]){
  wave_analysis_Reading w = MESSAGE_PARSER.parse_wave_analysis_message(msgB);
  print_wave_analysis_message(w);

  mySerial.print(F("raw ")); mySerial.print((uint32_t)wave_message_size);
  mySerial.println(F(" bytes:"));
  for (size_t j = 0; j < wave_message_size; j++){
    if (msgB[j] < 0x10) mySerial.print('0');
    mySerial.print(msgB[j], HEX); mySerial.print(' ');
  }
  mySerial.println();
}

/*
  Send one synthetic wave message down the production path: same sendData / SD
  logging pattern as the G and T messages in task_transmit.
*/
void task_test_wave(void){
  TestWaveResult res = make_test_wave_result();
  byte waveMsgB[wave_message_size];
  build_test_wave_message(waveMsgB, res);

  if (debug_serial){
    mySerial.print(F("Sending W: ")); mySerial.print(wave_message_size);
    mySerial.println(F(" bytes"));
    print_test_wave_message(waveMsgB);
  }

  Message_Data message_data = LORA.sendData(waveMsgB, wave_message_size, 10000);
  IWatchdog.reload();
  delay(500);
  sd_writer.logByteArray(waveMsgB, wave_message_size);
  sd_writer.logSignalInfo(message_data.RSSI, message_data.SNR);
  LORA.packet_count++;
  delay(200);
  IWatchdog.reload();
}
#endif  // DEBUG_WAVE_MSG


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

  // Handshake
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
  
#if DEBUG_WAVE_MSG
  // The G/T loop above runs zero rounds on the bench (the thermo queue is empty
  // without task_measure_gps_temp), so this is the only payload of the cycle.
  if (LORA.available){
    task_test_wave();
  }
#endif

  // Wrap up transmission, wait for base station instructions
  if (LORA.available){
    LORA.transmitFinished(thermo_manager.temperatures.size());
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
  measurement_timer = millis();
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

  // Measurement loop (temperature and GPS)
#if !DEBUG_WAVE_MSG
  if (millis_time_corrected(sleep_cycles_measurement) - measurement_timer > LORA.measurement_period){
      task_measure_gps_temp();
  }
#endif

  // Transmission protocol
  bool transmission_due = millis_time_corrected(sleep_cycles_transmission) - LORA.lastTransmission > minimal_transmission_period;
  bool have_payload     = gps_manager.GPSReadings.size() > packet_count_send_treshold;
#if DEBUG_WAVE_MSG
  // The GPS threshold is the production trigger, but a bench unit indoors never
  // gets a fix - so the test wave message has to be reason enough to transmit,
  // or it would never go out at all.
  have_payload = true;
#endif
  if (transmission_due && have_payload) {
        task_transmit();
  }

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
  sleep_cycles_transmission++;
  if (sd_writer.active){
    sd_writer.shutdown();
  }
  delay(150);
  LowPower.sleep(sleep_time);
}
