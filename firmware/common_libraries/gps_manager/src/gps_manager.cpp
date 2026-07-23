#include "gps_manager.h"

/*
  Little endian helpers for UBX payloads. The module always sends little
  endian regardless of host byte order, so the bytes are assembled by hand.
*/
static int32_t ubxI4(const uint8_t * p){
  return (int32_t) ((uint32_t) p[0] | ((uint32_t) p[1] << 8) \
                    | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24));
}

static uint32_t ubxU4(const uint8_t * p){
  return (uint32_t) p[0] | ((uint32_t) p[1] << 8) \
         | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

static uint16_t ubxU2(const uint8_t * p){
  return (uint16_t) (p[0] | (p[1] << 8));
}

/*
  Send a UBX message (class/id + payload) over the DDC port, appending the
  two byte Fletcher checksum. The buffer only has to hold the small CFG
  messages we send during startup.
*/
static void sendUbx(uint8_t cls, uint8_t id, const uint8_t * payload, uint16_t len){
  uint8_t buf[32];
  buf[0] = 0xB5; buf[1] = 0x62; buf[2] = cls; buf[3] = id;
  buf[4] = (uint8_t) (len & 0xFF); buf[5] = (uint8_t) (len >> 8);
  for (uint16_t i = 0; i < len; i++){
    buf[6 + i] = payload[i];
  }
  uint8_t ckA = 0, ckB = 0;
  for (uint16_t i = 2; i < 6 + len; i++){
    ckA += buf[i];
    ckB += ckA;
  }
  buf[6 + len] = ckA;
  buf[7 + len] = ckB;
  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write(buf, 8 + len);
  Wire.endTransmission();
}

// Pre-built NAV-PVT poll (empty payload), checksum included.
static const uint8_t navPvtPoll[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};

void GPS_Manager::begin(float baudrate){
  /*
    The GPS talks UBX over I2C, so the baudrate argument is unused. It is kept
    so the call sites in main.cpp stay the same as for the old UART module.

    We bring up the bus, verify the module answers a MON-VER poll, and raise
    its navigation rate. Without the CFG-RATE the M8 only computes a solution
    at 1 Hz and repeated polls would return the same fix over and over.
  */
  (void) baudrate;

  /*
    TEMP DIAGNOSTIC (ported from ORB_test): check the bus before init. Both
    lines must rest HIGH (external pull-ups to VCC). A LOW here means a line is
    held down - bad wiring, missing pull-up, or an unpowered module - and no
    address can ever ACK. Must be read BEFORE Wire.begin() claims the pins.
  */
  if (debug_serial){
    pinMode(sda_pin, INPUT);
    pinMode(scl_pin, INPUT);
    delay(10);
    Serial.print(F("GPS: SDA rest "));
    Serial.print(digitalRead(sda_pin) ? F("HIGH ok") : F("LOW <-- held down!"));
    Serial.print(F(", SCL rest "));
    Serial.println(digitalRead(scl_pin) ? F("HIGH ok") : F("LOW <-- held down!"));
  }

  // setSDA/setSCL must be called before begin()
  Wire.setSDA(sda_pin);
  Wire.setSCL(scl_pin);
  Wire.begin();
  Wire.setClock(GPS_I2C_CLOCK);
  delay(100);

  /*
    TEMP DIAGNOSTIC: scan the whole bus, then probe 0x42 with the raw
    endTransmission code so we can tell the failure modes apart:
    0 = ACK, 2 = NACK (bus ok, nobody answered), 4 = bus error, 5 = timeout.
  */
  if (debug_serial){
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++){
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0){
        Serial.print(F("GPS: I2C device at 0x"));
        Serial.println(addr, HEX);
        found++;
      }
    }
    Serial.print(F("GPS: scan found "));
    Serial.print(found);
    Serial.println(F(" device(s)"));
    Wire.beginTransmission(GPS_I2C_ADDR);
    Serial.print(F("GPS: 0x42 endTransmission code "));
    Serial.println(Wire.endTransmission());
  }

  initialized = false;
  const uint8_t monVer[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};

  /*
    Probe until the module answers. shutdownGPS() leaves it in RXM-PMREQ backup,
    and a warm reset (reflash, reset button) can hit it while it is still down.
    The bus traffic is itself the DDC wakeup source, so retrying is what brings
    it back; a cold start needs the retries too while the module boots.
  */
  uint8_t ack = 1;
  uint32_t probe_start = millis();
  while (ack != 0 && (millis() - probe_start < GPS_probe_timeout)){
    Wire.beginTransmission(GPS_I2C_ADDR);
    Wire.write(monVer, sizeof(monVer));
    ack = Wire.endTransmission();
    if (ack != 0){
      delay(50);
      IWatchdog.reload();
    }
  }

  if (ack == 0){
    delay(200);  // give the module time to place the answer in its buffer
    uint16_t answer_size = availBytes();
    if (answer_size > 0){
      initialized = true;
      // Drop the version string so it does not sit in front of the first PVT
      flushDDC(answer_size);

      // UBX-CFG-RATE: measRate = GPS_nav_period_ms, navRate = 1, timeRef = GPS
      uint8_t rate[6] = {(uint8_t) (GPS_nav_period_ms & 0xFF),
                         (uint8_t) (GPS_nav_period_ms >> 8),
                         0x01, 0x00, 0x01, 0x00};
      sendUbx(0x06, 0x08, rate, 6);
      delay(100);
      flushDDC(availBytes());  // drop the ACK
      if (debug_serial){
        Serial.print(F("GPS: ready, nav rate "));
        Serial.print(GPS_nav_rate_hz);
        Serial.println(F(" Hz"));
      }
    } else if (debug_serial){
      Serial.println(F("GPS: no MON-VER response"));
    }
  } else if (debug_serial){
    Serial.println(F("GPS: no ACK on I2C address 0x42"));
  }

  delay(100);
  currentPosition = {0,0,0,0,0,0};
}

uint16_t GPS_Manager::availBytes(void){
  /*
    Number of bytes waiting in the module's DDC output buffer, from the
    0xFD/0xFE register pair.
  */
  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write((uint8_t) 0xFD);
  if (Wire.endTransmission(false) != 0){  // repeated start, keep the bus
    return 0;
  }
  if (Wire.requestFrom(GPS_I2C_ADDR, (uint8_t) 2) < 2){
    return 0;
  }
  uint16_t nbytes = ((uint16_t) Wire.read() << 8) | (uint8_t) Wire.read();
  return (nbytes == 0xFFFF) ? 0 : nbytes;  // 0xFFFF means "no data" per the protocol spec
}

void GPS_Manager::flushDDC(uint16_t nbytes){
  /*
    Discard nbytes from the output buffer, 32 bytes at a time to stay within
    the Wire receive buffer.
  */
  while (nbytes > 0){
    uint8_t chunk = (nbytes > 32) ? 32 : (uint8_t) nbytes;
    uint8_t got = Wire.requestFrom(GPS_I2C_ADDR, chunk);
    if (got == 0){
      break;
    }
    while (Wire.available()){
      (void) Wire.read();
    }
    nbytes -= got;
  }
}

bool GPS_Manager::pollPVT(uint32_t max_wait_time){
  /*
    Poll NAV-PVT and block until the answer has been read and decoded, or
    max_wait_time has passed. Returns true if a checksum-valid PVT frame was
    decoded; the caller inspects pvt.valid to see whether it also holds a fix.
  */
  if (!initialized){
    return false;
  }
  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write(navPvtPoll, sizeof(navPvtPoll));
  if (Wire.endTransmission() != 0){
    return false;
  }

  uint32_t poll_start = millis();
  uint16_t avail = 0;
  while (millis() - poll_start < max_wait_time){
    avail = availBytes();
    if (avail >= GPS_pvt_frame_size){
      break;
    }
    delay(20);
    IWatchdog.reload();
  }
  if (avail < GPS_pvt_frame_size){
    return false;
  }

  uint8_t frame[GPS_pvt_frame_size];
  uint16_t idx = 0;
  bool gotPVT = false;
  while (avail > 0){
    uint8_t chunk = (avail > 32) ? 32 : (uint8_t) avail;
    uint8_t got = Wire.requestFrom(GPS_I2C_ADDR, chunk);
    if (got == 0){
      break;
    }
    while (Wire.available()){
      uint8_t b = Wire.read();
      if (idx == 0 && b != 0xB5){          // sync char 1
        continue;
      }
      if (idx == 1 && b != 0x62){          // sync char 2
        idx = 0;
        continue;
      }
      frame[idx++] = b;
      if (idx == GPS_pvt_frame_size){
        // NAV-PVT is class 0x01, id 0x07 with a 92 byte payload
        if (frame[2] == 0x01 && frame[3] == 0x07 && ubxU2(frame + 4) == 92){
          uint8_t ckA = 0, ckB = 0;
          for (uint16_t j = 2; j < GPS_pvt_frame_size - 2; j++){
            ckA += frame[j];
            ckB += ckA;
          }
          if (ckA == frame[GPS_pvt_frame_size - 2] && ckB == frame[GPS_pvt_frame_size - 1]){
            decodePVT(frame + 6);
            gotPVT = true;
          }
        }
        idx = 0;
      }
    }
    avail -= got;
    IWatchdog.reload();
  }
  return gotPVT;
}

void GPS_Manager::decodePVT(const uint8_t * p){
  /*
    Decode a NAV-PVT payload (92 bytes, protocol version 18) into pvt.
    Offsets are counted from the start of the payload.
  */
  pvt.year      = ubxU2(p + 4);
  pvt.month     = p[6];
  pvt.day       = p[7];
  pvt.hour      = p[8];
  pvt.minute    = p[9];
  pvt.second    = p[10];
  pvt.timeValid = (p[11] & 0x03) == 0x03;   // validDate and validTime
  pvt.fixType   = p[20];
  pvt.numSV     = p[23];
  pvt.valid     = ((p[21] & 0x01) != 0) && (pvt.fixType >= 2);  // gnssFixOK

  if (pvt.valid){
    pvt.lng_e7     = ubxI4(p + 24);
    pvt.lat_e7     = ubxI4(p + 28);
    pvt.hAcc_mm    = ubxU4(p + 40);
    pvt.gSpeed_mms = ubxI4(p + 60);
    pvt.headMot_e5 = ubxI4(p + 64);
  }
  fix = pvt.valid;
}

uint8_t GPS_Manager::setTimeFromGps(){

  uint8_t hour, minute, second;
  if (!enable_GPS){
    date.year  = 2025;
    date.month = 1;
    date.day   = 17;
    hour = 14;
    minute = 21;
    second = 0;
    date.valid = true;
  } else {
    if (!initialized){
      return 1;
    }
    uint32_t searchStart = millis();

    // We poll the module until it reports a valid date and time. The M8 knows
    // the UTC time from the navigation message before it has a position fix,
    // so this normally succeeds well before performNReadings does.
    while (!pvt.timeValid){
      if (millis() - searchStart >= max_GPS_read_time){
        return 2;
      }
      pollPVT(min(watchdog_wait_time/2, max_GPS_read_time - (millis() - searchStart)));
      IWatchdog.reload();
    }
    hour   = pvt.hour;
    minute = pvt.minute;
    second = pvt.second;
    date.year  = pvt.year;
    date.month = pvt.month;
    date.day   = pvt.day;
    date.valid = true;
  }
  // We set the RTC using the GPS measurements
  setTime(hour, minute, second, date.day, date.month, date.year);

  return 0;
}


uint8_t GPS_Manager::updateTimestamp(uint32_t max_wait_time, bool refreshGPStime){

  if (refreshGPStime){
    uint8_t rc = setTimeFromGps();
    if (rc != 0){
      return rc;
    } 
  }

  // Then update the timestamp
  timestamp = now();
  return 0;
}

uint8_t GPS_Manager::performNReadings(uint8_t N, uint32_t max_wait_time, bool logEveryReading){
  /*
    We read our position from the GPS N times, and push each 
    to the private packet vector. User then need to call the processReadings method
    to push the data to the public GPSReadings array
  */
 
  int8_t counter = 0;
  uint32_t start = millis();
  if (N > readings_per_measurement){
    if (debug_serial){
      Serial.println(F("N is larger than max value. Setting N equal to max value"));
    }
  }
  if (logEveryReading){
    sd_writer.logString("t:lat:lng:vel:dir");
  }
  uint8_t numReadings = min(N, readings_per_measurement);
  while( (counter < numReadings) && (millis() - start < max_wait_time)){
    updateTimestamp(max_wait_time, false);
    getGPSData(min(watchdog_wait_time/2, max_wait_time - (millis() - start)));
    if (logEveryReading){
      GPS_Data latestReading = packet.back();
      logReading(latestReading);
    }
    counter++;
    IWatchdog.reload();
  }

  iterations++;

  if (counter == N){
    return 0;
  } else {
    return 1;
  }
}

uint8_t GPS_Manager::getGPSData(uint32_t max_wait_time){
  /*
    Read a single full measurement packet from the GPS,
    or return a dummy value in case the GPS is disabled for debugging in config.h
    If the GPS spends more than max_wait_time milliseconds, then a reading filled with zeros is returned
  */
  GPS_Data reading = {0,0,0,0,0};
  if (!enable_GPS){
    reading.timestamp = timestamp;
    reading.lat       = scale_factor*4;
    reading.lng       = scale_factor*5;
    reading.vel       = scale_factor*6;
    reading.direction = scale_factor*7;
    date.year = 2025;
    date.month = 1;
    date.day = 16;
  } else {
    if (!initialized){
      return 2;
    }
    uint32_t searchStart = millis();

    // Poll until the module reports a usable fix, or we run out of time. A
    // frame without a fix still carries date and satellite count, so it is
    // worth decoding either way.
    while (!pvt.valid && (millis() - searchStart < max_wait_time)){
      pollPVT(min(watchdog_wait_time/2, max_wait_time - (millis() - searchStart)));
      IWatchdog.reload();
    }

    reading.timestamp = timestamp;
    if (pvt.valid){
      // 1e-7 deg -> deg and mm/s -> m/s before the shared scale_factor is applied
      reading.lat       = (uint32_t) (scale_factor*(pvt.lat_e7*1e-7));
      reading.lng       = (uint32_t) (scale_factor*(pvt.lng_e7*1e-7));
      reading.vel       = (uint32_t) (scale_factor*(pvt.gSpeed_mms/1000.0));
      reading.direction = (uint32_t) (scale_factor*(pvt.headMot_e5*1e-5));
    }

    if (pvt.timeValid || iterations < 1){
      date.year  = pvt.year;
      date.month = pvt.month;
      date.day   = pvt.day;
    }

  }
  packet.push_back(reading);
  return 0;
}

void GPS_Manager::getDeploymentMessage(uint32_t buoy_ID){
  /*
    Deployment message format:
    UIzzzzttttttttyyyyxxxxE
    Where
    z is the 4 byte buoy ID
    t is the 8 byte timestamp
    y is the 4 byte latitude
    x is the 4 byte longitude

    Total size: 23
  */
  struct GPS_Data initial_fix = GPSReadings.back();
  
  if (debug_serial){
    Serial.println("Writing deployment message!");
    delay(100);
  }
  // Create message
  uint8_t offset = 0;
  deploymentMessage[offset++] = 'U';
  deploymentMessage[offset++] = 'I';
  
  msg_insert_uint(deploymentMessage, buoy_ID, offset, deployment_message_size, offset, true);
  msg_insert_uint(deploymentMessage, timestamp, offset, deployment_message_size, offset, true);
  msg_insert_uint(deploymentMessage, initial_fix.lat, offset, deployment_message_size, offset, true);
  msg_insert_uint(deploymentMessage, initial_fix.lng, offset, deployment_message_size, offset, true);
  deploymentMessage[offset++] = 'E';
  
  if (debug_serial){
    Serial.println("Removing deployment data (gps)");
    delay(100);
  }
  gps_manager.GPSReadings.pop_back();
  
}

void GPS_Manager::processReadings(bool fullProcessingToggle){
  /*
    Method which processes the measured data packet and,
    if fullProcessingToggle is true, filters extreme values before computing 
    an average. Pushes the averaged GPS reading to the GPSReadings deque.
  */

  // Storage variables
  etl::vector<uint32_t, readings_per_measurement> int32vals;

  // As it is, afaik, not possible to iterate over struct variables
  // We instead have to perform the iteration though code repetition
  for (int i = 0; i < packet.size(); i++){
    int32vals.push_back(packet[i].lat);
  }
  if (fullProcessingToggle){
    mean_values.lat = filter_vector(int32vals);
  } else {
    etl::mean<uint32_t, double> mean_int32vals(int32vals.begin(), int32vals.end());
    mean_values.lat = (uint32_t) mean_int32vals;
  }
  mean_values.timestamp = timestamp;
  int32vals.clear();

  delay(50);
  IWatchdog.reload();

  for (int i = 0; i < packet.size(); i++){
    int32vals.push_back(packet[i].lng);
  }

  // Sigma filter or just averaging
  if (fullProcessingToggle){
    mean_values.lng = filter_vector(int32vals);
  } else {
    etl::mean<uint32_t, double> mean_int32vals(int32vals.begin(), int32vals.end());
    mean_values.lng = (uint32_t) mean_int32vals;
  }
  int32vals.clear();
  delay(50);
  
  for (int i = 0; i < packet.size(); i++){
    int32vals.push_back(packet[i].vel);
  }

  if (fullProcessingToggle){
    mean_values.vel = filter_vector(int32vals);
  } else {
    etl::mean<uint32_t, double> mean_int32vals(int32vals.begin(), int32vals.end());
    mean_values.vel = (uint32_t) (mean_int32vals);
  }
  current_buoy_velocity = mean_values.vel;
  IWatchdog.reload();
  int32vals.clear();
  delay(50);
  
  for (int i = 0; i < packet.size(); i++){
    int32vals.push_back(packet[i].direction);
  }
  if (fullProcessingToggle){
    mean_values.direction = filter_vector(int32vals);
  } else {
    etl::mean<uint32_t, double> mean_int32vals(int32vals.begin(), int32vals.end());
    mean_values.direction = (uint32_t) mean_int32vals;
  }

  IWatchdog.reload();
  int32vals.clear();
  delay(50);
  

  if (GPSReadings.size() == max_number_of_measurements){
    GPSReadings.pop_front();
  }

  // Minus one to match file name and ID number
  mean_values.readingID = sd_writer.logCount > 0 ?  sd_writer.logCount - 1 : 0;
  if (debug_serial){
    Serial.print("Reading IDs: ");
    Serial.println(mean_values.readingID);
  }

  currentPosition = mean_values;
  GPSReadings.push_back(mean_values);

  sd_writer.logString("Filtered:");
  logReading(mean_values);
}

void GPS_Manager::shutdownGPS(void){
  /*
    We shut the GPS down to save power.

    This PCB has no GPS enable line, so instead of cutting power we ask the
    module for a software backup (UBX-RXM-PMREQ) with the DDC port kept as a
    wakeup source. Any I2C traffic in the next begin() wakes it again, and it
    keeps its ephemeris so the next fix is a hot start.
  */
  if (sleep_GPS && initialized){
    // duration = 0 (until woken), flags = backup, wakeupSources = DDC(spics)
    uint8_t pmreq[16] = {0x00, 0x00, 0x00, 0x00,   // version + reserved
                         0x00, 0x00, 0x00, 0x00,   // duration 0 = infinite
                         0x02, 0x00, 0x00, 0x00,   // flags: backup
                         0x08, 0x00, 0x00, 0x00};  // wakeupSources: spics (DDC)
    sendUbx(0x02, 0x41, pmreq, 16);
    delay(20);
  }
  packet.clear();
  Wire.end();
  initialized = false;
}

size_t GPS_Manager::updateTransmitMessage(){
  /*
    We cast the oldest GPS reading to a byte array for transmission
    then remove the reading from the deque. 
  */
  GPS_Data gpsdata = GPSReadings.front();

  // Create transmit message
  uint8_t offset = 0;
  msgB[offset++] = 'G';
  msg_insert_uint(msgB, gpsdata.readingID, offset, GPS_message_size, offset, true);
  msg_insert_uint(msgB, gpsdata.lat, offset, GPS_message_size, offset, true);
  msg_insert_uint(msgB, gpsdata.lng, offset, GPS_message_size, offset, true);
  msg_insert_uint(msgB, gpsdata.vel, offset, GPS_message_size, offset, true);
  msg_insert_uint(msgB, gpsdata.direction, offset, GPS_message_size, offset, true);
  msg_insert_uint(msgB, gpsdata.timestamp, offset, GPS_message_size, offset, true);  
  msgB[offset++] = 'E';
  
  GPSReadings.pop_front();
  return GPS_message_size;
}

uint8_t GPS_Manager::logReading(GPS_Data & data){
  /*
    The latest reading is written to an SD file. 
  */

  byte data_reading[1 + sizeof(data.readingID) + 4*sizeof(data.lat) + sizeof(data.timestamp) + 1];
  size_t data_reading_size = sizeof(data_reading);

  uint8_t offset = 0;
  data_reading[offset++] = 'G';
  msg_insert_uint(data_reading, data.readingID, offset, data_reading_size, offset, true);
  msg_insert_uint(data_reading, data.lat, offset, data_reading_size, offset, true);
  msg_insert_uint(data_reading, data.lng, offset, data_reading_size, offset, true);
  msg_insert_uint(data_reading, data.vel, offset, data_reading_size, offset, true);
  msg_insert_uint(data_reading, data.direction, offset, data_reading_size, offset, true);
  msg_insert_uint(data_reading, data.timestamp, offset, data_reading_size, offset, true);
  data_reading[offset++] = 'E';
  
  uint8_t state = sd_writer.logByteArray(data_reading, sizeof(data_reading));
  return state;
}


/*
  Method which reads the filtered measurement from 
  a fil ein the file buffer, and appends it to the 
  front of the GPSReadings deque. 

*/
void GPS_Manager::getMeasurementFromFile(void){
  GPS_Data readData;
  if (sd_writer.numLines > 0){
    uint8_t lineNo = 1;
    fileLine currentLine = {"",0}, prevLine = {"",0};
    while(lineNo < sd_writer.file_buffer.size()){
      // We check for the filtered value
      prevLine = currentLine;
      currentLine = {"",0};
      currentLine = sd_writer.file_buffer.at(lineNo);
      

      if ((strncmp(prevLine.line, "Filtered:", 4) == 0) \
          && (strncmp(currentLine.line, "71b", 3) == 0)){

            byte filteredData[GPS_message_size];

            // Make a copy
            char tmpHolder[max_line_length];
            sscanf(currentLine.line, "%s", tmpHolder);
            
            // Filter for data
            for (uint8_t index = 0; index < GPS_message_size; index++){
              sscanf(tmpHolder, "%db%s", &filteredData[index], tmpHolder);
            }
            uint8_t offset = 1;
            readData.readingID = msg_extract_uint<uint16_t>(filteredData, offset, true, offset);
            readData.lat       = msg_extract_uint<uint32_t>(filteredData, offset, true, offset);
            readData.lng       = msg_extract_uint<uint32_t>(filteredData, offset, true, offset);
            readData.vel       = msg_extract_uint<uint32_t>(filteredData, offset, true, offset);
            readData.direction = msg_extract_uint<uint32_t>(filteredData, offset, true, offset);
            readData.timestamp = msg_extract_uint<uint64_t>(filteredData, offset, true, offset);


            if (debug_serial){
              delay(300);
              Serial.println(sd_writer.file_buffer.at(lineNo).line);
              Serial.println("Recovered data:\n---------------\n");
              Serial.print("ID = ");
              Serial.println(readData.readingID);
              Serial.print("Lat = ");
              Serial.println(readData.lat);
              Serial.print("Lon = ");
              Serial.println(readData.lng);
              Serial.print("Vel = ");
              Serial.println(readData.vel);
              Serial.print("Dir = ");
              Serial.println(readData.direction);
              Serial.print("t = ");
              Serial.println(readData.timestamp);
              Serial.println("---------------\n");
              delay(200);
            }
            
            // New measurements take precedence in memory over old measurements
            if (GPSReadings.full()){
              GPSReadings.pop_front();
            }
            GPSReadings.push_back(readData);
            break;
      }
      lineNo++;

      IWatchdog.reload();
    }
  }
}


void GPS_Manager::updateBeaconMsg(uint32_t WiO_ID){
  uint8_t offset = 0;
  beaconMsg[offset++] = 'U';
  beaconMsg[offset++] = 'R';
  msg_insert_uint(beaconMsg, currentPosition.timestamp, offset, beaconMsgSize, offset, true);
  msg_insert_uint(beaconMsg, currentPosition.lat, offset, beaconMsgSize, offset, true);
  msg_insert_uint(beaconMsg, currentPosition.lng, offset, beaconMsgSize, offset, true);
  msg_insert_uint(beaconMsg, WiO_ID, offset, beaconMsgSize, offset, true);
  beaconMsg[offset++] = 'E';
}


// Default GPS manager on the I2C2 bus shared with the GPS module
GPS_Manager gps_manager(I2C_SDA_PIN, I2C_SCL_PIN);