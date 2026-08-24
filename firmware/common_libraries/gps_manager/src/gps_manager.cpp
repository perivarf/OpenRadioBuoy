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

void GPS_Manager::begin(){
  /*
    Bring up the bus, verify the module answers a MON-VER poll, and raise navigation rate.
    Without the CFG-RATE the M8 only uses 1 Hz and repeated
    polls would return the same fix over and over.
  */

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
    Probe until the module answers. After a Controlled GNSS stop (shutdownGPS) the
    receiver's CPU and I2C stay alive, so it answers on the first try. The retries
    only matter for a cold start, or a warm reset (reflash, reset button) that hits
    the module while it is still booting.
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

      // Restart the GNSS engine (CFG-RST resetMode 0x09, hotstart) after a stop.
      uint8_t rstStart[4] = {0x00, 0x00, 0x09, 0x00};
      sendUbx(0x06, 0x04, rstStart, 4);
      delay(50);

      /*
        Turn NMEA off on DDC. BEFORE the CFG-RATE below, so the sentences never get a
        chance to come out at the raised rate.

        The factory default on this port is GGA, GLL, GSA, GSV, RMC and VTG, emitted once
        per navigation epoch - ten times a second once CFG-RATE has done its work. Nothing
        in this repo reads NMEA, but readPvtFrame drains the WHOLE output buffer, so every
        one of those bytes crosses the bus at the I2C byte cost before being discarded by
        the 0xB5 sync test.

        It was two thirds of the GPS bill: 285 ms/s measured 2026-08-15, which at 90 us per
        byte (100 kHz) is ~3200 B/s where NAV-PVT itself needs ~1200.

        The three-byte form of CFG-MSG sets the rate for the port the command ARRIVED on,
        so this is scoped to DDC and leaves the module's UART configuration alone. The list
        runs past the six that ship enabled: disabling a sentence that was already off
        costs one message, and is cheaper than finding out later that it was on.

        No CFG-CFG, so none of this reaches the module's flash - but shutdownGPS() ends with
        Wire.end() and begin() runs again on every wake-up, so it is re-sent each cycle
        anyway.
      */
      static const uint8_t kNmeaOff[] = {
          0x00, 0x01, 0x02, 0x03, 0x04, 0x05,          // GGA GLL GSA GSV RMC VTG - on by default
          0x06, 0x07, 0x08, 0x09, 0x0A, 0x0D, 0x0F};   // GRS GST ZDA GBS DTM GNS VLW
      for (uint8_t i = 0; i < sizeof(kNmeaOff); i++){
        uint8_t msg[3] = {0xF0, kNmeaOff[i], 0x00};    // class 0xF0 = NMEA, rate 0 = off
        sendUbx(0x06, 0x01, msg, 3);
        delay(10);
        IWatchdog.reload();
      }
      flushDDC(availBytes());  // the ACKs, and whatever was already queued

      // UBX-CFG-RATE: measRate = GPS_nav_period_ms, navRate = 1, timeRef = GPS
      uint8_t rate[6] = {(uint8_t) (GPS_nav_period_ms & 0xFF),
                         (uint8_t) (GPS_nav_period_ms >> 8),
                         0x01, 0x00, 0x01, 0x00};
      sendUbx(0x06, 0x08, rate, 6);
      delay(100);
      flushDDC(availBytes());  // drop the ACK

      /*
        Turn NAV-PVT into a periodic message instead of something update() has to ask for.
        AFTER CFG-RATE, so the first solution out is already at the final rate.

        Same three-byte CFG-MSG form as the NMEA loop above, so it is scoped to the port
        the command arrived on (DDC) and leaves the UART configuration alone.

        This is the fix for the 1.2 s gaps in gps.csv. A polled NAV-PVT has exactly one
        answer, and when that answer went missing nothing else was ever coming: update()
        had to sit out its whole timeout - twelve navigation epochs - before it could ask
        again. Two 30-minute captures on 2026-08-16 lost 446 and 461 epochs that way,
        ~38 % of the run, and the count did not move when NMEA was switched off or when
        the SD stalls got three times shorter. With the module emitting on its own, a
        frame that goes missing costs the one epoch: the next arrives 100 ms later
        whether or not anybody asked.

        Not written to flash (no CFG-CFG). shutdownGPS() ends with Wire.end() and begin()
        runs on every wake-up, so this is re-sent each cycle, same as the NMEA list.
      */
      uint8_t pvtOn[3] = {0x01, 0x07, 0x01};  // class 0x01 NAV, id 0x07 PVT, one per epoch
      sendUbx(0x06, 0x01, pvtOn, 3);
      delay(50);
      flushDDC(availBytes());  // the ACK, and the first solution if it beat us here
      frameIdx_ = 0;
      lastFixMs_ = lastCheckMs_ = millis();

      if (debug_serial){
        Serial.print(F("GPS: ready, nav rate "));
        Serial.print(GPS_nav_rate_hz);
        Serial.print(F(" Hz, NAV-PVT periodic, DDC check every "));
        Serial.print(GPS_ddc_check_ms);
        Serial.println(F(" ms"));
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

uint16_t GPS_Manager::availBytes(bool * ok){
  /*
    Number of bytes waiting in the module's DDC output buffer, from the
    0xFD/0xFE register pair.

    `ok` separates the two ways this can return 0. A transaction that never completed
    means the bus is in trouble and the count is unknown; a completed read of 0 means
    the receiver genuinely has nothing to say yet. update() has to tell them apart -
    treating a wedged bus as an empty buffer is what let it wait out a full timeout
    against a receiver it could not hear. Callers that only want the count (begin,
    the flushDDC sites, pollPVT) pass nothing and keep the old behaviour.
  */
  if (ok) *ok = false;
  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write((uint8_t) 0xFD);
  if (Wire.endTransmission(false) != 0){  // repeated start, keep the bus
    return 0;
  }
  if (Wire.requestFrom(GPS_I2C_ADDR, (uint8_t) 2) < 2){
    return 0;
  }
  uint16_t nbytes = ((uint16_t) Wire.read() << 8) | (uint8_t) Wire.read();
  if (ok) *ok = true;
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

    BLOCKING - up to max_wait_time. Never call this from the wave capture loop; that is
    what update() is for. The drifter's own measurement path (getGPSData, setTimeFromGps)
    runs with the IMU idle and can afford it.

    Since begin() switched the module to periodic output the explicit poll is belt and
    braces: the buffer fills on its own every epoch, so this usually finds a solution
    already waiting rather than one it asked for. The poll stays because it still works
    if the CFG-MSG did not take.
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
  return readPvtFrame(avail);
}

/*
  Drain `avail` bytes from the DDC output buffer, syncing on 0xB5 0x62 and
  decoding the first checksum-valid NAV-PVT frame. Assumes at least one full
  frame is already waiting (caller checks availBytes). Shared by the blocking
  pollPVT and the non-blocking update().
*/
bool GPS_Manager::readPvtFrame(uint16_t avail, uint16_t maxBytes){
  uint16_t budget = (avail > maxBytes) ? maxBytes : avail;
  bool gotPVT = false;
  while (budget > 0){
    uint8_t chunk = (budget > 32) ? 32 : (uint8_t) budget;
    uint8_t got = Wire.requestFrom(GPS_I2C_ADDR, chunk);
    if (got == 0){
      break;
    }
    while (Wire.available()){
      uint8_t b = Wire.read();
      if (frameIdx_ == 0 && b != 0xB5){          // sync char 1
        continue;
      }
      if (frameIdx_ == 1 && b != 0x62){          // sync char 2
        // A 0xB5 here is the start of a real frame, not a failed second sync char:
        // resyncing to 0 would drop it and cost the epoch.
        frameIdx_ = (b == 0xB5) ? 1 : 0;
        continue;
      }
      frameBuf_[frameIdx_++] = b;
      if (frameIdx_ == GPS_pvt_frame_size){
        // NAV-PVT is class 0x01, id 0x07 with a 92 byte payload
        if (frameBuf_[2] == 0x01 && frameBuf_[3] == 0x07 && ubxU2(frameBuf_ + 4) == 92){
          uint8_t ckA = 0, ckB = 0;
          for (uint16_t j = 2; j < GPS_pvt_frame_size - 2; j++){
            ckA += frameBuf_[j];
            ckB += ckA;
          }
          if (ckA == frameBuf_[GPS_pvt_frame_size - 2] && ckB == frameBuf_[GPS_pvt_frame_size - 1]){
            decodePVT(frameBuf_ + 6);
            gotPVT = true;
          }
        }
        frameIdx_ = 0;
      }
    }
    budget -= got;
    IWatchdog.reload();
  }
  return gotPVT;
}

/*
  Collect whatever NAV-PVT the module has emitted. Call it once per loop iteration; it
  rate-limits itself to GPS_ddc_check_ms and never waits on anything, so it can run inside
  the wave capture's IMU drain without starving the FIFO. freshFix() is true only on the
  call that decoded a new PVT, so a caller writes exactly one row per fix.

  There is no poll here any more, and no IDLE/WAIT state machine. begin() put the receiver
  in periodic output, so the rate is the module's: this can only ever miss an epoch, never
  make an extra one, and the entire job is making sure nothing costs more than one.

  Worst case for a single call, at 400 kHz: ~0.13 ms for the length read plus ~3 ms for a
  full-budget drain, against a 213 ms FIFO budget. The blocking pollPVT (up to ~1.1 s) is
  never reached from here.
*/
void GPS_Manager::update(void){
  freshFix_ = false;
  if (!initialized){
    return;
  }
  uint32_t nowMs = millis();
  if (nowMs - lastCheckMs_ < GPS_ddc_check_ms){
    return;
  }

  /*
    The stall timeout below measures silence while somebody was LISTENING, so a long gap
    between two CALLS has to rebase it. Nothing was read in that gap, and nothing could
    have been: the module keeps its output buffer, and the next call drains whatever
    collected there.

    Without this, every caller that goes away for longer than GPS_stall_timeout_ms is
    reported as a dead bus on its way back. takeReading does exactly that: waitForGpsFix
    polls until a fix lands, then seedReadingId, analyzer_.begin and startSession run -
    mkdir, five file opens, headers and the preAllocate calls, tens of MB on the raw file -
    before the capture loop makes its first update() call. That put a "DDC stalled, bus
    reset #1" in front of a capture whose very first debug line then showed the full 10 Hz.
    start_session_us in ses.csv is the gap.
  */
  if (nowMs - lastCheckMs_ > GPS_stall_timeout_ms){
    lastFixMs_ = nowMs;
  }
  lastCheckMs_ = nowMs;

  bool busOk = true;
  uint16_t avail = availBytes(&busOk);
  if (busOk && avail > 0 && readPvtFrame(avail, GPS_ddc_max_drain)){
    lastFixMs_ = nowMs;
  }

  // Silence for ten epochs is not something waiting can fix - see recoverDdc.
  if (nowMs - lastFixMs_ > GPS_stall_timeout_ms){
    recoverDdc(nowMs);
  }
}

void GPS_Manager::recoverDdc(uint32_t nowMs){
  /*
    Nothing decoded for GPS_stall_timeout_ms. Two things cause that and both are addressed
    here, because from the host side they look identical - a length register that reads as
    empty forever:

      1. The STM32's I2C peripheral latched an error flag. It will not clear on its own and
         every later transaction fails; end/begin re-initialises it.
      2. The module restarted and lost the CFG-MSG that makes it emit NAV-PVT at all, so it
         is alive and answering on the bus but has nothing to send.

    No delay() anywhere: this runs in the capture loop. end/begin are register writes only -
    i2c_deinit does an RCC force-reset, which is what actually clears case 1 - so the only
    part that can take time is the CFG-MSG write: ~0.3 ms on a healthy bus, and bounded by
    I2C_TIMEOUT_TICK (20 ms, set in platformio.ini, down from the core's 100 ms default) if
    the bus is still stuck. Setting lastFixMs_ rate-limits the whole thing to one attempt
    per stall timeout even when the cause persists, so the worst this costs the FIFO is
    20 ms per second against a 213 ms budget.
  */
  Wire.end();
  Wire.setSDA(sda_pin);  // same order as begin(): setSDA/setSCL must precede begin()
  Wire.setSCL(scl_pin);
  Wire.begin();
  Wire.setClock(GPS_I2C_CLOCK);
  frameIdx_ = 0;  // whatever was half-parsed died with the bus

  uint8_t pvtOn[3] = {0x01, 0x07, 0x01};
  sendUbx(0x06, 0x01, pvtOn, 3);

  lastFixMs_ = nowMs;
  recoverCount_++;
  if (debug_serial){
    Serial.print(F("GPS: DDC stalled, bus reset #"));
    Serial.println(recoverCount_);
  }
}

void GPS_Manager::decodePVT(const uint8_t * p){
  /*
    Decode a NAV-PVT payload (92 bytes, protocol version 18) into pvt.
    Offsets are counted from the start of the payload.
  */
  // The receiver's own epoch clock. hour/minute/second below resolve to a second, which is
  // too coarse to tell one 10 Hz epoch from the next; iTOW is what gives gps.csv a time
  // base independent of when the firmware got round to reading the frame.
  pvt.iTOW_ms   = ubxU4(p);
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
    // The channels gps.csv logs, in the module's own integer units, and no more:
    // Offsets are the NAV-PVT layout
    pvt.hAcc_mm    = ubxU4(p + 40);
    pvt.vAcc_mm    = ubxU4(p + 44);
    pvt.velN_mms   = ubxI4(p + 48);
    pvt.velE_mms   = ubxI4(p + 52);
    pvt.velD_mms   = ubxI4(p + 56);
    pvt.gSpeed_mms = ubxI4(p + 60);
    pvt.headMot_e5 = ubxI4(p + 64);
    pvt.sAcc_mms   = ubxU4(p + 68);
    pvt.pDOP_e2    = ubxU2(p + 76);
  }
  fix = pvt.valid;
  freshFix_ = true;  // new PVT decoded; update() reports it one-shot
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
  // counter only advances on a reading that was actually pushed, so the return
  // code says how many usable fixes we got, not how many attempts we made.
  while( (counter < numReadings) && (millis() - start < max_wait_time)){
    updateTimestamp(max_wait_time, false);
    uint8_t rc = getGPSData(min(watchdog_wait_time/2, max_wait_time - (millis() - start)));
    if (rc == 2){
      break;      // module not up, or packet full: retrying cannot help
    }
    if (rc != 0){
      IWatchdog.reload();
      continue;   // this attempt found no fix, try again with the time that is left
    }
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

/*
  The unit conversions in getGPSData are exact integer factors only as long as
  scale_factor is 1e5: NAV-PVT reports heading as deg*1e5, which is then already
  deg*scale_factor, and mm/s converts to m/s*scale_factor by the exact factor
  scale_factor/1000. Both silently stop being exact if scale_factor changes.
*/
static_assert(scale_factor == 100000,
              "getGPSData's integer unit conversions assume scale_factor == 1e5");

uint8_t GPS_Manager::getGPSData(uint32_t max_wait_time){
  /*
    Read a single measurement from the GPS, or a dummy value in case the GPS is
    disabled for debugging in config.h.

    Only a reading fresh, valid value

    Error codes:
      0 - reading pushed
      1 - no usable fix within max_wait_time (nothing pushed)
      2 - module never came up, or the packet is already full
  */
  if (packet.full()){
    return 2;
  }
  GPS_Data reading = {0,0,0,0,0,0};
  if (!enable_GPS){
    reading.timestamp = timestamp;
    reading.lat       = gps_coord_scale*4;
    reading.lng       = gps_coord_scale*5;
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

    /*
      Poll until a fresh frame carrying a usable fix arrives, or we run out of
      time. pollPVT returns true only when a new checksum-valid frame was decoded
    */
    bool fresh_fix = false;
    while (millis() - searchStart < max_wait_time){
      if (pollPVT(min(watchdog_wait_time/2, max_wait_time - (millis() - searchStart)))
          && pvt.valid){
        fresh_fix = true;
        break;
      }
      IWatchdog.reload();
    }

    // A frame without a fix still carries the date, so the RTC date is refreshed
    // even when the position was not usable.
    if (pvt.timeValid || iterations < 1){
      date.year  = pvt.year;
      date.month = pvt.month;
      date.day   = pvt.day;
    }

    if (!fresh_fix){
      return 1;
    }

    reading.timestamp = timestamp;
    /*
      Native NAV-PVT unit - lat/lng are 1e-7 deg (gps_coord_scale) and are copied unchanged
    */
    reading.lat       = pvt.lat_e7;
    reading.lng       = pvt.lng_e7;
    reading.vel       = pvt.gSpeed_mms > 0 ? (uint32_t) pvt.gSpeed_mms * (scale_factor/1000) : 0;
    reading.direction = pvt.headMot_e5 > 0 ? (uint32_t) pvt.headMot_e5 : 0;
  }
  packet.push_back(reading);
  return 0;
}

void GPS_Manager::getDeploymentMessage(uint32_t buoy_ID){
  /*
    Deployment message format:
    UIzzzzttttsyyyysxxxxE
    Where
    z is the 4 byte buoy ID
    t is the timestamp, sizeof(time_t) bytes (8 here: TimeLib defers to newlib)
    s is the 'P'/'N' sign char written by msg_insert_int
    y is the 4 byte latitude magnitude,  1e-7 deg
    x is the 4 byte longitude magnitude, 1e-7 deg

    Total size: deployment_message_size (25)
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
  msg_insert_int(deploymentMessage, initial_fix.lat, offset, deployment_message_size, offset, true);
  msg_insert_int(deploymentMessage, initial_fix.lng, offset, deployment_message_size, offset, true);
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

  /*
    Nothing was measured - every reading in this cycle timed out without a fix.
    Bailing out leaves currentPosition and GPSReadings at the last known good
    value; carrying on would divide by zero in etl::mean and publish the result.
  */
  if (packet.empty()){
    if (debug_serial){
      Serial.println(F("GPS: no valid fixes this cycle, keeping previous position"));
    }
    sd_writer.logString("No valid GPS fixes");
    return;
  }

  // Storage variables
  etl::vector<int32_t, readings_per_measurement> int32vals;

  // As it is, afaik, not possible to iterate over struct variables
  // We instead have to perform the iteration though code repetition
  for (int i = 0; i < packet.size(); i++){
    int32vals.push_back(packet[i].lat);
  }
  if (fullProcessingToggle){
    mean_values.lat = filter_vector(int32vals);
  } else {
    etl::mean<int32_t, double> mean_int32vals(int32vals.begin(), int32vals.end());
    mean_values.lat = (int32_t) mean_int32vals;
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
    etl::mean<int32_t, double> mean_int32vals(int32vals.begin(), int32vals.end());
    mean_values.lng = (int32_t) mean_int32vals;
  }
  int32vals.clear();
  delay(50);
  
  etl::vector<uint32_t, readings_per_measurement> uint32vals;
  for (int i = 0; i < packet.size(); i++){
    uint32vals.push_back(packet[i].vel);
  }

  if (fullProcessingToggle){
    mean_values.vel = filter_vector(uint32vals);
  } else {
    etl::mean<uint32_t, double> mean_uint32vals(uint32vals.begin(), uint32vals.end());
    mean_values.vel = (uint32_t) (mean_uint32vals);
  }
  current_buoy_velocity = mean_values.vel;
  IWatchdog.reload();
  uint32vals.clear();
  delay(50);
  
  for (int i = 0; i < packet.size(); i++){
    uint32vals.push_back(packet[i].direction);
  }
  if (fullProcessingToggle){
    mean_values.direction = filter_vector(uint32vals);
  } else {
    etl::mean<uint32_t, double> mean_uint32vals(uint32vals.begin(), uint32vals.end());
    mean_values.direction = (uint32_t) mean_uint32vals;
  }

  IWatchdog.reload();
  uint32vals.clear();
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

  // The packet has been consumed
  packet.clear();
}

void GPS_Manager::shutdownGPS(void){
  /*
    Power save mode, CPU and I2C stay alive.
    Alternatively
      1) power down module completely by cutting power to the VCC pin
      2) use timed backup
  */
  if (sleep_GPS && initialized){
    // navBbrMask 0x0000 (hotstart), resetMode 0x08 (controlled GNSS stop)
    uint8_t rst[4] = {0x00, 0x00, 0x08, 0x00};
    sendUbx(0x06, 0x04, rst, 4);
    delay(20);
  }
  packet.clear();
  Wire.end();
  frameIdx_ = 0;  // a half-parsed frame must not be finished with bytes from the next session
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
  msg_insert_int(msgB, gpsdata.lat, offset, GPS_message_size, offset, true);
  msg_insert_int(msgB, gpsdata.lng, offset, GPS_message_size, offset, true);
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

  byte data_reading[GPS_message_size];
  uint8_t data_reading_size = sizeof(data_reading);

  uint8_t offset = 0;
  data_reading[offset++] = 'G';
  msg_insert_uint(data_reading, data.readingID, offset, data_reading_size, offset, true);
  msg_insert_int(data_reading, data.lat, offset, data_reading_size, offset, true);
  msg_insert_int(data_reading, data.lng, offset, data_reading_size, offset, true);
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

            byte filteredData[GPS_message_size] = {0};

            /*
              logByteArray writes each byte as a decimal number followed by 'b',
              so the line is parsed by walking it with strtoul.
            */
            const char * cursor = currentLine.line;
            uint8_t nbytes = 0;
            while (nbytes < GPS_message_size && *cursor != '\0'){
              char * end = nullptr;
              unsigned long value = strtoul(cursor, &end, 10);
              if (end == cursor){
                break;  // no digits here: malformed or truncated line
              }
              filteredData[nbytes++] = (byte) value;
              cursor = (*end == 'b') ? end + 1 : end;
            }

            // A short line means the log entry was cut off mid-write; skip it
            if (nbytes == GPS_message_size){
              uint8_t offset = 1;
              readData.readingID = msg_extract_uint<uint16_t>(filteredData, offset, true, offset);
              readData.lat       = msg_extract_int<int32_t>(filteredData, offset, true, offset);
              readData.lng       = msg_extract_int<int32_t>(filteredData, offset, true, offset);
              readData.vel       = msg_extract_uint<uint32_t>(filteredData, offset, true, offset);
              readData.direction = msg_extract_uint<uint32_t>(filteredData, offset, true, offset);
              readData.timestamp = msg_extract_uint<time_t>(filteredData, offset, true, offset);

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
  msg_insert_int(beaconMsg, currentPosition.lat, offset, beaconMsgSize, offset, true);
  msg_insert_int(beaconMsg, currentPosition.lng, offset, beaconMsgSize, offset, true);
  msg_insert_uint(beaconMsg, WiO_ID, offset, beaconMsgSize, offset, true);
  beaconMsg[offset++] = 'E';
}


// Default GPS manager on the I2C2 bus shared with the GPS module
GPS_Manager gps_manager(I2C_SDA_PIN, I2C_SCL_PIN);