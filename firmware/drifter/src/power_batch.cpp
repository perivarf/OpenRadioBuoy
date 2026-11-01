#include "power_batch.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "config.h"
#include "gps_manager.h"
#include "lora_transceiver.h"
#include "sd_writer.h"
#include "wave_manager.h"
#include "STM32LowPower.h"

/*
  The current pattern in the supply log (decoded by power_batch_analyze.py):

    ID marker, before every test, with the board quiet:
      Stop2 kQuietLeadMs, then N x (busy kPulseMs, Stop2 kPulseMs), then Stop2
      kQuietTailMs, N = test ID. The test starts as the marker ends. A test that had
      to wait for the sd-card to be inserted or removed sends its marker twice: the
      analysis uses the last one.

    Capture-start marker, tests 4-7 and 10 only, right before the capture loop:
      Stop2 kCapGapMs, busy kCapPulseMs, Stop2 kCapGapMs. The capture loop starts as
      it ends, so the analysis window is anchored to the capture itself and not to the
      GNSS fix wait in front of it.

  Busy against Stop2 is ~8-10 mA on VBAT, far above the supply's 0.1 mA resolution.
*/
static constexpr uint32_t kQuietLeadMs  = 4000;
static constexpr uint32_t kPulseMs      = 1000;
static constexpr uint32_t kQuietTailMs  = 3000;
static constexpr uint32_t kCapGapMs     = 2000;
static constexpr uint32_t kCapPulseMs   = 3000;

// The GNSS wakes this long before a backup test ends, so the next quiet() finds it
// answering. Outside the analysis window either way.
static constexpr uint32_t kGpsWakeMarginMs = 10000;

// How often the card slot is checked while waiting for the card to be swapped
static constexpr uint32_t kCardPollMs = 2000;

/*
  Run order, all tests in one run: those without the sd-card first, then those with it,
  so the card is inserted by hand once, before test 9 (see waitForCard();
  power_batch_log.py says when). 9 and 10 repeat 1 and 4 with the card in, which gives
  the idle card's current against a quiet board rather than against the GNSS noise.
*/
static constexpr uint8_t kOrder[] = {1, 2, 3, 8, 4, 5, 9, 10, 6, 7};

static bool needsCard(uint8_t id){ return id == 6 || id == 7 || id == 9 || id == 10; }

static bool sdPresent = false;

static void busyMs(uint32_t ms){
  const uint32_t start = millis();
  while (millis() - start < ms){}
}

/*
  Stop2 for ms. Peripherals set up before a Stop2 do not work after it unless they are
  set up again (seen on the bench: the console went silent at the first Stop2, and the
  capture loop hung after the capture-start marker), so the console is re-begun here
  and SPI/I2C after the capture-start marker.
*/
static void stop2Ms(uint32_t ms){
  if (debug_serial) mySerial.flush();
  LowPower.deepSleep(ms);
  if (debug_serial || enable_bootloader_menu) mySerial.begin(serial_baud);
}

static void say(const char *msg, uint8_t id){
  if (!debug_serial) return;
  mySerial.print("PowerBatch: "); mySerial.print(msg);
  mySerial.print(" "); mySerial.println(id);
}

// SPI back on its pins after parkSpiPins(); with a card, the card is brought up too.
static void spiUp(void){
  if (sdPresent){
    if (!sd_writer.active) sd_writer.begin();
  } else {
    SPI.setMOSI(SPI_MOSI_PIN);
    SPI.setMISO(SPI_MISO_PIN);
    SPI.setSCLK(SPI_SCK_PIN);
    SPI.begin();
  }
}

// Drive the SPI lines rather than leave them floating into the hex buffer (U8) in front
// of the card. SCK high also keeps the SD activity LED dark.
static void parkSpiPins(void){
  pinMode(SPI_CS_SD_PIN, OUTPUT);   digitalWrite(SPI_CS_SD_PIN, HIGH);
  pinMode(SPI_CS_IMU_PIN, OUTPUT);  digitalWrite(SPI_CS_IMU_PIN, HIGH);
  pinMode(SPI_SCK_PIN, OUTPUT);     digitalWrite(SPI_SCK_PIN, HIGH);
  pinMode(SPI_MOSI_PIN, OUTPUT);    digitalWrite(SPI_MOSI_PIN, HIGH);
}

// The same quiet board in front of every marker: IMU powered down, GNSS stopped
// (GNSS stop, a known state whatever the test before left it in), radio asleep, card
// released, SPI parked.
static void quiet(void){
  wave_manager.sleep();
  gps_manager.begin();
  gps_manager.shutdownGPS();
  LORA.sleep();
  if (sd_writer.active) sd_writer.shutdown();
  parkSpiPins();
}

static void idMarker(uint8_t id){
  say("marker", id);
  quiet();
  stop2Ms(kQuietLeadMs);
  for (uint8_t k = 0; k < id; k++){
    busyMs(kPulseMs);
    stop2Ms(kPulseMs);
  }
  stop2Ms(kQuietTailMs);
}

/*
  With the drift track the GNSS draws 30-45 mA and wanders by several mA over seconds,
  which buries the ~5 mA pulse. It is held in GNSS stop for the marker and hot-started
  after, so the marker sits on the same quiet floor as the ID markers. The receiver
  keeps its fix data and is tracking again within seconds, well before the window.
*/
static void captureStartMarker(void){
  const bool gps = gps_manager.ready();
  if (gps) gps_manager.shutdownGPS();

  stop2Ms(kCapGapMs);
  busyMs(kCapPulseMs);
  stop2Ms(kCapGapMs);

  // The IMU (SPI) and, with the drift track, the GNSS (I2C) are used straight after;
  // begin() brings I2C up again and restarts the GNSS engine
  SPI.begin();
  if (gps) gps_manager.begin();
  say("capture marker done, gps restarted", gps);
}

// Tests 1, 3 and 9: the GNSS in timed backup, waking just before the test ends.
static void gpsBackupForTest(void){
  gps_manager.begin();
  gps_manager.backupGPS(power_batch_test_ms - kGpsWakeMarginMs);
}

// Tests 4-7 and 10: a real wave measurement, as waveCapture() in main.cpp runs it.
static void waveTest(bool gpsTrack, bool logToSd){
  spiUp();
  wave_manager.wake();
  const uint8_t status = wave_manager.benchCapture(power_batch_test_ms, gpsTrack, logToSd,
                                                   captureStartMarker);
  wave_manager.sleep();
  if (status == 0){
    wave_manager.processCapture();
  } else if (debug_serial){
    mySerial.print("PowerBatch: capture failed, status "); mySerial.println(status);
  }
}

static bool cardInSlot(void){
  sd_writer.begin();
  const bool present = sd_writer.active;
  sd_writer.shutdown();
  parkSpiPins();
  return present;
}

/*
  Wait, quiet in Stop2, until the card is in the slot (want) or out of it (!want). Run
  after the test's ID marker, so the log shows which test is waiting; when the card has
  been swapped the marker is sent again and the test starts from that one, which keeps
  the analysis window clear of the swap. A card found is checked once more after a
  pause, so one half pushed in is not taken for one in place.
*/
static bool waitForCard(bool want){
  if (cardInSlot() == want) return false;
  say(want ? "waiting for the sd-card to be inserted" : "waiting for the sd-card to be removed", 0);
  while (true){
    stop2Ms(kCardPollMs);
    if (cardInSlot() == want){
      stop2Ms(kCardPollMs);
      if (cardInSlot() == want) break;
    }
  }
  say(want ? "sd-card inserted" : "sd-card removed", 0);
  return true;
}

static void runTest(uint8_t id){
  idMarker(id);
  sdPresent = needsCard(id);
  if (waitForCard(sdPresent)) idMarker(id);
  say("test", id);
  switch (id){
    case 1:  gpsBackupForTest(); stop2Ms(power_batch_test_ms);  break;  // baseline
    case 2:                      stop2Ms(power_batch_test_ms);  break;  // GNSS stop only
    case 3:  gpsBackupForTest(); busyMs(power_batch_test_ms);   break;  // busy spin
    case 8:                      busyMs(power_batch_test_ms);   break;  // busy spin, GNSS stop
    case 9:  gpsBackupForTest(); stop2Ms(power_batch_test_ms);  break;  // baseline + sd-card
    case 4:  waveTest(false, false); break;                              // IMU
    case 5:  waveTest(true,  false); break;                              // IMU + GPS
    case 6:  waveTest(true,  false); break;                              // + sd-card, idle
    case 7:  waveTest(true,  true);  break;                              // + sd-card log
    case 10: waveTest(false, false); break;                              // IMU + sd-card
  }
}

void runPowerBatch(void){
  LORA.beginRadio(LoRa_freq_receive, LoRa_bw, LoRa_sf, LoRa_cr, LoRa_power);
  wave_manager.begin();
  LowPower.begin();

  const uint8_t n = sizeof(kOrder);
  if (debug_serial){
    mySerial.print("PowerBatch: "); mySerial.print(n); mySerial.println(" tests");
  }

  for (uint8_t k = 0; k < n; k++){
    runTest(kOrder[k]);
  }

  say("done, quiet from here on", kOrder[n - 1]);
  quiet();
  while (true){
    stop2Ms(power_batch_test_ms);
  }
}
