#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "sd_writer.h"
#include "parser_utils.h"
#include "stats.h"
#include "etl/deque.h"
#include "etl/vector.h"
#include "etl/string.h"
#include "IWatchdog.h"
#include "TimeLib.h"

struct GPS_Data{
  time_t timestamp;
  int32_t lat;
  int32_t lng;
  uint32_t vel;
  uint32_t direction;
  uint16_t readingID;
};

struct DateInfo{
  uint8_t day;
  uint8_t month;
  uint16_t year;
  bool valid;
};

/*
  Decoded UBX NAV-PVT solution, kept in the raw integer units the module
  reports so no precision is lost before the scale_factor conversion in
  getGPSData().
*/
struct UBX_PVT{
  int32_t  lat_e7;        // latitude,  1e-7 deg
  int32_t  lng_e7;        // longitude, 1e-7 deg
  int32_t  gSpeed_mms;    // 2D ground speed, mm/s
  int32_t  velN_mms;      // NED north velocity, mm/s
  int32_t  velE_mms;      // NED east  velocity, mm/s
  int32_t  velD_mms;      // NED down velocity,  mm/s (POSITIVE DOWN - gps.csv
                          // logs -velD as vUp, which is what the wave analysis
                          // builds its elevation spectrum from)
  int32_t  headMot_e5;    // course over ground, 1e-5 deg
  uint32_t hAcc_mm;       // horizontal accuracy estimate, mm
  uint32_t vAcc_mm;       // vertical accuracy estimate, mm
  uint32_t sAcc_mms;      // speed accuracy estimate, mm/s
  uint16_t pDOP_e2;       // position DOP, 0.01 (lower = better geometry)
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
  uint8_t  fixType;       // 0 = none, 2 = 2D, 3 = 3D
  uint8_t  numSV;         // satellites used in the solution
  bool     timeValid;     // validDate and validTime both set
  bool     valid;         // gnssFixOK and fixType >= 2
};

/*
  Bus config for the u-blox M8 (SAM-M8Q) GPS. The module speaks UBX over its
  DDC port; there is no GPS enable line and no UART. The SDA/SCL pins live in
  the project's config.h.
*/
static constexpr uint8_t  GPS_I2C_ADDR                   {0x42};
// Every GPS cost in this driver is bus time: one byte is 9 bit periods, so 90 us at
// 100 kHz and 22.5 at 400. The capture loop's timing buckets put GPS at 285 ms/s - 28 %
// of the CPU - at 100 kHz, measured 2026-08-15, and this is one of the two factors
// behind that (the other is the NMEA traffic begin() now switches off).
//
// 400 kHz is what the SAM-M8Q's DDC port is rated for, and the module is alone on the
// bus here: the temperature sensor is OneWire, and the only other Wire user in the repo
// (common_libraries/gsm) is linked into orb_basestation only.
//
// The one thing this depends on that the code cannot see is the pull-up resistors - the
// rise time at 400 kHz is four times tighter. If they are too weak the failure is loud
// rather than silent: corrupt bytes fail the checksum in readPvtFrame, freshFix() stops
// coming true, and begin()'s bus scan reports it at boot. Drop back to 100000 alone.
static constexpr uint32_t GPS_I2C_CLOCK                  {400000};  // DDC max
static constexpr uint16_t GPS_nav_rate_hz                {10};
static constexpr uint16_t GPS_nav_period_ms              {1000 / GPS_nav_rate_hz};
static constexpr uint16_t GPS_pvt_frame_size             {100};     // header 6 + payload 92 + checksum 2
static constexpr uint32_t GPS_probe_timeout              {3000};    // how long begin() retries the MON-VER probe


/*
  Wire size of one coordinate
*/
static constexpr size_t gps_coord_field_size {1 + sizeof(int32_t)};

// 'U','R' + timestamp + lat + lng + WiO_ID + 'E'
static constexpr uint32_t beaconMsgSize  {3 + sizeof(time_t) + sizeof(uint32_t) + 2*gps_coord_field_size};
// 'G' + readingID + lat + lng + vel + direction + timestamp + 'E'
static constexpr uint8_t GPS_message_size {2 + sizeof(uint16_t) + 2*gps_coord_field_size \
                                           + 2*sizeof(int32_t) + sizeof(time_t)};
// 'U','I' + buoy_ID + timestamp + lat + lng + 'E'
static constexpr uint8_t deployment_message_size {2 + sizeof(uint32_t) + sizeof(time_t) \
                                                  + 2*gps_coord_field_size + 1};
class GPS_Manager{
  public:
    GPS_Manager(uint32_t sdapin, uint32_t sclpin) : sda_pin(sdapin), scl_pin(sclpin) {};
    bool fix = false;
    void begin(void);
    DateInfo date = {0,0,0,false};
    etl::deque<GPS_Data, max_number_of_measurements> GPSReadings;

    uint8_t setTimeFromGps();
    uint8_t updateTimestamp(uint32_t max_wait_time, bool refreshGPStime);
    void getDeploymentMessage(uint32_t buoy_ID);
    time_t timestamp = 0;
    uint8_t performNReadings(uint8_t N, uint32_t max_wait_time, bool logEveryReading);
    void shutdownGPS(void);
    void processReadings(bool);
    uint32_t iterations = 0;

    size_t updateTransmitMessage(void);
    byte msgB[GPS_message_size];
    byte deploymentMessage[deployment_message_size];
    uint8_t logReading(GPS_Data & reading);
    uint32_t current_buoy_velocity = 0;
    void getMeasurementFromFile(void);
    GPS_Data currentPosition;
    void updateBeaconMsg(uint32_t WiO_ID);
    byte beaconMsg[beaconMsgSize];

    /*
      Non-blocking NAV-PVT polling (ported from ORB_test Gps::update). Runs an
      IDLE/WAIT state machine: sends a poll every GPS_nav_period_ms and reads the
      answer back without blocking, so it can be called from a tight loop (e.g.
      the wave-capture IMU drain) without starving the IMU FIFO. Sets freshFix()
      true only on the call where a new PVT was decoded (one-shot). Leaves the
      blocking pollPVT()/performNReadings() path untouched.
    */
    void update(void);
    bool freshFix(void) const { return freshFix_; };

    // True between a successful begin() and the next shutdownGPS(). update() is a
    // no-op outside that window, so a caller that waits on a fix (wave_manager)
    // can give up immediately instead of polling a receiver that cannot answer.
    bool ready(void) const { return initialized; };

    // Latest decoded solution, for fix quality checks and debug output
    const UBX_PVT & lastFix(void) const { return pvt; };
  private:
    uint8_t getGPSData(uint32_t max_wait_time);

    /*
      UBX transport over the DDC (I2C) port. availBytes reads the 0xFD/0xFE
      byte count registers, pollPVT sends a NAV-PVT poll and blocks until the
      answer has been read back and decoded. readPvtFrame drains and decodes one
      already-available frame (shared by pollPVT and the non-blocking update).
    */
    uint16_t availBytes(void);
    void flushDDC(uint16_t nbytes);
    bool pollPVT(uint32_t max_wait_time);
    bool readPvtFrame(uint16_t avail);
    void decodePVT(const uint8_t * payload);

    // Non-blocking poll state (see update()).
    enum GpsPollState : uint8_t { GPS_POLL_IDLE, GPS_POLL_WAIT };
    GpsPollState pollState_ = GPS_POLL_IDLE;
    uint32_t lastPollMs_ = 0;
    uint32_t pollSentMs_ = 0;
    bool freshFix_ = false;

    etl::vector<GPS_Data, readings_per_measurement> packet;
    uint32_t sda_pin;
    uint32_t scl_pin;
    UBX_PVT pvt = {};
    bool initialized = false;
    uint64_t initial_timestamp = 0;
    GPS_Data mean_values;
};

extern GPS_Manager gps_manager;
#endif
