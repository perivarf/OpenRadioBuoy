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
  uint32_t iTOW_ms;       // GPS time of week of the solution, ms. The receiver's own epoch
                          // clock, so a log carrying it has a time base that does not depend
                          // on when the firmware got round to reading the frame.
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
// 400 kHz is the SAM-M8Q DDC maximum, and the module is alone on the bus. GPS cost is
// bus time, so this is 4x cheaper than 100 kHz. Depends on the pull-ups: if they are
// too weak, frames fail the checksum - drop back to 100000.
static constexpr uint32_t GPS_I2C_CLOCK                  {400000};  // DDC max
static constexpr uint16_t GPS_nav_rate_hz                {10};
static constexpr uint16_t GPS_nav_period_ms              {1000 / GPS_nav_rate_hz};
static constexpr uint16_t GPS_pvt_frame_size             {100};     // header 6 + payload 92 + checksum 2
static constexpr uint32_t GPS_probe_timeout              {3000};    // how long begin() retries the MON-VER probe

/*
  update() checks the DDC buffer GPS_ddc_check_div times per navigation epoch. A late
  check loses nothing, the frame waits in the module's buffer.
*/
static constexpr uint8_t  GPS_ddc_check_div              {4};
static constexpr uint16_t GPS_ddc_check_ms               {GPS_nav_period_ms / GPS_ddc_check_div};

/*
  Max bytes one update() call drains (~3 ms at 400 kHz). Only matters for a backlog
  after an SD stall. A frame cut short is completed on the next call.
*/
static constexpr uint16_t GPS_ddc_max_drain              {128};

// No valid frame for ten navigation epochs means the bus or the module needs a nudge; see
// recoverDdc(). Long enough that a normal SD stall (measured max 878 ms) never trips it.
// Counted only across epochs update() was there to see - a gap between CALLS this long
// rebases the timer instead of tripping it (see update()).
static constexpr uint16_t GPS_stall_timeout_ms           {10 * GPS_nav_period_ms};


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
      Non-blocking NAV-PVT reader, call once per loop iteration. Rate-limits itself
      to GPS_ddc_check_ms and never waits. Does not poll: begin() sets the module to
      periodic output. freshFix() is true only on the call that decoded a new PVT.
    */
    void update(void);
    bool freshFix(void) const { return freshFix_; };

    // How many times recoverDdc() has had to reset the bus. Worth writing to ses.csv when
    // chasing whether the DDC still latches up at GPS_I2C_CLOCK.
    uint32_t recoverCount(void) const { return recoverCount_; };

    // True between a successful begin() and the next shutdownGPS(). update() is a
    // no-op outside that window, so a caller that waits on a fix (wave_manager)
    // can give up immediately instead of polling a receiver that cannot answer.
    bool ready(void) const { return initialized; };

    // Latest decoded solution, for fix quality checks and debug output
    const UBX_PVT & lastFix(void) const { return pvt; };
  private:
    uint8_t getGPSData(uint32_t max_wait_time);

    /*
      UBX over DDC (I2C). availBytes reads the byte count, and `ok` separates a bus
      error from an empty buffer. pollPVT polls and blocks; readPvtFrame decodes
      what is waiting (shared by pollPVT and update).
    */
    uint16_t availBytes(bool * ok = nullptr);
    void flushDDC(uint16_t nbytes);
    bool pollPVT(uint32_t max_wait_time);
    // maxBytes caps one drain so update() cannot block the IMU FIFO behind a backlog. The
    // default drains everything available, which is what the blocking pollPVT path wants:
    // capping it there would leave more in the buffer than the next call takes out, since
    // that path adds a poll response of its own on every iteration.
    bool readPvtFrame(uint16_t avail, uint16_t maxBytes = 0xFFFF);
    void decodePVT(const uint8_t * payload);
    void recoverDdc(uint32_t nowMs);

    /*
      Parser state for readPvtFrame, kept across calls so a frame split between
      two drains is completed instead of discarded.
    */
    uint8_t  frameBuf_[GPS_pvt_frame_size];
    uint16_t frameIdx_ = 0;

    // Non-blocking reader state (see update()).
    uint32_t lastCheckMs_ = 0;    // last time the DDC length register was read
    uint32_t lastFixMs_ = 0;      // last time a frame decoded; drives the stall timeout
    uint32_t recoverCount_ = 0;
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
