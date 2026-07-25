// #include "etl/deque.h"
#ifndef READINGS_H
#define READINGS_H

#include <Arduino.h>
#include "config.h"

// typedef byte BuoyID[8];

static constexpr uint8_t ANALOG_READING_TYPE_UNKNOWN   {0};
static constexpr uint8_t ANALOG_READING_TYPE_PH        {1};
static constexpr uint8_t ANALOG_READING_TYPE_TURBIDITY {2};

struct temperature_Reading
{
    uint16_t reading_ID;
    uint8_t num_sensors;
    int32_t temps[max_number_of_thermometres] = {0};
    time_t timestamp;
};

struct GPS_Reading
{   
    uint16_t reading_ID;
    int32_t lat;
    int32_t lng;
    int32_t vel;
    int32_t direction;
    time_t timestamp;
};

struct beacon_Reading
{   
    time_t timestamp;
    int32_t lat;
    int32_t lng;
    uint32_t buoy_id;
};

struct analog_Reading
{
    uint8_t  analogReadingType;
    uint32_t voltage;
    uint16_t measurementID;
    time_t timestamp;
};

struct wave_analysis_Reading
{
    uint16_t reading_ID;
    uint32_t Hs;
    uint32_t Tc;
    uint32_t Tp;
    uint32_t Tz;
    uint32_t max_value;
    uint16_t wave_spectrum[welch_bins];
    time_t timestamp_start;
    time_t timestamp_end;
};

/*
  On-wire size of a wave-analysis message, framed 'W' ... 'E'. Must match the byte
  layout produced by WaveManager::updateTransmitMessage and consumed by
  Message_Parser::parse_wave_analysis_message: tag + reading_ID(u16) +
  {Hs,Tc,Tp,Tz,max_value}(5x u32) + wave_spectrum(welch_bins x u16) +
  {timestamp_start,timestamp_end}(2x time_t) + trailing 'E'.
*/
static constexpr uint8_t wave_message_size =
    1 + sizeof(uint16_t) + 5 * sizeof(uint32_t)
    + welch_bins * sizeof(uint16_t) + 2 * sizeof(time_t) + 1;
static_assert(wave_message_size <= max_message_length,
    "wave_message_size exceeds the LoRa byte-message buffer (max_message_length)");

struct buoyInfoReading
{
    int32_t sent_packets;
    int32_t left_packets;
    uint32_t listen_time;
    bool crashed;
};

struct buoyInitMessage
{
    uint32_t buoy_id;
    uint8_t base_station_ID;
};

#endif