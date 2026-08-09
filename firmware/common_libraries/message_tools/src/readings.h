// #include "etl/deque.h"
#ifndef READINGS_H
#define READINGS_H

#include <Arduino.h>
#include "config.h"

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

/* Scaling factor when sending frequencies over LoRa
   uint32 represents up to around 4e9, so with scaling factor 1e8
   we can send 4e9 / 1e8 = 40 Hz with 8 decimals.

   This will be enough precision when casting to float later, since
   float has 1 sign bit, 8 exponent bits and 23 mantissa bits
   So at at right under 4Hz we can represent numbers with spacing 2 * 1/2^23 = 2.38e-7
   i.e. around 6 decimals of precision.
*/
static constexpr uint32_t wave_freq_scale {10000000UL};

struct wave_analysis_Reading
{
    uint16_t reading_ID;
    time_t timestamp_start;
    time_t timestamp_end;
    uint32_t Hs;
    uint32_t Tc;
    uint32_t Tp;
    uint32_t Tz;
    uint32_t max_value; // Maximum value of the PSD, used for normalizing the spectrum
    uint32_t spec_f_min; // Bin centre of the first bin, scaled with wave_freq_scale
    uint32_t spec_f_max; // Bin centre of the last bin, scaled with wave_freq_scale
    uint16_t num_bins; // Number of bins in the array, must be smaller than welch_bins
    uint16_t wave_spectrum[welch_bins]; // normalised spectrum (value/65535 * max_value = absolute PSD value)
};

/*
  On-wire size of a wave-analysis message, framed 'W' ... 'E'. Must match the byte
  layout produced by WaveManager::updateTransmitMessage and consumed by
  Message_Parser::parse_wave_analysis_message

  The spectrum is last on purpose: it is the only variable-length field, so a sender
  that has fewer bins to ship - or none at all - simply stops earlier and everything
  before it keeps its fixed offset.
*/
static constexpr uint16_t wave_message_size =
    1 + sizeof(uint16_t) + 2 * sizeof(time_t) + 7 * sizeof(uint32_t)
    + sizeof(uint16_t) + welch_bins * sizeof(uint16_t) + 1;

static_assert(wave_message_size <= max_message_length,
    "wave_message_size exceeds the LoRa byte-message buffer (max_message_length) - "
    "lower welch_bins in common_config.h; the SX126x payload length field is one byte, "
    "so 255 is a hard ceiling and welch_bins cannot exceed 102");

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