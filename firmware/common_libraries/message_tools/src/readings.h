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
    uint32_t vel;
    uint32_t direction;
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

/* Scaling factor when sending frequencies over LoRa.
   uint32 holds up to ~4.29e9, so 1e7 gives up to ~429 Hz with 7 decimals,
   finer than float resolution (~2.4e-7 just below 4 Hz).
*/
static constexpr uint32_t wave_freq_scale {10000000UL};

/* Scaling factor for max_value, the peak of the acceleration PSD in (m/s^2)^2/Hz.
   1e8 gives a resolution of 1e-8 (a buoy at rest is ~1e-6) and a ceiling of 42.9.
   The sender clamps above that. */
static constexpr uint32_t wave_psd_scale {100000000UL};

/*
  A wave measurement is sent as two messages:

    'W'  parameters: time, position, Hs/Tc/Tp/Tz        - 56 bytes
    'P'  spectrum:   max_value, frequency axis, bins     - 26 + 2*num_bins

  They are joined on (buoy, ts_start), not reading_ID, since reading_ID restarts
  at 0 on every reboot. ts_start is therefore in both messages.
*/
struct wave_analysis_Reading
{
    uint16_t reading_ID;
    time_t timestamp_start;
    time_t timestamp_end;
    uint32_t Hs;
    uint32_t Tc;
    uint32_t Tp;
    uint32_t Tz;
    /*
      Position at the start and end of the analysis window, 1e-7 deg
      (gps_coord_scale), sign-and-magnitude like the 'G' message. 0,0 means no fix.
    */
    int32_t lat_start;
    int32_t lng_start;
    int32_t lat_end;
    int32_t lng_end;
};

/*
  The spectrum, as its own message. Carries ts_start so it can find its parameters
  - see the join-key note above - and reading_ID so a human can match the two in a
  log.
*/
struct wave_spectrum_Reading
{
    uint16_t reading_ID;
    time_t timestamp_start;    // the join key, same value and width as the 'W' message
    /*
      Vertical acceleration PSD, untapered. Bins below kPsdMinFreq are not sent;
      max_value is the peak over the sent bins.

      Bin centres: f_i = spec_f_min + i*(spec_f_max - spec_f_min)/(num_bins - 1).

      Bins are sqrt-companded: stored as sqrt(bin/peak) * 65535, so
      PSD = (value/65535)^2 * max_value. Elevation is S_acc/(2*pi*f)^4.
    */
    uint32_t max_value; // Peak of the acceleration PSD, scaled with wave_psd_scale
    uint32_t spec_f_min; // Bin centre of the first bin, scaled with wave_freq_scale
    uint32_t spec_f_max; // Bin centre of the last bin, scaled with wave_freq_scale
    uint16_t num_bins; // Number of bins in the array, must be smaller than welch_bins
    uint16_t wave_spectrum[welch_bins]; // sqrt-companded spectrum ((value/65535)^2 * max_value = absolute PSD value)
};

/*
  On-wire size of the 'W' message. Must match WaveManager::updateTransmitMessage and
  Message_Parser::parse_wave_analysis_message. Coordinates are 5 bytes (sign + int32).
*/
static constexpr uint16_t wave_message_size =
    1 + sizeof(uint16_t) + 2 * sizeof(time_t) + 4 * sizeof(uint32_t)
    + 4 * (1 + sizeof(int32_t)) + 1;

static_assert(wave_message_size <= max_message_length,
    "wave_message_size exceeds the LoRa byte-message buffer (max_message_length)");

/*
  On-wire size of the 'P' message. The bins are last, since they are the only
  variable-length field.
*/
static constexpr uint16_t wave_spectrum_message_size =
    1 + sizeof(uint16_t) + sizeof(time_t) + 3 * sizeof(uint32_t)
    + sizeof(uint16_t) + welch_bins * sizeof(uint16_t) + 1;

static_assert(wave_spectrum_message_size <= max_message_length,
    "wave_spectrum_message_size exceeds the LoRa byte-message buffer "
    "(max_message_length) - lower welch_bins in common_config.h; the SX126x payload "
    "length field is one byte, so 255 is a hard ceiling and with this header "
    "welch_bins cannot exceed 114");

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