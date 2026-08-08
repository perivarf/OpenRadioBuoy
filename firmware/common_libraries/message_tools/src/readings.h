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

/*
  Fixed-point scale for the frequency axis below - NOT scale_factor.

  scale_factor (1e5) is too coarse here. It quantises the first bin centre 0.0048828
  to 0.00488 and the last to 0.98145, and after rebuilding the step from that pair the
  top bins land within 5e-6 Hz of a 4-decimal rounding boundary: the sender prints
  0.9814 and the receiver 0.9815 for the same bin, which defeats the whole point of
  the two consoles being diffable. At 1e7 the round trip is exact to well below the
  printed resolution and the reconstructed step comes back as 0.01953125 on the nose.

  Headroom is not a concern: u32 at this scale reaches 429 Hz against a 10 Hz series.
*/
static constexpr uint32_t wave_freq_scale {10000000UL};

// Field order mirrors the wire, and the wire puts every fixed-size field first - see
// wave_message_size below for why the spectrum has to be last.
struct wave_analysis_Reading
{
    uint16_t reading_ID;
    time_t timestamp_start;
    time_t timestamp_end;
    uint32_t Hs;
    uint32_t Tc;
    uint32_t Tp;
    uint32_t Tz;
    uint32_t max_value;
    /*
      Frequency axis of the spectrum below, carried in the message so this end is
      self-sufficient: it has no sample rate or segment length to derive f = k*fs/N
      from, and a bin index alone says nothing physical.

      Both are BIN CENTRES, fixed-point by wave_freq_scale above, and num_bins is
      the count actually on the wire. The step follows:

          df   = (spec_f_max - spec_f_min) / (num_bins - 1)
          f_j  = spec_f_min + j * df

      num_bins may be smaller than welch_bins - the array is a capacity bound now,
      not a parse contract - so always loop over num_bins, never welch_bins.
    */
    uint32_t spec_f_min;
    uint32_t spec_f_max;
    uint16_t num_bins;
    uint16_t wave_spectrum[welch_bins];
};

/*
  On-wire size of a wave-analysis message, framed 'W' ... 'E'. Must match the byte
  layout produced by the drifter and consumed by
  Message_Parser::parse_wave_analysis_message: tag + reading_ID(u16) +
  {timestamp_start,timestamp_end}(2x u32) + {Hs,Tc,Tp,Tz,max_value}(5x u32) +
  {spec_f_min,spec_f_max}(2x u32) + num_bins(u16) + wave_spectrum(num_bins x u16)
  + trailing 'E'.

  The spectrum is LAST on purpose: it is the only variable-length field, so a sender
  that has fewer bins to ship - or none at all - simply stops earlier and everything
  before it keeps its fixed offset. With the timestamps behind the spectrum instead,
  every reader had to walk all num_bins just to reach them, which made a truncated
  message unreadable rather than merely shorter.

  This constant is the size at the FULL bin count, i.e. the largest such message and
  what the send buffer must hold. The receiver takes the actual length from num_bins.

  The timestamps are uint32_t ON THE WIRE, deliberately not sizeof(time_t): time_t is
  8 bytes in this toolchain, and the serialiser has always cast them down to 4. Sizing
  or parsing them as time_t reads 8 bytes where 4 were written and shifts every field
  behind them - which is invisible while they sit last in the message and corrupts
  everything once they do not. An epoch fits in uint32 until 2106.

  Whether it fits in a given device's ByteMessage buffer is asserted in
  message_parser.h, i.e. only for the targets that actually handle wave messages -
  this header is also pulled in (via lora_transceiver.h) by the drifter, whose
  max_message_length is smaller and which neither sends nor receives them.
*/
static constexpr uint8_t wave_message_size =
    1 + sizeof(uint16_t) + 2 * sizeof(uint32_t) + 7 * sizeof(uint32_t)
    + sizeof(uint16_t) + welch_bins * sizeof(uint16_t) + 1;

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