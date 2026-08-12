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

/*
  Units, matching GPS_Data on the drifter side:
    lat, lng   1e-7 deg   (gps_coord_scale, the receiver's native resolution)
    vel        m/s * scale_factor
    direction  deg * scale_factor

  lat/lng are deliberately not scale_factor-scaled: 5 decimals is ~1.1 m, and the
  raw 1e-7 deg value fits an int32 as it stands. They are also the only signed
  fields, and the only ones carrying a 'P'/'N' sign char on the wire - speed and
  course over ground are magnitudes.
*/
struct GPS_Reading
{
    uint16_t reading_ID;
    int32_t lat;
    int32_t lng;
    uint32_t vel;
    uint32_t direction;
    time_t timestamp;
};

// lat/lng in 1e-7 deg (gps_coord_scale), sign-magnitude on the wire
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

/*
  Fixed-point scale for max_value, the acceleration-PSD peak the spectrum below is
  normalised against. Its own scale for the same reason the frequency axis has one:
  scale_factor's 1e5 cannot express it at all.

  An acceleration PSD in (m/s^2)^2/Hz spans an enormous range. A buoy at rest on a
  bench sits on the accelerometer's noise floor around 6e-7, four decades BELOW
  scale_factor's smallest step, so the field quantised to 0 and every receiver
  reconstructed a spectrum of zeros - the shape was on the wire the whole time and
  only its scale was lost.

  1e8 puts the resolution at 1e-8 and the u32 ceiling at 42.9 (m/s^2)^2/Hz. For
  scale: Hs 1 m at Tp 5 s peaks near 3, Hs 10 m at Tp 12 s near 15, and the largest
  peak in the field captures so far is 1.85 - a factor 23 under the ceiling. Beyond
  it the sender clamps rather than wraps, so a saturated peak reads as a spectrum
  that is too flat rather than as noise, and Hs/Tc/Tp/Tz travel as their own fields
  in the 'W' message and stay right regardless.
*/
static constexpr uint32_t wave_psd_scale {100000000UL};

/*
  A WAVE MEASUREMENT ARRIVES AS TWO MESSAGES.

    'W'  the parameters: when, where, Hs/Tc/Tp/Tz          - 56 bytes
    'P'  the spectrum:   max_value, the axis, the bins     - 26 + 2*num_bins

  They were one message until the single packet hit 254 of the 255 bytes the SX126x
  payload-length field can express, which left no room for a position and made the
  transmitted band a payload question rather than a Nyquist one. The split costs
  about 15 % more airtime for the pair and buys three things: the parameters get
  their own short packet (a lost 'P' now costs the spectrum alone, where a lost
  combined message cost everything), a position fits, and 'P' holds up to 114 bins
  in 255 bytes against 98 before.

  THE JOIN KEY IS (buoy, timestamp_start), NOT reading_ID. reading_ID is a counter
  that restarts from 0 on every reboot, so two sessions both produce reading 1, 2, 3
  and the spectrum from one would attach to the parameters of the other.
  timestamp_start is therefore repeated in BOTH messages: it makes the pair
  order-independent - whichever arrives first creates the record downstream - and it
  lets a 'P' whose 'W' was lost still be placed in time. reading_ID stays as a
  human-readable label, which is all it was ever reliable enough to be.

  This base station does not pair them. It forwards bytes and prints each message on
  its own; the join happens in the cloud. Keeping it that way means a 'W' and its 'P'
  can be minutes and several packets apart without anything here having to hold state.

  Field order mirrors the wire, and the wire puts every fixed-size field first - see
  wave_spectrum_message_size below for why the spectrum has to be last.
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
      Position at the START and at the END of the analysis window, 1e-7 deg
      (gps_coord_scale), sign-and-magnitude on the wire like the 'G' message.

      Two, not one: over a 30-minute window a free-drifting buoy moves, and the pair
      is the drift vector - surface current, which is a measurement rather than
      metadata. For placing the record on a map either will do.

      0,0 means NO FIX, the convention the rest of the system already uses - a real
      0,0 is in the Gulf of Guinea, and the dashboard keeps such points off the map
      deliberately.
    */
    int32_t lat_start;
    int32_t lng_start;
    int32_t lat_end;
    int32_t lng_end;
};

/*
  The spectrum, as its own message. Carries timestamp_start so it can find its
  parameters - see the join-key note above - and reading_ID so a human can match the
  two in a log.
*/
struct wave_spectrum_Reading
{
    uint16_t reading_ID;
    time_t timestamp_start;   // the join key, same value and width as in the 'W' message
    /*
      Peak of the vertical ACCELERATION PSD, scaled by wave_psd_scale, and the value
      every bin below is normalised against.

      Acceleration, not elevation, is what travels: elevation is S_acc/(2*pi*f)^4,
      exact arithmetic on this side, which leaves the receiver free to choose its own
      taper and low-frequency cut. Sending elevation instead was measured and
      rejected - untapered it peaks in the BOTTOM bin in every field capture, so
      max_value would be omega^-4-amplified drift rather than a wave measurement.
    */
    uint32_t max_value;
    /*
      Frequency axis of the spectrum below, carried in the message so this end is
      self-sufficient: it has no sample rate or segment length to derive f = k*fs/N
      from, and a bin index alone says nothing physical.

      Both are BIN CENTRES, fixed-point by wave_freq_scale above, and num_bins is
      the count actually on the wire. The step follows:

          df   = (spec_f_max - spec_f_min) / (num_bins - 1)
          f_j  = spec_f_min + j * df

      num_bins may be smaller than welch_bins - the array is a capacity bound, not a
      parse contract - so always loop over num_bins, never welch_bins.
    */
    uint32_t spec_f_min;
    uint32_t spec_f_max;
    uint16_t num_bins;
    /*
      Each bin is SQRT-COMPANDED: the sender stores sqrt(bin/peak) * 65535, so the
      absolute PSD is (value/65535)^2 * max_value. The square is not optional. The
      normalisation peak is set by chop near 1 Hz while the wave band sits decades
      below it, and a linear u16 left the wave bins on a handful of counts - worst
      relative error on the field captures ran to 9.9 %, against 0.16 % here, for the
      same number of bytes.

      Reading it linearly does not fail loudly: it returns a spectrum that is too
      flat and too high in the wave band. A sender and a receiver from different
      builds must therefore not be mixed - the message carries no version byte.
    */
    uint16_t wave_spectrum[welch_bins];
};

/*
  On-wire size of the parameter message, framed 'W' ... 'E'. Must match the byte
  layout produced by the drifter and consumed by
  Message_Parser::parse_wave_analysis_message: tag + reading_ID(u16) +
  {timestamp_start,timestamp_end}(2x time_t) + {Hs,Tc,Tp,Tz}(4x u32) +
  {lat,lng}x{start,end}(4x sign char + i32) + trailing 'E'.

  Coordinates cost five bytes each, not four: msg_insert_int writes a 'P'/'N' sign
  character and then the magnitude, which is the encoding the 'G' message already
  uses. Four would fit an int32 and save 4 bytes, but this message has room, and a
  receiver that decodes every coordinate the same way is worth more than that.

  Sender and receiver must size time_t identically: parsing it at a different width
  than it was written shifts every field behind it.
*/
static constexpr uint16_t wave_message_size =
    1 + sizeof(uint16_t) + 2 * sizeof(time_t) + 4 * sizeof(uint32_t)
    + 4 * (1 + sizeof(int32_t)) + 1;

/*
  On-wire size of the spectrum message, framed 'P' ... 'E': tag + reading_ID(u16) +
  timestamp_start(time_t) + {max_value,spec_f_min,spec_f_max}(3x u32) + num_bins(u16)
  + wave_spectrum(num_bins x u16) + trailing 'E'.

  The spectrum is LAST on purpose: it is the only variable-length field, so a sender
  that has fewer bins to ship - or none at all - simply stops earlier and everything
  before it keeps its fixed offset. With the timestamp behind the spectrum instead,
  every reader had to walk all num_bins just to reach it, which made a truncated
  message unreadable rather than merely shorter.

  This constant is the size at the FULL bin count, i.e. the largest such message and
  what the send buffer must hold. The receiver takes the actual length from num_bins.

  Whether either size fits a given device's ByteMessage buffer is asserted in
  message_parser.h, i.e. only for the targets that actually handle wave messages.
*/
static constexpr uint16_t wave_spectrum_message_size =
    1 + sizeof(uint16_t) + sizeof(time_t) + 3 * sizeof(uint32_t)
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