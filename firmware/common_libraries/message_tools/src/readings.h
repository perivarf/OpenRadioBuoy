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

/* Scaling factor when sending frequencies over LoRa
   uint32 represents up to around 4e9, so with scaling factor 1e8
   we can send 4e9 / 1e8 = 40 Hz with 8 decimals.

   This will be enough precision when casting to float later, since
   float has 1 sign bit, 8 exponent bits and 23 mantissa bits
   So at at right under 4Hz we can represent numbers with spacing 2 * 1/2^23 = 2.38e-7
   i.e. around 6 decimals of precision.
*/
static constexpr uint32_t wave_freq_scale {10000000UL};

/* Scaling factor for max_value, the acceleration-PSD peak the spectrum is
   normalised against. It needs its own scale for the same reason the frequency
   axis does - scale_factor's 1e5 cannot express it at all.

   An acceleration PSD in (m/s^2)^2/Hz spans an enormous range: a buoy at rest on
   a bench sits on the accelerometer's noise floor at ~6e-7, four decades BELOW
   scale_factor's smallest step, so the field quantised to 0 and every receiver
   reconstructed value/65535 * 0 = a spectrum of zeros. The shape was on the wire
   the whole time; only its scale was lost.

   1e8 puts the resolution at 1e-8, two decades under that noise floor, and the
   uint32 ceiling at 4294967295/1e8 = 42.9 (m/s^2)^2/Hz. For scale: Hs 1 m at
   Tp 5 s peaks near 3, Hs 10 m at Tp 12 s near 15. Beyond that the sender clamps
   rather than wraps - a saturated peak reads as a spectrum that is too flat, not
   as noise, and Hs/Tz/Tc/Tp are transmitted as their own fields and stay right
   regardless.

   Replaying the field captures through this format put the largest observed peak at
   1.85 (m/s^2)^2/Hz - a factor 23 under the ceiling - and the quietest, a buoy on a
   bench, at 1e-6, which is 100 counts here and would have been 0 at 1e5. */
static constexpr uint32_t wave_psd_scale {100000000UL};

/*
  A WAVE MEASUREMENT TRAVELS AS TWO MESSAGES.

    'W'  the parameters: when, where, Hs/Tc/Tp/Tz          - 56 bytes
    'P'  the spectrum:   max_value, the axis, the bins     - 26 + 2*num_bins

  They were one message until the single packet hit 254 of the 255 bytes the
  SX126x payload-length field can express, which left nothing for a position and
  made kPsdMaxFreq a payload question rather than a Nyquist one. Splitting costs
  about 15 % more airtime for the pair, and buys three things:

    - The parameters get their own short packet. Hs and the periods are what the
      measurement is FOR, and they now ride in ~56 bytes instead of sharing a
      250-byte packet with the spectrum - a far smaller target at the same link
      margin. A lost 'P' now costs the spectrum alone, where a lost combined
      message cost everything.
    - A position fits. The measurement becomes self-describing instead of relying
      on a separate 'G' message being received and matched up afterwards.
    - 'P' holds up to 114 bins in 255 bytes against 98 before, so the band is
      limited by the analysis, not by the frame.

  THE JOIN KEY IS (buoy, ts_start), NOT reading_ID. reading_ID is a counter that
  restarts from 0 on every reboot (readingID_ in wave_manager.h), so two sessions
  both produce reading 1, 2, 3 and the spectrum from one would attach to the
  parameters of the other. ts_start is therefore repeated in BOTH messages: it is
  already the primary key the base station's storage uses, it makes the pair
  order-independent - whichever arrives first creates the record - and it lets a
  'P' whose 'W' was lost still be placed in time. reading_ID stays as a
  human-readable label, which is all it was ever reliable enough to be.
*/
struct wave_analysis_Reading
{
    uint16_t reading_ID;
    /*
      Full 8-byte time_t, the natural width every other message here serialises at.
      Four would hold an epoch until 2106 and would save eight bytes, but the split
      is what made those bytes cheap: this message is 56 of the 255 a packet can
      carry, so there is nothing to buy with them. A format that cannot express a
      date is a poor trade for slack nobody needs.
    */
    time_t timestamp_start;
    time_t timestamp_end;
    uint32_t Hs;
    uint32_t Tc;
    uint32_t Tp;
    uint32_t Tz;
    /*
      Position at the START and at the END of the analysis window, 1e-7 deg
      (gps_coord_scale), sign-and-magnitude on the wire like the 'G' message.

      Two, not one: over a 30-minute window a free-drifting buoy moves, and the
      pair is the drift vector - surface current, which is a measurement rather
      than metadata. For placing the record on a map either will do.

      0,0 means NO FIX, which is the convention the rest of the system already
      uses - a real 0,0 is in the Gulf of Guinea and the dashboard keeps such
      points off the map deliberately.
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
      The spectrum is the vertical ACCELERATION PSD with no taper applied.

      The transmitted band is a SLICE, bins below kPsdMinFreq is dropped.
      max_value is the peak over the  bins.

      The bin mid frequencies are
      f_i = spec_f_min + i*(spec_f_max - spec_f_min)/(num_bins - 1).

      Each bin is SQRT-COMPANDED: the sender stores sqrt(bin/peak) * 65535, so the
      absolute PSD is (value/65535)^2 * max_value. Acceleration, not elevation, is what
      travels: elevation is S_acc/(2*pi*f)^4, exact arithmetic on this side, which
      leaves the receiver free to choose its own taper and low-frequency cut. Sending
      elevation instead was measured and rejected - untapered it peaks in the BOTTOM bin
      in every field capture, so max_value would be omega^-4-amplified drift rather than
      a wave measurement.
    */
    uint32_t max_value; // Peak of the acceleration PSD, scaled with wave_psd_scale
    uint32_t spec_f_min; // Bin centre of the first bin, scaled with wave_freq_scale
    uint32_t spec_f_max; // Bin centre of the last bin, scaled with wave_freq_scale
    uint16_t num_bins; // Number of bins in the array, must be smaller than welch_bins
    uint16_t wave_spectrum[welch_bins]; // sqrt-companded spectrum ((value/65535)^2 * max_value = absolute PSD value)
};

/*
  On-wire size of the parameter message, framed 'W' ... 'E'. Must match the byte
  layout produced by WaveManager::updateTransmitMessage and consumed by
  Message_Parser::parse_wave_analysis_message.

  Coordinates cost five bytes each, not four: msg_insert_int writes a 'P'/'N' sign
  character and then the magnitude, which is the encoding the 'G' message already
  uses. Four would fit an int32 and save 4 bytes, but this message has room and a
  receiver that decodes every coordinate the same way is worth more than that.
*/
static constexpr uint16_t wave_message_size =
    1 + sizeof(uint16_t) + 2 * sizeof(time_t) + 4 * sizeof(uint32_t)
    + 4 * (1 + sizeof(int32_t)) + 1;

static_assert(wave_message_size <= max_message_length,
    "wave_message_size exceeds the LoRa byte-message buffer (max_message_length)");

/*
  On-wire size of the spectrum message, framed 'P' ... 'E'.

  The spectrum is last on purpose: it is the only variable-length field, so a sender
  that has fewer bins to ship - or none at all - simply stops earlier and everything
  before it keeps its fixed offset.
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