#ifndef MESSAGE_PARSER_H
#define MESSAGE_PARSER_H
#include <TimeLib.h>
#include <Arduino.h>
#include "readings.h"

/*
  A device that receives wave messages must be able to hold one first: parsing reads
  wave_message_size bytes out of a ByteMessage, which is max_message_length long.

  Only the base station does, so only it is asserted. lora_transceiver.h pulls this
  header into the drifter build too, and the drifter's max_message_length is
  deliberately smaller (it neither sends nor receives wave messages here) - asserting
  unconditionally would force RAM onto a target that has no use for it.
*/
static_assert(WIO_MODE != BST_MODE || wave_message_size <= max_message_length,
    "wave_message_size exceeds the LoRa byte-message buffer (max_message_length)");

static_assert(WIO_MODE != BST_MODE || wave_spectrum_message_size <= max_message_length,
    "wave_spectrum_message_size exceeds the LoRa byte-message buffer "
    "(max_message_length) - lower welch_bins in config.h; the SX126x payload length "
    "field is one byte, so 255 is a hard ceiling and with this header welch_bins "
    "cannot exceed 114");

class Message_Parser{
    public:
        temperature_Reading parse_temperature_message(byte* msg);
        GPS_Reading parse_gps_message(byte* msg);
        analog_Reading parse_analog_message(byte* msg);
        wave_analysis_Reading parse_wave_analysis_message(byte* msg);
        wave_spectrum_Reading parse_wave_spectrum_message(byte* msg);
        buoyInfoReading parse_buoy_info_message(byte* msg);
        buoyInitMessage parse_buoy_init_message(byte* msg);
        beacon_Reading parse_beacon_message(byte *msg);
};

extern Message_Parser MESSAGE_PARSER;

/*
  Print a decoded wave message to the debug console (and the debug file, when one is
  open). One printer per message, because the two carry different fields.

  Shared deliberately: the base station prints these on reception, and the drifter's
  DEBUG_WAVE_MSG bench build prints the same thing before transmitting, so the two
  consoles can be diffed line for line to tell a codec fault from a radio fault. That
  only works if there is exactly one implementation - two approximately equal copies
  would drift and quietly invalidate the comparison.
*/
void print_wave_analysis_message(const wave_analysis_Reading &w);
void print_wave_spectrum_message(const wave_spectrum_Reading &w);
#endif