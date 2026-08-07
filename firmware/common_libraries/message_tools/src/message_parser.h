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

class Message_Parser{
    public:
        temperature_Reading parse_temperature_message(byte* msg);
        GPS_Reading parse_gps_message(byte* msg);
        analog_Reading parse_analog_message(byte* msg);
        wave_analysis_Reading parse_wave_analysis_message(byte* msg);
        buoyInfoReading parse_buoy_info_message(byte* msg);
        buoyInitMessage parse_buoy_init_message(byte* msg);
        beacon_Reading parse_beacon_message(byte *msg);
};

extern Message_Parser MESSAGE_PARSER;
#endif