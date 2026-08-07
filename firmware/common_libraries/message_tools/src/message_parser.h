#ifndef MESSAGE_PARSER_H
#define MESSAGE_PARSER_H
#include <TimeLib.h>
#include <Arduino.h>
#include "readings.h"

class Message_Parser{
    public:
        temperature_Reading parse_temperature_message(byte* msg);
        GPS_Reading parse_gps_message(byte* msg);
        analog_Reading parse_analog_message(byte* msg);
        wave_analysis_Reading parse_wave_analysis_message(byte* msg);
        buoyInfoReading parse_buoy_info_message(byte* msg);
        buoyInitMessage parse_buoy_init_message(byte* msg);
        beacon_Reading parse_beacon_message(byte *msg);

        /*
          Debug rendering of a decoded reading, one printer per parser above. They live
          here rather than at the call site so a field added to a struct in readings.h
          has its parser and its printout within a screen of each other - the pairs used
          to sit in different files and drifted apart.

          All of them write through sd_writer, so output reaches the serial console and
          the debug file exactly as the rest of the base station's logging does.
        */
        void print_gps_reading(const GPS_Reading & r);
        void print_temperature_reading(const temperature_Reading & r);
        void print_wave_analysis_reading(const wave_analysis_Reading & r, float rssi);
        void print_beacon_reading(const beacon_Reading & r);
        void print_buoy_info_reading(const buoyInfoReading & r);
};

extern Message_Parser MESSAGE_PARSER;
#endif