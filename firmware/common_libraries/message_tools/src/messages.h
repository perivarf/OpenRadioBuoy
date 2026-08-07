#ifndef MESSAGES_H
#define MESSAGES_H

#include "config.h"
#include <TimeLib.h>

static const uint32_t byte_message_size = max_message_length;

struct ByteMessage{
  byte byteMsg[byte_message_size];
  int numBytes;
  bool success;
};

struct BuoyMessage{
    ByteMessage* byteMsg;
    float rssi;
};

struct FrequencyMessage{
    bool update_frequency;
    uint32_t measurement_frequency;
    bool adaptive_frequency;
    uint32_t target_length;
    uint32_t threshold_velocity;
};

struct BeaconIncomingMessage{
    bool enable_rescue_mode;
    time_t timeout_rescue_mode;
};

struct BeaconOutgoingMessage{
    float rssi;
    uint32_t buoy_id;
    int32_t lat;
    int32_t lng;
    time_t timestamp;
};



struct StringMessage{
  String msg; // TODO: replace more meaningful struct
  bool success;
};



/*
  Signal quality of a link, in units of 1e4 * dB. Zero-initialised on purpose:
  sendData only fills these in when the TxDone flag arrived, so a plain
  `Message_Data m;` would otherwise be logged as stack garbage on a timeout.
  Note that both are properties of the last RECEIVED packet - after a transmit
  with no answer they are legitimately 0.
*/
struct Message_Data{
  uint32_t RSSI{0};
  uint32_t SNR{0};
};

struct buoyInfo{
  uint32_t ID;
  bool inrange;
};
#endif