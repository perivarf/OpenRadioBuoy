#include "message_parser.h"
#include "parser_utils.h"
#include "sd_writer.h"   // the printers below log through sd_writer, as the rest does

Message_Parser MESSAGE_PARSER;
// The new structure is "Tn(n*s*xxxx)ttttttttE"
temperature_Reading Message_Parser::parse_temperature_message(byte *msg)
{
    temperature_Reading temperature_reading_packet;
    uint8_t offset = 1;

    temperature_reading_packet.reading_ID = msg_extract_uint<uint16_t>(msg, offset, true, offset);

    temperature_reading_packet.num_sensors =  msg[offset];
    offset += sizeof(byte);

    for (int j = 0; j < temperature_reading_packet.num_sensors; j++)
    {
        int8_t factor = msg[offset] == 'N' ? -1 : 1;

        offset += sizeof(byte);
        temperature_reading_packet.temps[j] = msg_extract_uint<int32_t>(msg, offset, true, offset) * factor;

    }
   
    temperature_reading_packet.timestamp = msg_extract_uint<time_t>(msg, offset, true, offset);
   
    return temperature_reading_packet;
}

GPS_Reading Message_Parser::parse_gps_message(byte *msg)
{
    GPS_Reading gps_reading_packet;
    uint8_t offset = 1;
    gps_reading_packet.reading_ID = msg_extract_uint<uint16_t>(msg, offset, true, offset);
    gps_reading_packet.lat = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    gps_reading_packet.lng = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    gps_reading_packet.vel = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    gps_reading_packet.direction = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    gps_reading_packet.timestamp = msg_extract_uint<time_t>(msg, offset, true, offset);
   
    return gps_reading_packet;
}

beacon_Reading Message_Parser::parse_beacon_message(byte *msg)
{
    beacon_Reading beacon_reading_packet;
    uint8_t offset = 2;
    beacon_reading_packet.timestamp = msg_extract_uint<time_t>(msg, offset, true, offset);
    beacon_reading_packet.lat = msg_extract_uint<int32_t>(msg, offset, true, offset);
    beacon_reading_packet.lng = msg_extract_uint<int32_t>(msg, offset, true, offset);
    beacon_reading_packet.buoy_id = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    return beacon_reading_packet;

}

analog_Reading Message_Parser::parse_analog_message(byte* msg)
{
    analog_Reading analog_reading_packet;

    uint8_t offset = 1;
    analog_reading_packet.timestamp = msg_extract_uint<time_t>(msg, offset, true, offset);
    analog_reading_packet.voltage = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    return analog_reading_packet;
}

wave_analysis_Reading Message_Parser::parse_wave_analysis_message(byte *msg)
{
    wave_analysis_Reading wa_reading_packet;
    uint8_t offset = 1;

    wa_reading_packet.reading_ID      = msg_extract_uint<uint16_t>(msg, offset, true, offset);
    wa_reading_packet.timestamp_start = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.timestamp_end   = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Hs         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Tc         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Tp         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Tz         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.max_value  = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.spec_f_min = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.spec_f_max = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.num_bins   = msg_extract_uint<uint16_t>(msg, offset, true, offset);

    // Limiting the number of bins to the maximum allowed, to avoid buffer overflow
    if (wa_reading_packet.num_bins > welch_bins) wa_reading_packet.num_bins = welch_bins;
    for (uint16_t i = 0; i < wa_reading_packet.num_bins; i++)
        wa_reading_packet.wave_spectrum[i] = msg_extract_uint<uint16_t>(msg, offset, true, offset);

    return wa_reading_packet;
}

// Buoy infor structure is EMxytttteE
buoyInfoReading Message_Parser::parse_buoy_info_message(byte *msg)
{
    buoyInfoReading buoy_info_packet;
    buoy_info_packet.sent_packets = msg[2];
    buoy_info_packet.left_packets = msg[3];
    uint8_t offset = 4;
    buoy_info_packet.listen_time = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    buoy_info_packet.crashed = msg[offset];

    return buoy_info_packet;
}

buoyInitMessage Message_Parser::parse_buoy_init_message(byte *msg)
{
    buoyInitMessage buoy_init_message;

    if (msg[0] == 'U' && msg[6] == 'E')
    {
        uint8_t offset = 1;
        buoy_init_message.buoy_id = msg_extract_uint<uint32_t>(msg, offset, true, offset);
        buoy_init_message.base_station_ID = msg[offset];
    }

    return buoy_init_message;
}


/*
  ---------------------------------------------------------------------------
  Debug printers.
  ---------------------------------------------------------------------------
*/

void Message_Parser::print_gps_reading(const GPS_Reading & r)
{
    sd_writer.debugSerialPrintln("location info");
    sd_writer.debugSerialPrint("reading id: ");
    sd_writer.debugSerialPrint(r.reading_ID);
    sd_writer.debugSerialPrint(", lat: ");
    sd_writer.debugSerialPrint(r.lat);
    sd_writer.debugSerialPrint(", lng: ");
    sd_writer.debugSerialPrint(r.lng);
    sd_writer.debugSerialPrint(", vel: ");
    sd_writer.debugSerialPrint(r.vel);
    sd_writer.debugSerialPrint(", direction: ");
    sd_writer.debugSerialPrint(r.direction);
    sd_writer.debugSerialPrint(", timestamp: ");
    sd_writer.debugSerialPrintln(r.timestamp);
}


void Message_Parser::print_temperature_reading(const temperature_Reading & r)
{
    sd_writer.debugSerialPrintln("temperature info");
    sd_writer.debugSerialPrint("reading id: ");
    sd_writer.debugSerialPrint(r.reading_ID);
    sd_writer.debugSerialPrint(", ");
    for (int i = 0; i < max_number_of_thermometres; i++){
      sd_writer.debugSerialPrint("temperature sensor ");
      sd_writer.debugSerialPrint(i);
      sd_writer.debugSerialPrint(": ");
      sd_writer.debugSerialPrint(r.temps[i]);
      sd_writer.debugSerialPrint(", ");
    }
    sd_writer.debugSerialPrint(", timestamp: ");
    sd_writer.debugSerialPrintln(r.timestamp);
}


void Message_Parser::print_wave_analysis_reading(const wave_analysis_Reading & r, float rssi)
{
    const float max_value = (float)r.max_value / scale_factor;

    sd_writer.debugSerialPrint("  wave result #");
    sd_writer.debugSerialPrintln((float)r.reading_ID, 0);
    sd_writer.debugSerialPrint("    Hs ");
    sd_writer.debugSerialPrint((float)r.Hs / scale_factor, 3);
    sd_writer.debugSerialPrint(" m   Tc ");
    sd_writer.debugSerialPrint((float)r.Tc / scale_factor, 3);
    sd_writer.debugSerialPrint(" s   Tp ");
    sd_writer.debugSerialPrint((float)r.Tp / scale_factor, 3);
    sd_writer.debugSerialPrint(" s   Tz ");
    sd_writer.debugSerialPrint((float)r.Tz / scale_factor, 3);
    sd_writer.debugSerialPrintln(" s");
    sd_writer.debugSerialPrint("    max_value ");
    sd_writer.debugSerialPrint(max_value, 6);
    sd_writer.debugSerialPrint(" m^2/Hz   span ");
    sd_writer.debugSerialPrint((float)(r.timestamp_end - r.timestamp_start), 0);
    sd_writer.debugSerialPrint(" s   RSSI ");
    sd_writer.debugSerialPrint(rssi, 1);
    sd_writer.debugSerialPrintln(" dBm");

    // NB. Timestamp sent as 4B will only be valid until 2106. 
    // Either increase to 8B or use a different start data for the epoch (i.e. 2020)
    char ts[64];  // "    window " + two 10-digit values + " .. " is 39; leave margin
    sprintf(ts, "    window %lu .. %lu",
            (unsigned long)r.timestamp_start, (unsigned long)r.timestamp_end);
    sd_writer.debugSerialPrintln(ts);

    /*
      Wire format is a uint16 per bin normalised to the peak; absolute PSD is
      value/65535 * max_value. Both columns are printed so the raw payload can be checked
    */
    const float f_min = (float)r.spec_f_min / wave_freq_scale;
    const float f_max = (float)r.spec_f_max / wave_freq_scale;
    const float df    = r.num_bins > 1 ? (f_max - f_min) / (r.num_bins - 1) : 0.0f;

    sd_writer.debugSerialPrint("    PSD, ");
    sd_writer.debugSerialPrint((float)r.num_bins, 0);
    sd_writer.debugSerialPrint(" bins, f_min ");
    sd_writer.debugSerialPrint(f_min, 4);
    sd_writer.debugSerialPrint(" Hz, f_max ");
    sd_writer.debugSerialPrint(f_max, 4);
    sd_writer.debugSerialPrint(" Hz, df ");
    sd_writer.debugSerialPrint(df, 6);
    sd_writer.debugSerialPrintln(" Hz (f_hz raw psd_eta):");

    // Printing the spectrum.
    // Notice that spectrum frequencies are bin centres, not edges, 
    // and the step is df = (f_max - f_min)/(num_bins - 1)
    for (uint16_t i = 0; i < r.num_bins; i++){
      sd_writer.debugSerialPrint("      ");
      sd_writer.debugSerialPrint(f_min + i * df, 4);
      sd_writer.debugSerialPrint(" ");
      sd_writer.debugSerialPrint((float)r.wave_spectrum[i], 0);
      sd_writer.debugSerialPrint(" ");
      sd_writer.debugSerialPrintln(r.wave_spectrum[i] / 65535.0f * max_value, 6);
    }
}


void Message_Parser::print_beacon_reading(const beacon_Reading & r)
{
    sd_writer.debugSerialPrint("Received beacon message");
    sd_writer.debugSerialPrint("timestamp: ");
    sd_writer.debugSerialPrint(r.timestamp);
    sd_writer.debugSerialPrint(", lat: ");
    sd_writer.debugSerialPrint(r.lat);
    sd_writer.debugSerialPrint(", lng: ");
    sd_writer.debugSerialPrint(r.lng);
    sd_writer.debugSerialPrint(", bouy ID: ");
    sd_writer.debugSerialPrintln(r.buoy_id);
}


void Message_Parser::print_buoy_info_reading(const buoyInfoReading & r)
{
    sd_writer.debugSerialPrint("Sent packets: ");
    sd_writer.debugSerialPrint(r.sent_packets);
    sd_writer.debugSerialPrint(", left packets: ");
    sd_writer.debugSerialPrint(r.left_packets);
    sd_writer.debugSerialPrint(", listen time: ");
    sd_writer.debugSerialPrint(r.listen_time);
    sd_writer.debugSerialPrint(", crashed: ");
    sd_writer.debugSerialPrintln(r.crashed);
}
