#include "message_parser.h"
#include "parser_utils.h"
#include "sd_writer.h"

Message_Parser MESSAGE_PARSER;

void print_wave_analysis_message(const wave_analysis_Reading &w)
{
    sd_writer.debugSerialPrint("wave info - reading id: ");
    sd_writer.debugSerialPrint(w.reading_ID);
    sd_writer.debugSerialPrint(", Hs: ");
    sd_writer.debugSerialPrint((float)w.Hs / scale_factor);
    sd_writer.debugSerialPrint(" m, Tz: ");
    sd_writer.debugSerialPrint((float)w.Tz / scale_factor);
    sd_writer.debugSerialPrint(" s, Tc: ");
    sd_writer.debugSerialPrint((float)w.Tc / scale_factor);
    sd_writer.debugSerialPrint(" s, Tp: ");
    sd_writer.debugSerialPrint((float)w.Tp / scale_factor);
    sd_writer.debugSerialPrint(" s, max_value: ");
    sd_writer.debugSerialPrintln((float)w.max_value / scale_factor);

    /*
      Elevation PSD spectrum, welch_bins consecutive bins. Which frequencies they
      cover is set drifter-side and logged with the capture; it is deliberately
      not a shared constant, so do not label these in Hz here. Values are
      normalised to the peak (0-65535); absolute PSD = value/65535 * max_value.
    */
    sd_writer.debugSerialPrintln("wave spectrum (normalised 0-65535):");
    for (size_t i = 0; i < welch_bins; i++){
      sd_writer.debugSerialPrint((float)w.wave_spectrum[i]);
      sd_writer.debugSerialPrint(" ");
    }
    sd_writer.debugSerialPrintln("");

    /*
      Formatted through sprintf rather than debugSerialPrint(float): that
      overload is the only numeric one, and an epoch near 1.77e9 does not survive
      a float round trip (24-bit mantissa -> rounded to the nearest 128 s). These
      are also the fields that wrap if the sender's RTC was never set, so they
      need to be exact to be worth printing at all.
    */
    char ts[80];  // worst case is exactly 64 with three wrapped uint32s; leave margin
    sprintf(ts, "timestamps: start %lu, end %lu, span %lu s",
            (unsigned long)w.timestamp_start,
            (unsigned long)w.timestamp_end,
            (unsigned long)(w.timestamp_end - w.timestamp_start));
    sd_writer.debugSerialPrintln(ts);
}
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

    wa_reading_packet.reading_ID = msg_extract_uint<uint16_t>(msg, offset, true, offset);
    wa_reading_packet.Hs         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Tc         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Tp         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Tz         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.max_value  = msg_extract_uint<uint32_t>(msg, offset, true, offset);

    for (size_t i = 0; i < welch_bins; i++)
        wa_reading_packet.wave_spectrum[i] = msg_extract_uint<uint16_t>(msg, offset, true, offset);

    wa_reading_packet.timestamp_start = msg_extract_uint<time_t>(msg, offset, true, offset);
    wa_reading_packet.timestamp_end   = msg_extract_uint<time_t>(msg, offset, true, offset);

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
