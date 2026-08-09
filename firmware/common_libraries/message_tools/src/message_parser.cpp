#include "message_parser.h"
#include "parser_utils.h"
#include "sd_writer.h"

Message_Parser MESSAGE_PARSER;

void print_wave_analysis_message(const wave_analysis_Reading &w)
{
    const float max_value = (float)w.max_value / scale_factor;

    /*
      Layout is byte-for-byte what WaveManager::printPendingResult writes on the
      drifter before it transmits, so the two consoles can be diffed line for line
      during a bench test: if they agree the codec is right and any remaining fault
      is on the air. Change one and change the other.
    */
    sd_writer.debugSerialPrint("  wave result #");
    sd_writer.debugSerialPrintln((float)w.reading_ID, 0);
    sd_writer.debugSerialPrint("    Hs ");
    sd_writer.debugSerialPrint((float)w.Hs / scale_factor, 3);
    sd_writer.debugSerialPrint(" m   Tc ");
    sd_writer.debugSerialPrint((float)w.Tc / scale_factor, 3);
    sd_writer.debugSerialPrint(" s   Tp ");
    sd_writer.debugSerialPrint((float)w.Tp / scale_factor, 3);
    sd_writer.debugSerialPrint(" s   Tz ");
    sd_writer.debugSerialPrint((float)w.Tz / scale_factor, 3);
    sd_writer.debugSerialPrintln(" s");
    sd_writer.debugSerialPrint("    max_value ");
    sd_writer.debugSerialPrint(max_value, 6);
    sd_writer.debugSerialPrint(" m^2/Hz   span ");
    sd_writer.debugSerialPrint((float)(w.timestamp_end - w.timestamp_start), 0);
    sd_writer.debugSerialPrintln(" s");

    /*
      Formatted through sprintf rather than debugSerialPrint(float): that overload is
      the only numeric one, and an epoch near 1.77e9 does not survive a float round
      trip (24-bit mantissa -> rounded to the nearest 128 s). These are also the
      fields that wrap if the sender's RTC was never set, so they need to be exact to
      be worth printing at all.
    */
    char ts[64];  // "    window " + two 10-digit values + " .. " is 39; leave margin
    sprintf(ts, "    window %lu .. %lu",
            (unsigned long)w.timestamp_start, (unsigned long)w.timestamp_end);
    sd_writer.debugSerialPrintln(ts);

    /*
      Elevation PSD spectrum, ONE LINE PER BIN so the shape is readable and a single
      bin can be quoted. Columns are the bin centre frequency, the raw wire value and
      the absolute PSD it decodes to - printing only the normalised value leaves the
      reader to do value/65535 * max_value by hand for every bin, which is the whole
      quantity of interest.

      The frequency axis comes from the message (spec_f_min/spec_f_max/num_bins), not
      from a local constant: this target has no sample rate or segment length, so
      anything derived here would be a guess about the sender.

      Digits matter. The one-argument float overload prints two decimals, and the bins
      land around 1e-3 m^2/Hz - every one of them would read 0.00.
    */
    const float f_min = (float)w.spec_f_min / wave_freq_scale;
    const float f_max = (float)w.spec_f_max / wave_freq_scale;
    const float df    = w.num_bins > 1 ? (f_max - f_min) / (w.num_bins - 1) : 0.0f;

    sd_writer.debugSerialPrint("    PSD, ");
    sd_writer.debugSerialPrint((float)w.num_bins, 0);
    sd_writer.debugSerialPrint(" bins, f_min ");
    sd_writer.debugSerialPrint(f_min, 4);
    sd_writer.debugSerialPrint(" Hz, f_max ");
    sd_writer.debugSerialPrint(f_max, 4);
    sd_writer.debugSerialPrint(" Hz, df ");
    sd_writer.debugSerialPrint(df, 6);
    sd_writer.debugSerialPrintln(" Hz (f_hz raw psd_eta):");
    for (uint16_t i = 0; i < w.num_bins; i++){
      sd_writer.debugSerialPrint("      ");
      sd_writer.debugSerialPrint(f_min + i * df, 4);
      sd_writer.debugSerialPrint(" ");
      sd_writer.debugSerialPrint((float)w.wave_spectrum[i], 0);
      sd_writer.debugSerialPrint(" ");
      sd_writer.debugSerialPrintln(w.wave_spectrum[i] / 65535.0f * max_value, 6);
    }
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

    // Everything up to num_bins sits at a fixed offset; only the spectrum varies, and
    // it comes last, so a message carrying fewer bins still parses down to here.
    // time_t, matching the width the sender writes - see wave_message_size in
    // readings.h. Extracting at any other width shifts every field behind it.
    wa_reading_packet.reading_ID      = msg_extract_uint<uint16_t>(msg, offset, true, offset);
    wa_reading_packet.timestamp_start = msg_extract_uint<time_t>(msg, offset, true, offset);
    wa_reading_packet.timestamp_end   = msg_extract_uint<time_t>(msg, offset, true, offset);
    wa_reading_packet.Hs         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Tc         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Tp         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.Tz         = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.max_value  = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.spec_f_min = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.spec_f_max = msg_extract_uint<uint32_t>(msg, offset, true, offset);
    wa_reading_packet.num_bins   = msg_extract_uint<uint16_t>(msg, offset, true, offset);

    // The sender states its own bin count, so one built with a different welch_bins
    // no longer misaligns anything - it can only overrun our array, and those bins
    // are dropped. num_bins is corrected to what was actually stored so the printer
    // and any consumer loop cannot walk past the end.
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
