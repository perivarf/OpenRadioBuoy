#include "wave_manager.h"

#include <math.h>
#include <TimeLib.h>
#include "parser_utils.h"

/*
  Serialising a result for the radio, and the DEBUG_WAVE_MSG bench fixture that
  exercises the same path without a capture.

  Split out of wave_manager.cpp: the wire layouts are read against readings.h and
  message_parser.cpp on the base station, not against the capture loop.
*/

// -----------------------------------------------------------------------------
// Serialise the front result. A measurement goes out as TWO messages: the
// parameters in msgB ('W' ... 'E') and the spectrum in psdB ('P' ... 'E'), paired
// by ts_start. See readings.h for the layouts and for why the pair is keyed on the
// timestamp rather than on reading_ID.
// -----------------------------------------------------------------------------

// Physical value -> the uint32 fixed point the wave parameters travel in.
static uint32_t waveToFixed(float v) {
  if (!(v > 0.0f)) return 0;                       // undefined (-1) or negative -> 0
  double scaled = (double)v * (double)scale_factor;
  if (scaled > 4294967295.0) return 0xFFFFFFFFUL;  // clamp to uint32 range
  return (uint32_t)llround(scaled);
}

size_t WaveManager::updateTransmitMessage(void) {
  if (wave_analysis_results.empty()) return 0;
  WaveResult res = wave_analysis_results.front();

  uint8_t offset = 0;
  msgB[offset++] = 'W';
  msg_insert_uint(msgB, res.reading_ID, offset, wave_message_size, offset, true);

  // Full time_t, 8 bytes, as every other message here serialises it. See readings.h.
  msg_insert_uint(msgB, res.timestamp_start, offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, res.timestamp_end,   offset, wave_message_size, offset, true);

  msg_insert_uint(msgB, waveToFixed(res.Hs), offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, waveToFixed(res.Tc), offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, waveToFixed(res.Tp), offset, wave_message_size, offset, true);
  msg_insert_uint(msgB, waveToFixed(res.Tz), offset, wave_message_size, offset, true);

  // msg_insert_int so the sign survives: sign-and-magnitude, five bytes each, the same
  // encoding the 'G' message uses. Already 1e-7 deg (gps_coord_scale) straight from the
  // receiver, so nothing is rescaled. 0,0 means "unknown" - see readings.h.
  msg_insert_int(msgB, res.lat_start_e7, offset, wave_message_size, offset, true);
  msg_insert_int(msgB, res.lng_start_e7, offset, wave_message_size, offset, true);
  msg_insert_int(msgB, res.lat_end_e7,   offset, wave_message_size, offset, true);
  msg_insert_int(msgB, res.lng_end_e7,   offset, wave_message_size, offset, true);

  msgB[offset++] = 'E';

  /*
    The result is deliberately NOT popped here. The caller pops with
    popTransmittedResult() once the radio has confirmed TxDone, and a failure
    leaves the result at the head of the queue for the next window.
  */
  return offset;
}

size_t WaveManager::updatePsdTransmitMessage(void) {
  if (wave_analysis_results.empty() || !kSendPsd) return 0;
  WaveResult res = wave_analysis_results.front();

  uint8_t offset = 0;
  psdB[offset++] = 'P';
  msg_insert_uint(psdB, res.reading_ID, offset, wave_spectrum_message_size, offset, true);

  // The join key. Must be byte-identical to the value the 'W' message carried, so
  // it is the same field at the same width and nothing recomputes it.
  msg_insert_uint(psdB, res.timestamp_start, offset, wave_spectrum_message_size, offset, true);

  // max_value gets wave_psd_scale, not scale_factor. An acceleration PSD is orders of
  // magnitude smaller than a wave height or a period, and at 1e5 a calm-sea peak rounds
  // to 0 - which zeroes the whole spectrum on the far side, since every bin is
  // reconstructed as (value/65535)^2 * max_value. See readings.h for the range.
  auto toPsdFixed = [](float v) -> uint32_t {
    if (!(v > 0.0f)) return 0;                       // undefined (-1) or negative -> 0
    double scaled = (double)v * (double)wave_psd_scale;
    if (scaled > 4294967295.0) return 0xFFFFFFFFUL;  // clamp to uint32 range
    return (uint32_t)llround(scaled);
  };
  msg_insert_uint(psdB, toPsdFixed(res.max_value), offset, wave_spectrum_message_size, offset, true);

  // Frequency axis, so the base station can label the bins it is about to read: bin
  // CENTRES, then the count immediately before the bins themselves, which is what tells
  // the receiver how many to consume. wave_freq_scale, not scale_factor - see readings.h.
  auto toFreqFixed = [](float f) -> uint32_t {
    return (uint32_t)llround((double)f * (double)wave_freq_scale);
  };
  msg_insert_uint(psdB, toFreqFixed(kSpecFMinHz), offset, wave_spectrum_message_size, offset, true);
  msg_insert_uint(psdB, toFreqFixed(kSpecFMaxHz), offset, wave_spectrum_message_size, offset, true);
  msg_insert_uint(psdB, (uint16_t)kSpecTxBins,    offset, wave_spectrum_message_size, offset, true);

  // Last field: it is the only variable-length one, so stopping short here shortens
  // the message without moving anything the receiver has already read. kSpecTxBins is
  // at most welch_bins - the capacity wave_spectrum_message_size was budgeted for - and
  // is smaller whenever kPsdMaxFreq does not divide evenly into the bin grid.
  for (size_t i = 0; i < kSpecTxBins; i++) {
    msg_insert_uint(psdB, res.wave_spectrum[i], offset, wave_spectrum_message_size, offset, true);
  }
  psdB[offset++] = 'E';
  return offset;
}

void WaveManager::popTransmittedResult(void) {
  if (!wave_analysis_results.empty()) wave_analysis_results.pop_front();
}

#if DEBUG_WAVE_MSG
// Synthetic result for bench testing - see DEBUG_WAVE_MSG in wave_config.h.
void WaveManager::enqueueFakeResult(void) {
  WaveResult res{};
  res.reading_ID = ++readingID_;

  // Deliberately NOT round numbers, and all distinct: if the fixed-point scaling or
  // the field ORDER is wrong on the receiving side, distinct odd values say so
  // immediately, where 1.0/2.0/3.0 could line up plausibly after a swap.
  res.Hs        = 1.37f;   // m
  res.Tc        = 2.53f;   // s
  res.Tp        = 6.91f;   // s
  res.Tz        = 4.29f;   // s
  res.max_value = 0.0842f; // peak acceleration PSD ((m/s^2)^2/Hz)

  // A single smooth peak, encoded exactly as finalize() does: sqrt(binAcc/peakAcc) *
  // 65535, so the far side reconstructs (value/65535)^2 * max_value. The sqrt has to be
  // here too - a fixture that encoded linearly would still decode to a plausible
  // gaussian and stop testing the one thing it exists to test. The peak sits off-centre
  // so a mirrored or off-by-one bin axis is visible.
  const float peakBin = 0.35f * (float)kSpecNBins;
  const float width   = 0.12f * (float)kSpecNBins;
  for (size_t j = 0; j < kSpecNBins; j++) {
    const float d = ((float)j - peakBin) / width;
    const float norm = expf(-0.5f * d * d);
    res.wave_spectrum[j] = (uint16_t)lroundf(sqrtf(norm) * 65535.0f);
  }

  res.timestamp_end   = now();
  res.timestamp_start = res.timestamp_end -
                        (time_t)(wave_measurement_duration / s_2_ms);

  // A short synthetic drift with both signs present: a receiver that drops the sign
  // character, or reads the pair in the wrong order, cannot produce these four numbers
  // by accident. One is west of Greenwich on purpose - all-positive coordinates would
  // never exercise the sign branch.
  res.lat_start_e7 =  599578000;   //  59.9578 N
  res.lng_start_e7 =  110686000;   //  11.0686 E
  res.lat_end_e7   =  599601000;   //  59.9601 N, drifted north
  res.lng_end_e7   =  -110701000;  // -11.0701, i.e. W: exercises the sign byte

  // Same bound handling as processReading: the deque is fixed-size, and dropping the
  // OLDEST keeps the freshest results when transmit cannot keep up.
  if (wave_analysis_results.full()) wave_analysis_results.pop_back();
  wave_analysis_results.push_front(res);
}

// What is about to go on the air, in physical units - so a scaling or field-order
// mistake in updateTransmitMessage is visible against these numbers rather than only
// after decoding on the base station.
void WaveManager::printPendingResult(Print &out) const {
  if (wave_analysis_results.empty()) return;
  const WaveResult &r = wave_analysis_results.front();

  out.print(F("  wave result #"));  out.println(r.reading_ID);
  out.print(F("    Hs "));  out.print(r.Hs, 3);  out.print(F(" m   Tc "));
  out.print(r.Tc, 3);       out.print(F(" s   Tp "));
  out.print(r.Tp, 3);       out.print(F(" s   Tz "));
  out.print(r.Tz, 3);       out.println(F(" s"));
  out.print(F("    max_value "));  out.print(r.max_value, 9);
  out.print(F(" (m/s^2)^2/Hz   span "));
  out.print((uint32_t)(r.timestamp_end - r.timestamp_start));  out.println(F(" s"));

  // sprintf, not out.print(float): an epoch near 1.77e9 does not survive a float
  // round trip (24-bit mantissa -> rounded to the nearest 128 s), and these are the
  // fields that wrap if the RTC was never set, so they must be exact to be useful.
  char ts[64];  // "    window " + two 10-digit values + " .. " is 39; leave margin
  sprintf(ts, "    window %lu .. %lu",
          (unsigned long)r.timestamp_start, (unsigned long)r.timestamp_end);
  out.println(ts);

  // Position at each end of the window, printed as the receiver will read it:
  // 1e-7 deg back to degrees. 0,0 is what a window without a fix sends, and it is
  // labelled rather than printed as a coordinate off West Africa.
  auto printPos = [&out](const __FlashStringHelper *label, int32_t lat, int32_t lng) {
    out.print(label);
    if (lat == 0 && lng == 0) { out.println(F(" no fix (0,0)")); return; }
    out.print(' ');  out.print((double)lat / gps_coord_scale, 6);
    out.print(F(", ")); out.println((double)lng / gps_coord_scale, 6);
  };
  printPos(F("    pos start"), r.lat_start_e7, r.lng_start_e7);
  printPos(F("    pos end  "), r.lat_end_e7,   r.lng_end_e7);

  // Raw uint16 and decoded value side by side, so the payload can be checked without
  // doing (value/65535)^2 * max_value by hand. Must stay identical to
  // print_wave_analysis_reading() in message_parser.cpp - disagreement between the two
  // is what would reveal a half-finished format change.
  //
  // The frequency is built from kSpecFMinHz and kSpecBinWidthHz, the two values the
  // message actually carries, rather than from welch_bin_min and the group size - so
  // this is the receiver's arithmetic and not a parallel derivation that could agree
  // here and disagree over the air.
  if (!kSendPsd) {
    out.println(F("    PSD not transmitted (kSendPsd off) - num_bins 0"));
    return;
  }

  out.print(F("    PSD, "));      out.print((uint32_t)kSpecTxBins);
  out.print(F(" bins, f_min "));  out.print(kSpecFMinHz, 4);
  out.print(F(" Hz, f_max "));    out.print(kSpecFMaxHz, 4);
  out.print(F(" Hz, df "));       out.print(kSpecBinWidthHz, 6);
  out.println(F(" Hz (f_hz raw psd_acc):"));
  for (size_t j = 0; j < kSpecTxBins; j++) {
    out.print(F("      "));      out.print(kSpecFMinHz + j * kSpecBinWidthHz, 4);
    out.print(' ');              out.print(r.wave_spectrum[j]);
    out.print(' ');
    const float n = r.wave_spectrum[j] / 65535.0f;
    out.println(n * n * r.max_value, 9);
  }
}
#endif  // DEBUG_WAVE_MSG
