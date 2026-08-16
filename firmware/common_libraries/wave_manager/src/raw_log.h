#ifndef RAW_LOG_H
#define RAW_LOG_H

#include <Arduino.h>
#include "wave_config.h"

/*
  Writer for <stamp>_raw.bin: the 6-byte payload of every FIFO word verbatim, where
  imu.csv is the FIR-decimated record. Nothing is decoded on the way out - the
  sensitivities ride in the header, and this runs inside the drain.

  LAYOUT, little-endian (kRaw* in wave_config.h):
    header  kRawHeaderBytes, once at the top
    word    1 B tag_sensor + 6 B payload                          = kRawWordBytes
    sync    1 B kRawSyncTag + u32 t_us + u32 accel_n + u32 millis
                            + u16 n_words + u16 flags             = kRawSyncBytes

  tag_sensor is 5 bits, so it can never collide with kRawSyncTag. There is no per-word
  timestamp: word order is the time axis, and one sync record per drain pins it to the
  clock.

  The file is a BYTE STREAM, not an array of blocks - a record may straddle a block
  boundary. Decoded by tools/raw_to_csv.py.
*/

// One filled block. Return false on a short write: it desynchronises every byte after
// it, so the next sync record carries kRawFlagWriteFail and the decoder knows where to
// stop trusting.
using RawBlockSink = bool (*)(const uint8_t *data, uint16_t len);

class RawLogWriter {
 public:
  // nullptr disables the log: every emit becomes a null check and nothing else.
  void setSink(RawBlockSink sink) { sink_ = sink; }
  bool active() const { return sink_ != nullptr; }

  // Self-describing header, so a capture can be decoded without the firmware that
  // wrote it. kRawHeaderBytes is fixed; the tail is reserved and zeroed, and a reader
  // must skip to kRawHeaderBytes rather than assume the fields end where this version
  // stopped writing. Goes through the sink, so setSink must come first.
  void writeHeader(uint32_t captureStartEpoch, uint16_t readingId);

  // One per drain, written BEFORE that drain's words. Pins the sample axis to the
  // clock: tUs is the fractional sample time, accelN the cumulative accel count so a
  // gap is arithmetic rather than guesswork, and millis() is when the drain actually
  // ran - the measurement that says whether SD stalls are threatening the FIFO.
  void emitSync(uint32_t tUs, uint32_t accelN, uint16_t nWords, uint16_t flags);

  // The FIFO word exactly as it came off the bus. Called for EVERY word, including
  // tags the wave chain does not decode: this is a record of what the sensor produced.
  void emitWord(uint8_t tag, const uint8_t payload[6]);

  /*
    Write what has accumulated, and return the bytes that actually reached the sink -
    0 when the card was not touched.

    The normal path (force = false) writes WHOLE SECTORS ONLY, and only once
    kRawFlushThreshold has accumulated; the remainder (< 512 B) stays at the front of
    the buffer until next time. See the flush-threshold section in wave_config.h.

    force = true writes everything, tail included. Required at the end of a capture -
    otherwise the last partial sector is lost - and by the safety valve in append().
  */
  uint16_t flush(bool force = false);

  // Blocks the sink failed to write in full. Non-zero means the file is desynchronised
  // past the first failure, however clean the sensor was.
  uint32_t writeFailCount() const { return nWriteFail_; }

  // Drop buffered bytes and counters for a new capture.
  void reset(void);

 private:
  // APPEND ONLY - never writes. The buffer holds the sub-threshold remainder plus one
  // whole drain (kRawBufBytes), because flush() is called by ImuSampler::update()
  // AFTER the pop loop has emptied the FIFO. That is the point: a card that stalls for
  // 800 ms should meet an empty FIFO with 256 free levels, not a half-drained one.
  void append(const uint8_t *p, uint8_t n);

  RawBlockSink sink_ = nullptr;
  uint8_t  buf_[kRawBufBytes];
  uint16_t len_ = 0;
  uint32_t nWriteFail_ = 0;

  // Sticky: set the moment a block is lost, cleared only once a sync record has
  // actually carried it into the file. Without the stickiness the report could itself
  // be the write that fails, and the loss would go unrecorded in the stream.
  bool     writeFailPending_ = false;
};

#endif  // RAW_LOG_H
