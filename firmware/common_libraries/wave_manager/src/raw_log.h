#ifndef RAW_LOG_H
#define RAW_LOG_H

#include <Arduino.h>
#include "wave_config.h"

/*
  Writer for <stamp>_raw.bin: the 6-byte payload of every FIFO word verbatim

  LAYOUT, little-endian (kRaw* in wave_config.h):
    header  kRawHeaderBytes, once at the top
    word    1 B tag_sensor + 6 B payload                          = kRawWordBytes (7 B)
    sync    1 B kRawSyncTag + u32 t_us + u32 accel_n + u32 millis
                            + u16 n_words + u16 flags             = kRawSyncBytes (17 B)

  tag_sensor is 5 bits, so it can never collide with kRawSyncTag. There is no per-word
  timestamp: word order is the time axis, and one sync record per drain pins it to the
  clock.

  The file is a byte stream, not an array of blocks, a record may overlap sector boundaries
  Decoded by tools/raw_to_csv.py.
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

  // Header with most of the necessary information, so a capcture can be decoded without
  // the firmware that wrote it. kRawHeaderBytes is fixed; the tail is zeroed, and a reader
  // must skip to kRawHeaderBytes. 
  void writeHeader(uint32_t captureStartEpoch, uint16_t readingId);

  // One per drain, written before that drain's words. Pins the sample axis to the
  // clock: tUs is the sample time, accelN the cumulative accel (and also gyro) count
  void emitSync(uint32_t tUs, uint32_t accelN, uint16_t nWords, uint16_t flags);

  // The FIFO word exactly as it came off the bus. Called for every word, including
  // tags the wave chain does not decode: this is a record of what the sensor produce.
  void emitWord(uint8_t tag, const uint8_t payload[6]);

  /*
    Write buffer to sd-card.

    The normal path (force = false) writes whole sectors only, and only once
    kRawFlushThreshold has accumulated. The remainder stays at the front of
    the buffer until next time.

    force = true writes everything, tail included. Required at the end of a capture.
  */
  uint16_t flush(bool force = false);

  // Blocks the sink failed to write in full.
  uint32_t writeFailCount() const { return nWriteFail_; }

  // Drop buffered bytes and counters (for a new capture)
  void reset(void);

 private:
  // Append to write buffer(buf_) only.
  void append(const uint8_t *p, uint8_t n);

  RawBlockSink sink_ = nullptr;
  uint8_t  buf_[kRawBufBytes];
  uint16_t len_ = 0;
  uint32_t nWriteFail_ = 0;

  // Is set to true the moment a block is lost, cleared only once a sync record has
  // actually carried it into the file
  bool     writeFailPending_ = false;
};

#endif  // RAW_LOG_H
