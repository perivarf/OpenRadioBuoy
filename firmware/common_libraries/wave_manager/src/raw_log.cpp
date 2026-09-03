#include "raw_log.h"

#include <string.h>   // memmove: flush() moves the tail forward after a partial write

void RawLogWriter::reset(void) {
  len_ = 0;
  nWriteFail_ = 0;
  writeFailPending_ = false;
}

void RawLogWriter::writeHeader(uint32_t captureStartEpoch, uint16_t readingId) {
  if (!sink_) return;
  uint8_t h[kRawHeaderBytes] = {0};
  uint8_t o = 0;
  auto put32 = [&](uint32_t v) {
    h[o++] = (uint8_t)v;         h[o++] = (uint8_t)(v >> 8);
    h[o++] = (uint8_t)(v >> 16); h[o++] = (uint8_t)(v >> 24);
  };
  auto put16 = [&](uint16_t v) { h[o++] = (uint8_t)v; h[o++] = (uint8_t)(v >> 8); };
  auto putf  = [&](float f) { uint32_t b; memcpy(&b, &f, 4); put32(b); };

  // Header
  put32(kRawMagic);
  h[o++] = kRawFormatVersion;
  h[o++] = kRawWordBytes;
  put16(kImuOdrHz);
  put16((uint16_t)kSflpOdrHz);
  putf(kAccSensMgPerLsb);       // int16 LSB -> mg
  putf(kGyrSensMdpsPerLsb);     // int16 LSB -> mdps
  put32(captureStartEpoch);     // capture t=0 in UTC epoch seconds
  put16(readingId);
  put16(kRawSyncBytes);

  // Straight to the sink, not through buf_: the header is written before any word and
  // is not part of the sector-alignment budget the buffer is sized for.
  if (!sink_(h, kRawHeaderBytes)) {
    nWriteFail_++;
    writeFailPending_ = true;
  }
}

void RawLogWriter::append(const uint8_t *p, uint8_t n) {
  if (!sink_) return;
  
  // Safety valve, not the normal path: this will only fire if the FIFO delivered more than
  // kFifoDepthWords (i.e. if the FIFO is not properly sized)
  if (len_ + n > kRawBufBytes) flush(true);

  // Sdding data to the buffer
  for (uint8_t i = 0; i < n; i++) buf_[len_++] = p[i];
}

void RawLogWriter::emitWord(uint8_t tag, const uint8_t payload[6]) {
  if (!sink_) return;
  uint8_t rec[kRawWordBytes];
  rec[0] = tag;
  for (uint8_t i = 0; i < 6; i++) rec[i + 1] = payload[i];
  append(rec, kRawWordBytes);
}

void RawLogWriter::emitSync(uint32_t tUs, uint32_t accelN, uint16_t nWords,
                            uint16_t flags) {
  if (!sink_) return;
  uint8_t rec[kRawSyncBytes];
  uint8_t o = 0;
  rec[o++] = kRawSyncTag;
  const uint32_t vals[3] = {tUs, accelN, millis()};
  for (uint8_t v = 0; v < 3; v++) {
    rec[o++] = (uint8_t)(vals[v]);
    rec[o++] = (uint8_t)(vals[v] >> 8);
    rec[o++] = (uint8_t)(vals[v] >> 16);
    rec[o++] = (uint8_t)(vals[v] >> 24);
  }
  
  // If writeFail, report the loss in the raw stream
  if (writeFailPending_) flags |= kRawFlagWriteFail;
  const uint16_t vals16[2] = {nWords, flags};
  for (uint8_t v = 0; v < 2; v++) {
    rec[o++] = (uint8_t)(vals16[v]);
    rec[o++] = (uint8_t)(vals16[v] >> 8);
  }

  const uint32_t failBefore = nWriteFail_;
  append(rec, kRawSyncBytes);

  // Clear only if appending the report did not itself lose a block
  if (nWriteFail_ == failBefore) writeFailPending_ = false;
}

uint16_t RawLogWriter::flush(bool force) {

  // No need to flush if no sink or no data
  if (!sink_ || len_ == 0) return 0;

  // If below threshhold, delay write unless forced.
  if (!force && len_ < kRawFlushThreshold) return 0;

  // Force: Write all, otherwise write whole sector only
  const uint16_t n = force ? len_
                           : (uint16_t)(len_ / kRawBlockBytes) * kRawBlockBytes;
  if (n == 0) return 0;

  // Write
  if (!sink_(buf_, n)) {
    nWriteFail_++;
    writeFailPending_ = true; // That we have a write fail not written to file..
  }

  // Move the remainder forward. Easier than doing a ring-buffer with split of the data.
  len_ -= n;
  if (len_ > 0) memmove(buf_, buf_ + n, len_);
  return n;
}
