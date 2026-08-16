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
  // Safety valve, not the normal path: the buffer holds the sub-threshold remainder
  // plus a worst-case drain, so this can only fire if the FIFO delivered more than
  // kFifoDepthWords - i.e. if that constant is wrong again. A write mid-loop is then
  // better than writing past the end of the buffer. force, or a call that only writes
  // whole sectors could leave too little room to help.
  if (len_ + n > kRawBufBytes) flush(true);
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
  // Fold in any block lost since the last sync. This is the only place the loss can be
  // reported IN the stream, and it must be reported there: ana.csv can say a capture
  // lost blocks, but not WHERE - and where is the whole question, because everything
  // after the first loss is misaligned.
  if (writeFailPending_) flags |= kRawFlagWriteFail;
  const uint16_t vals16[2] = {nWords, flags};
  for (uint8_t v = 0; v < 2; v++) {
    rec[o++] = (uint8_t)(vals16[v]);
    rec[o++] = (uint8_t)(vals16[v] >> 8);
  }

  const uint32_t failBefore = nWriteFail_;
  append(rec, kRawSyncBytes);
  // Clear only if appending the report did not itself lose a block. When it did, the
  // flag stays pending and the NEXT sync carries it - one record late in the file, but
  // never silently dropped. The decoder's job is to stop trusting the tail, and it
  // still does.
  if (nWriteFail_ == failBefore) writeFailPending_ = false;
}

uint16_t RawLogWriter::flush(bool force) {
  if (!sink_ || len_ == 0) return 0;

  // The threshold belongs HERE and not at the call site: kRawBufBytes is sized on the
  // assumption that no more than kRawFlushThreshold - 1 is ever left when a drain
  // starts, and that only holds if every call respects it.
  if (!force && len_ < kRawFlushThreshold) return 0;

  // force takes the tail; otherwise it waits for the next drain to fill a whole sector
  // around it. The remainder's space is budgeted in kRawBufBytes.
  const uint16_t n = force ? len_
                           : (uint16_t)(len_ / kRawBlockBytes) * kRawBlockBytes;
  if (n == 0) return 0;

  if (!sink_(buf_, n)) {
    nWriteFail_++;
    writeFailPending_ = true;
  }

  // Move the remainder forward: up to 511 B a couple of times a second, negligible
  // against the write it just waited on.
  len_ -= n;
  if (len_ > 0) memmove(buf_, buf_ + n, len_);
  return n;
}
