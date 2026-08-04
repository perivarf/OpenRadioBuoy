#include "fir.h"

void FirDecimator::reset(void) {
  for (uint16_t k = 0; k < kFirNtap; k++) z_[k] = 0.0f;
  idx_ = 0;
}

void FirDecimator::push(float x) {
  z_[idx_] = x;
  idx_ = (uint16_t)(idx_ + 1 == kFirNtap ? 0 : idx_ + 1);
}

float FirDecimator::eval(void) const {
  // y[n] = sum_k c[k] * x[n-k]. A linear-phase FIR has c[k] == c[N-1-k], so the two
  // samples that share a coefficient can be added before the multiply: 65 multiplies
  // instead of 129. That is ~25 % of the cost of this function, and this function is
  // the dominant new load in the FIFO drain loop - so folding is the implementation,
  // not a fallback if it turns out too slow.
  uint16_t i = idx_ ? (uint16_t)(idx_ - 1) : (uint16_t)(kFirNtap - 1);  // newest, x[n]
  uint16_t j = idx_;                                                    // oldest, x[n-(N-1)]
  float acc = 0.0f;
  for (uint16_t k = 0; k < kFirHalf; k++) {
    acc += c_[k] * (z_[i] + z_[j]);
    i = i ? (uint16_t)(i - 1) : (uint16_t)(kFirNtap - 1);
    j = (uint16_t)(j + 1 == kFirNtap ? 0 : j + 1);
  }
  // After kFirHalf steps back from the newest sample, i is the centre tap - the one
  // coefficient without a partner.
  return acc + c_[kFirHalf] * z_[i];
}
