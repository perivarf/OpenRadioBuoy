#ifndef QUAT_DELAY_H
#define QUAT_DELAY_H

#include <stdint.h>

// PIF TODO

/*
  A pure delay line for the two quaternions that get logged per row: the software
  AHRS one and the chip's SFLP one.

  Why a delay and not a filter. The attitude is NOT decimated through the FIR the
  way ax..gz and vacc are, for three reasons: there is no aliasing to remove (the
  attitude is already the output of a heavy low-pass - Madgwick's accel correction
  has a multi-second time constant and the gyro feeding it is band-limited by the
  sea, so between the AHRS rate and the row rate there is essentially nothing to
  fold); a linear FIR over quaternion components preserves neither |q| = 1 nor the
  +-q double cover, where a single sign flip inside the tap window destroys the
  output; and eight more filter instances would cost ~4 kB of RAM and ~11 % CPU on a
  budget that has neither to spare.

  What the attitude does need is the FIR's DELAY. The decimating FIR is causal
  (compensate = 0, matching fir.py's firmware mode), so the ax..gz and vacc columns
  in a row describe the signal kFirHalf raw samples EARLIER than the row's
  timestamp. A plain latest-quaternion hold would describe now, leaving one CSV row
  describing two different instants - about 4 degrees of error at 1 Hz and 10 deg of
  tilt, and a trap for anyone cross-checking device attitude against an offline AHRS
  fed the same columns. Carrying the quaternions back by the same amount makes every
  column in a row refer to one moment.

  The AHRS steps once per raw sample, so the delay is kFirHalf pushes exactly - the
  FIR's group delay in its own units, with no conversion and no residual phase error.
  That is the reason the AHRS has no rate divider: a divided AHRS would make this a
  second decimation with its own divisibility constraint to get wrong.

  The SFLP quaternion runs at its own on-chip rate (kSflpOdrHz, 120 Hz) and is
  pushed in here zero-order-held at the AHRS cadence. That is deliberate: one ring
  and one index means both quaternions are delayed by exactly the same amount, and a
  dropped SFLP FIFO word cannot quietly change the delay the way a ring clocked by
  SFLP arrivals would.

  Arduino-free and templated on the slot count so it can be exercised on a host -
  see the delay check in the plan's verification section.
*/
template <uint16_t Slots>
class QuatDelay {
 public:
  static_assert(Slots >= 2, "a delay line needs at least one step of history");

  // Fill the whole ring with one attitude. Used at capture start, once the AHRS has
  // been seeded from gravity: reading out of a ring that has never been written
  // would otherwise hand the logger an all-zero quaternion for the first 16 steps.
  void reset(const float *ahrsQ, const float *sflpQ) {
    for (uint16_t s = 0; s < Slots; s++) store(s, ahrsQ, sflpQ);
    idx_ = 0;
  }

  // One AHRS step: 8 stores, no arithmetic.
  void push(const float *ahrsQ, const float *sflpQ) {
    store(idx_, ahrsQ, sflpQ);
    idx_ = (uint16_t)(idx_ + 1 == Slots ? 0 : idx_ + 1);
  }

  // The attitude from Slots-1 pushes ago. idx_ is the next write slot, which is also
  // the oldest entry still held - so the read is just an indexed load, no arithmetic
  // on the hot path. Both outputs are [w,x,y,z], the shared rotation.h convention.
  void read(float *ahrsQOut, float *sflpQOut) const {
    const float *s = q_[idx_];
    ahrsQOut[0] = s[0]; ahrsQOut[1] = s[1]; ahrsQOut[2] = s[2]; ahrsQOut[3] = s[3];
    sflpQOut[0] = s[4]; sflpQOut[1] = s[5]; sflpQOut[2] = s[6]; sflpQOut[3] = s[7];
  }

 private:
  void store(uint16_t s, const float *ahrsQ, const float *sflpQ) {
    q_[s][0] = ahrsQ[0]; q_[s][1] = ahrsQ[1]; q_[s][2] = ahrsQ[2]; q_[s][3] = ahrsQ[3];
    q_[s][4] = sflpQ[0]; q_[s][5] = sflpQ[1]; q_[s][6] = sflpQ[2]; q_[s][7] = sflpQ[3];
  }

  float    q_[Slots][8] = {};   // [0..3] AHRS, [4..7] SFLP
  uint16_t idx_ = 0;            // next write slot == oldest entry held
};

#endif  // QUAT_DELAY_H
