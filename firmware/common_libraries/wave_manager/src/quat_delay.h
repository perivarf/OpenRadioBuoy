#ifndef QUAT_DELAY_H
#define QUAT_DELAY_H

#include <stdint.h>

/*
  Delay buffer for the AHRS and SFLP quaternions to match the FIR delay in the 
  ax, ay, az, gx, gy, gz and vacc columns of an ImuRow.

  The ax..gz and vacc columns in a row describe the signal kFirHalf raw samples earlier
  than the row's timestamp. Selecting quaternions from kFirHalf raw samples earlier
  makes every element in a row refer to the same moment.

  The SFLP quaternion runs at its own on-chip rate (kSflpOdrHz<=kImuOdrHz) and is
  pushed in here zero-order-held at kImuOdrHz (the AHRS cadence).

*/
template <uint16_t Slots>
class QuatDelay {
 public:
  static_assert(Slots >= 2, "a delay line needs at least one step of history");

  // Fills the whole ring with one attitude. Used at capture start, once the AHRS has
  // been seeded from gravity.
  // Without this, the logger would receive an all-zero quaternion for the first Slots (=kFirHalf+1) steps.
  void reset(const float *ahrsQ, const float *sflpQ) {
    for (uint16_t s = 0; s < Slots; s++) store(s, ahrsQ, sflpQ);
    idx_ = 0;
  }

  // One AHRS step
  void push(const float *ahrsQ, const float *sflpQ) {
    store(idx_, ahrsQ, sflpQ);
    idx_ = (uint16_t)(idx_ + 1 == Slots ? 0 : idx_ + 1);
  }

  // The attitude from Slots-1 pushes ago
  // The oldest entry in the ring is the next one to be overwritten, at idx_.
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
