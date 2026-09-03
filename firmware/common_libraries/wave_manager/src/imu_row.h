#ifndef IMU_ROW_H
#define IMU_ROW_H

#include <stdint.h>

/*
  One row per row period (1/kRowOdrHz). Every field describes the SAME instant - the window centre,
  where the decimating FIR's group delay puts it (kFirS1DelayS). The
  quaternions are not filtered but are delayed by the same amount to match.
  winStartMs labels the window; it is not the instant the values describe. It is the
  window's true start rounded down to whole ms
*/

struct ImuRow {
  uint32_t winStartMs;                  // window start, relative ms from capture start
  uint16_t n;                           // accel samples in the window (quality metric only)
  float ax, ay, az;                     // FIR-decimated accel (mg, body frame)
  float axnSflp, aynSflp, aznSflp;      // FIR-decimated linear accel rotated by the SFLP
                                        // quaternion (mg, world frame, gravity removed)
  float gx, gy, gz;                     // FIR-decimated gyro (mdps)
  float qwSflp, qxSflp, qySflp, qzSflp; // on-chip SFLP quaternion, delay-matched to the above
  uint8_t braking;                      // 1 if linear |a| > threshold long enough
  uint8_t fifoOvf;                      // 1 if the FIFO overflowed while this window was open
  float qw, qx, qy, qz;                 // the selected WaveAhrs (Madgwick/Kalman) quaternion,
                                        // delay-matched to the above
  float vacc, vaccSflp;                 // unfiltered vertical linear accel (m/s^2) at the same instant (center())
  float vaccFir, vaccSflpFir;           // the same two series, FIR-decimated (eval())
  uint8_t sflpNan;                      // 1 if a NaN SFLP quaternion was rejected in the window
};

// Called when a window closes. The row leaves ImuSampler complete, hence const.
using ImuRowSink = void (*)(const ImuRow &);

#endif  // IMU_ROW_H
