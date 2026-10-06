#include "madgwick.h"
#include <math.h>
#include "rotation.h"

void Madgwick::reset(void) {
  q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
}

void Madgwick::initFromAccel(float ax, float ay, float az) {
  float roll, pitch;
  rollPitchFromAccel(ax, ay, az, roll, pitch);
  quatFromRollPitch(q_, roll, pitch);
}

// One filter step. The equation numbers are from section 3 of the report above.
void Madgwick::update(float gx, float gy, float gz,
                      float ax, float ay, float az, float dt) {
  // (12) the gyro's contribution: qDot = 0.5 * q (x) [0, w]
  const float omega[4] = {0.0f, gx, gy, gz};
  float qDot[4];
  quatMultiply(q_, omega, qDot);
  for (int i = 0; i < 4; ++i) qDot[i] *= 0.5f;

  const float n2 = ax * ax + ay * ay + az * az;
  
  if (n2 > 0.0f) {
    // Only the DIRECTION of the accel is used, so the unit does not matter.
    const float invLen = 1.0f / sqrtf(n2);
    ax *= invLen; ay *= invLen; az *= invLen;

    const float qw = q_[0], qx = q_[1], qy = q_[2], qz = q_[3];

    // (25) the error: measured gravity direction against the one the current
    // attitude predicts
    // assumes buoy acceleration is zero, i.e. only gravity affects accelerometer
    // Only valid when the buoy accelerometer is small compared to gravity
    const float f[3] = {
      2.0f * (qx * qz - qw * qy) - ax,
      2.0f * (qw * qx + qy * qz) - ay,
      2.0f * (0.5f - qx * qx - qy * qy) - az
    };

    // (26) the Jacobian df/dq, 3x4
    const float J[3][4] = {
      {-2.0f * qy,  2.0f * qz, -2.0f * qw, 2.0f * qx},
      { 2.0f * qx,  2.0f * qw,  2.0f * qz, 2.0f * qy},
      { 0.0f,      -4.0f * qx, -4.0f * qy, 0.0f     }
    };

    // (34) grad = J^T f (first branch with acceleration only)
    float grad[4];
    for (int c = 0; c < 4; ++c) {
      grad[c] = J[0][c] * f[0] + J[1][c] * f[1] + J[2][c] * f[2];
    }

    // (43) step beta along the error direction
    const float gn = sqrtf(grad[0] * grad[0] + grad[1] * grad[1] +
                           grad[2] * grad[2] + grad[3] * grad[3]);
    if (gn > 0.0f) {
      const float s = beta_ / gn;
      for (int i = 0; i < 4; ++i) qDot[i] -= s * grad[i];
    }
  }

  // (42) integrate
  float q[4];
  for (int i = 0; i < 4; ++i) q[i] = q_[i] + qDot[i] * dt;
  
  // Renormalise because q is a rotation
  const float invLen = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] +
                                    q[2] * q[2] + q[3] * q[3]);

  for (int i = 0; i < 4; ++i) q_[i] = q[i] * invLen;
}
