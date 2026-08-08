#include "rotation.h"

#include <math.h>

void rollPitchFromAccel(float ax, float ay, float az, float &roll, float &pitch) {
  roll = atan2f(ay, az);
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
}

void quatFromRollPitch(float q[4], float roll, float pitch) {
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  q[0] = cp * cr; q[1] = cp * sr; q[2] = sp * cr; q[3] = -sp * sr;
}

void quatMultiplyNorm(float q[4], const float p[4]) {
  const float r0 = q[0] * p[0] - q[1] * p[1] - q[2] * p[2] - q[3] * p[3];
  const float r1 = q[0] * p[1] + q[1] * p[0] + q[2] * p[3] - q[3] * p[2];
  const float r2 = q[0] * p[2] - q[1] * p[3] + q[2] * p[0] + q[3] * p[1];
  const float r3 = q[0] * p[3] + q[1] * p[2] - q[2] * p[1] + q[3] * p[0];
  const float n = 1.0f / sqrtf(r0 * r0 + r1 * r1 + r2 * r2 + r3 * r3);
  q[0] = r0 * n; q[1] = r1 * n; q[2] = r2 * n; q[3] = r3 * n;
}

void quatFromRotationVector(const float v[3], float q[4]) {
  const float th = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (th < 1.0e-12f) {
    // Small-angle branch, and not only for speed: sinf(th/2)/th is 0/0 at th = 0.
    q[0] = 1.0f; q[1] = 0.5f * v[0]; q[2] = 0.5f * v[1]; q[3] = 0.5f * v[2];
    return;
  }
  const float s = sinf(0.5f * th) / th;
  q[0] = cosf(0.5f * th); q[1] = v[0] * s; q[2] = v[1] * s; q[3] = v[2] * s;
}

void skewSymmetric(const float v[3], float m[3][3]) {
  m[0][0] =  0.0f;  m[0][1] = -v[2];  m[0][2] =  v[1];
  m[1][0] =  v[2];  m[1][1] =  0.0f;  m[1][2] = -v[0];
  m[2][0] = -v[1];  m[2][1] =  v[0];  m[2][2] =  0.0f;
}

void rotateBodyToWorld(const float q[4], float ax, float ay, float az, float w[3]) {
  const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  w[0] = (1.0f - 2.0f * (qy * qy + qz * qz)) * ax +
         2.0f * (qx * qy - qw * qz) * ay +
         2.0f * (qx * qz + qw * qy) * az;
  w[1] = 2.0f * (qx * qy + qw * qz) * ax +
         (1.0f - 2.0f * (qx * qx + qz * qz)) * ay +
         2.0f * (qy * qz - qw * qx) * az;
  w[2] = 2.0f * (qx * qz - qw * qy) * ax +
         2.0f * (qy * qz + qw * qx) * ay +
         (1.0f - 2.0f * (qx * qx + qy * qy)) * az;
}

float verticalAccel(const float q[4], float ax, float ay, float az, float gravity) {
  // Third row of the body->world rotation matrix, applied to (ax,ay,az).
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  float wZ = 2.0f * (qx * qz - qw * qy) * ax +
             2.0f * (qy * qz + qw * qx) * ay +
             (1.0f - 2.0f * (qx * qx + qy * qy)) * az;
  return wZ - gravity;
}
