#ifndef ROTATION_H
#define ROTATION_H

/*
  Convention: q = [w,x,y,z], body -> world, right-handed, roll about x and
  pitch about y, yaw = 0 (unobservable without a magnetometer  or other orientation sensors). 
*/

// Tilt from one accel sample: roll about x, pitch about y (rad). 
// Valid only when the accel vector is dominated by gravity, but used as seed for AHRS filter.
void rollPitchFromAccel(float ax, float ay, float az, float &roll, float &pitch);

// Quaternion [w,x,y,z] from roll/pitch, yaw = 0.
void quatFromRollPitch(float q[4], float roll, float pitch);

// Full body->world rotation, w = R(q) * a
void rotateBodyToWorld(const float q[4], float ax, float ay, float az, float w[3]);

// Vertical linear accel: rotate the body accel onto world Z and subtract
// gravity
float verticalAccel(const float q[4], float ax, float ay, float az, float gravity);

// p = p * q (Hamilton product), renormalised
void quatMultiplyNorm(float p[4], const float q[4]);

// Rotation vector {wx*dt, wy*dt, wz*dt} in radians -> quaternion
// Exact for any angle
void quatFromRotationVector(const float v[3], float q[4]);

// Skew-symmetric (cross-product) matrix of v, i.e. the matrix M with M*u = v x u.
// The matrix form of an angular rate
void skewSymmetric(const float v[3], float m[3][3]);

#endif  // ROTATION_H
