#ifndef ROTATION_H
#define ROTATION_H

/*
  Quaternion / rotation math shared by the orientation methods, split out of
  wave_analysis.cpp and madgwick.cpp so neither owns primitives the other needs:

    Madgwick  -> rollPitchFromAccel + quatFromRollPitch to seed its quaternion
    Kalman    -> rollPitchFromAccel for the accel measurement, quatFromRollPitch
                 to turn the two filtered angles back into a quaternion
    both/SFLP -> verticalAccel to project body acceleration onto world Z

  Convention throughout: q = [w,x,y,z], body -> world, right-handed, roll about x
  and pitch about y, yaw = 0 (unobservable without a magnetometer). Every method
  must produce this same convention, otherwise verticalAccel() silently returns
  the wrong projection.

  Deliberately free of Arduino / wave_config.h dependencies (only <math.h>), so
  it compiles on a host next to ORB_test/tools/postprocess.py - hence gravity is
  a parameter rather than kGravity.
*/

// Tilt from one accel sample: roll about x, pitch about y (rad). Valid only when
// the accel vector is dominated by gravity - during a wave the buoy's own
// acceleration biases both angles, which is exactly what the AHRS filters out.
void rollPitchFromAccel(float ax, float ay, float az, float &roll, float &pitch);

// Quaternion [w,x,y,z] from roll/pitch, yaw = 0.
void quatFromRollPitch(float q[4], float roll, float pitch);

// Vertical linear accel: rotate the body accel onto world Z and subtract
// gravity. Units follow the inputs - pass accel and gravity in the same unit
// (m/s^2 on the drifter).
float verticalAccel(const float q[4], float ax, float ay, float az, float gravity);

// q = q * p (Hamilton product), renormalised. Folding a small rotation into an
// attitude: the Kalman filter injects its attitude correction this way.
void quatMultiplyNorm(float q[4], const float p[4]);

// Rotation vector (axis * angle, rad) -> quaternion; the exponential map of a
// rotation. Exact for any angle, not the [1, v/2] small-angle form - which is
// why the Kalman filter can use one and the same call both to integrate a full
// gyro step and to inject a tiny attitude correction.
void quatFromRotationVector(const float v[3], float q[4]);

// Skew-symmetric (cross-product) matrix of v, i.e. the matrix M with M*u = v x u.
// The matrix form of an angular rate - it is what makes a rotation infinitesimal,
// so it turns up in the Kalman error dynamics and in its measurement Jacobian.
void skewSymmetric(const float v[3], float m[3][3]);

#endif  // ROTATION_H
