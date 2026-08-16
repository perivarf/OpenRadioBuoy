#ifndef ROTATION_H
#define ROTATION_H

/*
  Quaternion / rotation math shared by the orientation methods, so neither owns
  primitives the other needs:

    Madgwick  -> rollPitchFromAccel + quatFromRollPitch to seed its quaternion
    Kalman    -> rollPitchFromAccel for the accel measurement, quatFromRollPitch
                 to turn the two filtered angles back into a quaternion
    both/SFLP -> verticalAccel to project body acceleration onto world Z

  Convention throughout: q = [w,x,y,z], body -> world, right-handed, roll about x and
  pitch about y, yaw = 0 (unobservable without a magnetometer). Every method must
  produce this same convention, or verticalAccel() silently returns the wrong
  projection.

  Deliberately free of Arduino and config dependencies (only <math.h>), so it compiles
  on a host next to tools/postprocess.py - hence gravity is a parameter, not kGravity.
*/

// Tilt from one accel sample: roll about x, pitch about y (rad). Valid only when
// the accel vector is dominated by gravity - during a wave the buoy's own
// acceleration biases both angles, which is exactly what the AHRS filters out.
void rollPitchFromAccel(float ax, float ay, float az, float &roll, float &pitch);

// Quaternion [w,x,y,z] from roll/pitch, yaw = 0.
void quatFromRollPitch(float q[4], float roll, float pitch);

// Full body->world rotation, w = R(q) * a. Gravity is NOT removed - the caller
// subtracts it from whichever component it uses, in whatever unit it passed in.
//
// verticalAccel below is the third row of this, kept as its own function because the
// raw-sample path usually needs only that row and computing all three would be waste.
// Use this one when the whole vector is needed (the brake flag's |a|, the world-frame
// accel columns), so the rotation exists once rather than as an inlined 3x3 copy.
void rotateBodyToWorld(const float q[4], float ax, float ay, float az, float w[3]);

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
