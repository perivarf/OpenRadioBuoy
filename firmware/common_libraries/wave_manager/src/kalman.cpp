#include "kalman.h"

#include <math.h>
#include <string.h>

#include "matrix.h"     // matMul / matMulTransposed / inv3
#include "rotation.h"   // quatMultiplyNorm / quatFromRotationVector / skewSymmetric

// Error-state dimension: 3 attitude + 3 gyro bias. kN is the covariance size, not
// the state size - the quaternion itself stays out of P, which is the whole point
// of the multiplicative form.
static constexpr int kN = 6;

void KalmanAhrs::reset(void) {
  q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
  b_[0] = b_[1] = b_[2] = 0.0f;
  memset(P_, 0, sizeof(P_));
  for (int i = 0; i < 3; i++) P_[i][i] = p_.p0Angle * p_.p0Angle;
  for (int i = 3; i < kN; i++) P_[i][i] = p_.p0Bias * p_.p0Bias;
  wNorm_ = 0.0f;
  // Seeded to dtRef, not 0: a correct() before any predict() would otherwise
  // divide by zero in measurementNoise().
  dt_ = p_.dtRef;
}

void KalmanAhrs::initFromAccel(float ax, float ay, float az) {
  float roll, pitch;
  rollPitchFromAccel(ax, ay, az, roll, pitch);
  quatFromRollPitch(q_, roll, pitch);
}

void KalmanAhrs::update(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
  // A zero or negative interval is not a step: predict() would have nothing to
  // propagate, and correcting anyway would fold the same instant into P twice.
  if (dt <= 0.0f) return;
  // Order is binding: the correction must act on the predicted state, and the
  // adaptive R reads the |w| and dt that predict() just recorded.
  predict(gx, gy, gz, dt);
  correct(ax, ay, az);
}

float KalmanAhrs::measurementNoise(float accelNorm) const {
  const float dev = (accelNorm - p_.gravity) / p_.gravity;
  const float wn = wNorm_ / p_.w0;
  // dtRef/dt: see RATE INVARIANCE in kalman.h. Without it the filter bandwidth
  // would follow the logging rate and two captures could not be compared.
  const float rate = dt_ > 0.0f ? p_.dtRef / dt_ : 1.0f;
  return p_.r0 * rate * (1.0f + p_.lambdaA * dev * dev + p_.lambdaW * wn * wn);
}

// Propagate the nominal quaternion with the bias-corrected rate, and the error
// covariance with the linearised error dynamics
//     dTheta' = -[w x] dTheta - dBias,   dBias' = 0.
// Takes the RAW gyro: the bias is subtracted here, but |w| is measured before
// that, because the adaptive R asks how fast the sensor is actually turning.
void KalmanAhrs::predict(float gx, float gy, float gz, float dt) {
  if (dt <= 0.0f) return;

  wNorm_ = sqrtf(gx * gx + gy * gy + gz * gz);
  dt_ = dt;

  const float wx = gx - b_[0], wy = gy - b_[1], wz = gz - b_[2];

  // Nominal attitude: q <- q * exp(w*dt), the exact exponential map rather than a
  // first-order step. At 100 Hz the two differ by ~1e-7 rad, but the exact form
  // costs one sinf/cosf and removes the question entirely.
  const float rot[3] = {wx * dt, wy * dt, wz * dt};
  float dq[4];
  quatFromRotationVector(rot, dq);
  quatMultiplyNorm(q_, dq);

  // F = I + dt * [ -[w x]  -I ]
  //              [    0     0 ]
  float F[kN][kN] = {};
  for (int i = 0; i < kN; i++) F[i][i] = 1.0f;
  F[0][1] =  wz * dt;  F[0][2] = -wy * dt;
  F[1][0] = -wz * dt;  F[1][2] =  wx * dt;
  F[2][0] =  wy * dt;  F[2][1] = -wx * dt;
  F[0][3] = -dt;       F[1][4] = -dt;      F[2][5] = -dt;

  // P = F P F^T + Q, with Q = diag(sigmaG^2, sigmaB^2)*dt - the noise DENSITIES
  // squared, which is what makes them independent of the sample rate.
  float tmp[kN][kN];
  matMul(F, P_, tmp);
  matMulTransposed(tmp, F, P_);
  const float qg = p_.sigmaG * p_.sigmaG * dt;
  const float qb = p_.sigmaB * p_.sigmaB * dt;
  for (int i = 0; i < 3; i++) P_[i][i] += qg;
  for (int i = 3; i < kN; i++) P_[i][i] += qb;
}

// Correct with the accelerometer read as a gravity direction.
void KalmanAhrs::correct(float ax, float ay, float az) {
  const float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm <= 0.0f) return;      // no direction information in this sample
  const float z[3] = {ax / norm, ay / norm, az / norm};

  // Predicted gravity direction in the body frame: R(q)^T applied to world +Z,
  // which is the third ROW of the body->world rotation matrix - the same row
  // verticalAccel() projects onto.
  const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
  const float h[3] = {2.0f * (q1 * q3 - q0 * q2),
                      2.0f * (q2 * q3 + q0 * q1),
                      1.0f - 2.0f * (q1 * q1 + q2 * q2)};

  const float y[3] = {z[0] - h[0], z[1] - h[1], z[2] - h[2]};

  // H = [ [h x] | 0 ]: a small body-frame attitude error dTheta rotates the
  // predicted direction by h x dTheta. Note the rank: H sees only the two
  // directions gravity can move in, so yaw is never touched.
  float H[3][3];
  skewSymmetric(h, H);

  // The adaptive part. r is measured HERE, from this sample's |a| and the
  // rotation rate predict() just saw - not a constant.
  const float r = measurementNoise(norm);

  // PHt = P * H^T  (6x3), using that the bias block of H is zero.
  float PHt[kN][3];
  for (int i = 0; i < kN; i++) {
    for (int j = 0; j < 3; j++) {
      float s = 0.0f;
      for (int k = 0; k < 3; k++) s += P_[i][k] * H[j][k];
      PHt[i][j] = s;
    }
  }

  // S = H * PHt + R
  float S[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float s = 0.0f;
      for (int k = 0; k < 3; k++) s += H[i][k] * PHt[k][j];
      S[i][j] = s + (i == j ? r : 0.0f);
    }
  }

  float Sinv[3][3];
  if (!inv3(S, Sinv)) return;

  // K = PHt * S^-1  (6x3)
  float K[kN][3];
  for (int i = 0; i < kN; i++) {
    for (int j = 0; j < 3; j++) {
      float s = 0.0f;
      for (int k = 0; k < 3; k++) s += PHt[i][k] * Sinv[k][j];
      K[i][j] = s;
    }
  }

  // Error state dx = K y, injected into the nominal state and then dropped - the
  // "multiplicative" part: the attitude error becomes a small rotation applied to
  // the quaternion, so the error state is zero again by construction.
  float dx[kN];
  for (int i = 0; i < kN; i++) {
    dx[i] = K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
  }

  const float dtheta[3] = {dx[0], dx[1], dx[2]};
  float dq[4];
  quatFromRotationVector(dtheta, dq);
  quatMultiplyNorm(q_, dq);
  b_[0] += dx[3]; b_[1] += dx[4]; b_[2] += dx[5];

  // Joseph form: P = (I-KH) P (I-KH)^T + K R K^T, rather than the short
  // P = (I-KH) P. The short form is only valid for the OPTIMAL gain, and with an
  // adaptive R the gain is far from optimal every time R has just inflated -
  // there the short form can drive P indefinite and the filter with it. Joseph
  // stays symmetric positive definite for any K, which is exactly the insurance
  // an adaptive filter needs.
  float IKH[kN][kN] = {};
  for (int i = 0; i < kN; i++) IKH[i][i] = 1.0f;
  for (int i = 0; i < kN; i++) {
    for (int j = 0; j < 3; j++) {         // K*H is zero outside its first 3 cols
      float s = 0.0f;
      for (int k = 0; k < 3; k++) s += K[i][k] * H[k][j];
      IKH[i][j] -= s;
    }
  }
  float tmp[kN][kN];
  matMul(IKH, P_, tmp);
  matMulTransposed(tmp, IKH, P_);
  // + K R K^T. R = r*I, so this is r * K K^T.
  for (int i = 0; i < kN; i++) {
    for (int j = 0; j < kN; j++) {
      float s = 0.0f;
      for (int k = 0; k < 3; k++) s += K[i][k] * K[j][k];
      P_[i][j] += r * s;
    }
  }
  // Force symmetry back onto P. Joseph is symmetric in exact arithmetic, so this
  // only removes float32 rounding - but it is 15 additions, and an asymmetric
  // covariance eventually produces a gain that amplifies noise instead of
  // rejecting it.
  for (int i = 0; i < kN; i++) {
    for (int j = i + 1; j < kN; j++) {
      const float s = 0.5f * (P_[i][j] + P_[j][i]);
      P_[i][j] = P_[j][i] = s;
    }
  }
}
