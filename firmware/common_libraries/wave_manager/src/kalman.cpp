#include "kalman.h"

#include <math.h>
#include <string.h>

#include "constants.h"  // kGravity
#include "matrix.h"     // inv3
#include "rotation.h"   // quatMultiplyNorm / quatFromRotationVector

// Equation numbers refer to Kok, Hol & Schön (2017), see kalman.h.
//
// P is stored as the full 6x6, but updated block by block. The STM32WL has no
// FPU, so every float operation is a library call, and the structure of F, G
// and H removes most of the work of the general 6x6 products:
//     P = [ A  B ]    A: eta-eta, B: eta-bias, C: bias-bias
//         [ B' C ]

static constexpr int kN = 6;

// Nominal sample interval, so a correct() before any predict() has a dt.
static constexpr float kDtSeed = 0.002f;

// R(q), body -> world, 3x3. Row i is the world axis i resolved in the body frame.
static void quatToMatrix(const float q[4], float R[3][3]) {
  const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  R[0][0] = 1.0f - 2.0f * (qy * qy + qz * qz);
  R[0][1] = 2.0f * (qx * qy - qw * qz);
  R[0][2] = 2.0f * (qx * qz + qw * qy);
  R[1][0] = 2.0f * (qx * qy + qw * qz);
  R[1][1] = 1.0f - 2.0f * (qx * qx + qz * qz);
  R[1][2] = 2.0f * (qy * qz - qw * qx);
  R[2][0] = 2.0f * (qx * qz - qw * qy);
  R[2][1] = 2.0f * (qy * qz + qw * qx);
  R[2][2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

void KalmanAhrs::reset(void) {
  q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
  b_[0] = b_[1] = b_[2] = 0.0f;
  memset(P_, 0, sizeof(P_));
  for (int i = 0; i < 3; i++) P_[i][i] = p_.p0Angle * p_.p0Angle;
  for (int i = 3; i < kN; i++) P_[i][i] = p_.p0Bias * p_.p0Bias;
  dt_ = kDtSeed;
}

// Algorithm 4, step 1: orientation from the accel, yaw = 0.
void KalmanAhrs::initFromAccel(float ax, float ay, float az) {
  float roll, pitch;
  rollPitchFromAccel(ax, ay, az, roll, pitch);
  quatFromRollPitch(q_, roll, pitch);
}

void KalmanAhrs::update(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
  if (dt <= 0.0f) return;
  predict(gx, gy, gz, dt);
  correct(ax, ay, az);
}

// Time update, (C.8), (C.9), (4.54b).
void KalmanAhrs::predict(float gx, float gy, float gz, float dt) {
  dt_ = dt;

  // (C.8): q~ <- q~ (.) exp_q(T/2 (y_w - delta_w)). Kok's exp_q(v) rotates by
  // 2|v|, quatFromRotationVector(v) by |v|, so the argument is T (y_w - delta_w).
  const float rot[3] = {(gx - b_[0]) * dt, (gy - b_[1]) * dt, (gz - b_[2]) * dt};
  float dq[4];
  quatFromRotationVector(rot, dq);
  quatMultiplyNorm(q_, dq);

  // (C.9): F = [I  M; 0  I] with M = -T R~, using R~ after the time update.
  float R[3][3];
  quatToMatrix(q_, R);
  float M[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) M[i][j] = -dt * R[i][j];

  // F P F^T, block by block:
  //     B <- B + M C
  //     A <- A + M B_old^T + B_new M^T
  //     C unchanged
  float Bn[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float s = P_[i][3 + j];
      for (int k = 0; k < 3; k++) s += M[i][k] * P_[3 + k][3 + j];
      Bn[i][j] = s;
    }
  }
  float An[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = i; j < 3; j++) {           // A is symmetric: upper triangle
      float s = P_[i][j];
      for (int k = 0; k < 3; k++) s += M[i][k] * P_[j][3 + k] + Bn[i][k] * M[j][k];
      An[i][j] = s;
    }
  }

  // + G Q G^T (C.5b) with Sigma_w = sigmaG^2/T and Sigma_dw = sigmaB^2 T. The
  // eta block is T R~ Sigma_w R~^T T = sigmaG^2 T I, since R~ is orthonormal.
  const float qg = p_.sigmaG * p_.sigmaG * dt;
  const float qb = p_.sigmaB * p_.sigmaB * dt;
  for (int i = 0; i < 3; i++) {
    for (int j = i; j < 3; j++) {
      const float a = An[i][j] + (i == j ? qg : 0.0f);
      P_[i][j] = P_[j][i] = a;
    }
    for (int j = 0; j < 3; j++) P_[i][3 + j] = P_[3 + j][i] = Bn[i][j];
    P_[3 + i][3 + i] += qb;
  }
}

// Measurement update (4.51), (4.38), (4.55a-b), relinearisation (4.56).
void KalmanAhrs::correct(float ax, float ay, float az) {
  // With g^n = [0, 0, -g] (z up):
  //   y^ = -R~^T g^n = g * (row 2 of R~)
  //   H_eta = -R~^T [g^n x] = g * [row 1 of R~, -row 0 of R~, 0]   (as columns)
  // The zero third column is yaw, which gravity cannot observe.
  float R[3][3];
  quatToMatrix(q_, R);
  const float g = kGravity;
  float H[3][2];
  for (int i = 0; i < 3; i++) {
    H[i][0] = g * R[1][i];
    H[i][1] = -g * R[0][i];
  }
  const float eps[3] = {ax - g * R[2][0], ay - g * R[2][1], az - g * R[2][2]};  // (4.38)

  // P H^T (6x3); only the first two columns of P meet a nonzero column of H.
  float PHt[kN][3];
  for (int i = 0; i < kN; i++)
    for (int j = 0; j < 3; j++) PHt[i][j] = P_[i][0] * H[j][0] + P_[i][1] * H[j][1];

  // S = H P H^T + Sigma_a (4.38), Sigma_a = sigmaA^2 / T.
  const float dt = dt_ > 0.0f ? dt_ : kDtSeed;
  const float r = p_.sigmaA * p_.sigmaA / dt;
  float S[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = i; j < 3; j++) {
      const float s = H[i][0] * PHt[0][j] + H[i][1] * PHt[1][j] + (i == j ? r : 0.0f);
      S[i][j] = S[j][i] = s;
    }
  }
  float Sinv[3][3];
  if (!inv3(S, Sinv)) return;

  // K = P H^T S^-1 (4.38)
  float K[kN][3];
  for (int i = 0; i < kN; i++)
    for (int j = 0; j < 3; j++)
      K[i][j] = PHt[i][0] * Sinv[0][j] + PHt[i][1] * Sinv[1][j] + PHt[i][2] * Sinv[2][j];

  // (4.55a): x^ = K eps (eta^ was zero before the update)
  float dx[kN];
  for (int i = 0; i < kN; i++)
    dx[i] = K[i][0] * eps[0] + K[i][1] * eps[1] + K[i][2] * eps[2];

  // (4.55b): P <- P - K S K^T = P - K (P H^T)^T, since K S = P H^T.
  for (int i = 0; i < kN; i++) {
    for (int j = i; j < kN; j++) {
      const float s = K[i][0] * PHt[j][0] + K[i][1] * PHt[j][1] + K[i][2] * PHt[j][2];
      P_[i][j] -= s;
      P_[j][i] = P_[i][j];
    }
  }

  // The bias is part of the state and is not reset (C.4).
  b_[0] += dx[3]; b_[1] += dx[4]; b_[2] += dx[5];

  // (4.56): q~ <- exp_q(eta^/2) (.) q~, eta^ <- 0. A LEFT multiplication,
  // because eta is expressed in the navigation frame.
  float dq[4];
  quatFromRotationVector(dx, dq);
  float qn[4] = {dq[0], dq[1], dq[2], dq[3]};
  quatMultiplyNorm(qn, q_);
  q_[0] = qn[0]; q_[1] = qn[1]; q_[2] = qn[2]; q_[3] = qn[3];
}
