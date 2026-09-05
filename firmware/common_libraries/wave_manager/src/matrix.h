#ifndef MATRIX_H
#define MATRIX_H

#include <math.h>

// PIF TODO

/*
  Small fixed-size dense matrix helpers, split out of kalman.cpp. Nothing here is
  attitude- or IMU-specific - it is plain linear algebra, so it does not belong in
  the filter, and it does not belong in rotation.h either (that file holds the
  primitives that ARE attitude-specific).

  Header-only and templated on the dimension: every call site knows its size at
  compile time, so the loops unroll and nothing is dispatched. Arrays are taken BY
  REFERENCE rather than as pointers, so both extents stay part of the type and a
  mismatched size is a compile error instead of a silent out-of-bounds read.

  Only what the Kalman AHRS needs is here, and no more. The rectangular products
  inside KalmanAhrs::correct() stay hand-written on purpose: they exploit the zero
  bias block of the measurement Jacobian, which a generic multiply cannot, and
  would otherwise do twice the work.

  Free of Arduino and config dependencies, like the filters that use it, so the
  host build keeps working.
*/

// out = a * b. out must not alias a or b.
template <int N>
void matMul(const float (&a)[N][N], const float (&b)[N][N], float (&out)[N][N]) {
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      float s = 0.0f;
      for (int k = 0; k < N; k++) s += a[i][k] * b[k][j];
      out[i][j] = s;
    }
  }
}

// out = a * b^T. out must not alias a or b.
template <int N>
void matMulTransposed(const float (&a)[N][N], const float (&b)[N][N], float (&out)[N][N]) {
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      float s = 0.0f;
      for (int k = 0; k < N; k++) s += a[i][k] * b[j][k];
      out[i][j] = s;
    }
  }
}

// Inverse of a 3x3 via its adjugate - the closed form is both faster and steadier
// in float32 than elimination at this size. Returns false on a (near-)singular
// matrix, which for a Kalman innovation covariance would mean a zero measurement
// noise and a degenerate update.
inline bool inv3(const float (&m)[3][3], float (&out)[3][3]) {
  const float a = m[0][0], b = m[0][1], c = m[0][2];
  const float d = m[1][0], e = m[1][1], f = m[1][2];
  const float g = m[2][0], h = m[2][1], i = m[2][2];

  const float A =  (e * i - f * h);
  const float B = -(d * i - f * g);
  const float C =  (d * h - e * g);
  const float det = a * A + b * B + c * C;
  if (fabsf(det) < 1e-20f) return false;

  const float invDet = 1.0f / det;
  out[0][0] = A * invDet; out[0][1] = (c * h - b * i) * invDet; out[0][2] = (b * f - c * e) * invDet;
  out[1][0] = B * invDet; out[1][1] = (a * i - c * g) * invDet; out[1][2] = (c * d - a * f) * invDet;
  out[2][0] = C * invDet; out[2][1] = (b * g - a * h) * invDet; out[2][2] = (a * e - b * d) * invDet;
  return true;
}

#endif  // MATRIX_H
