#ifndef KALMAN_H
#define KALMAN_H

#include "math.h"

/*
  EKF with orientation deviation states and gyro bias, accel + gyro only.

  Follows Kok, Hol & Schön, "Using Inertial Sensors for Position and Orientation
  Estimation", Foundations and Trends in Signal Processing 11(1-2), 2017:
  Algorithm 4 with the gyro bias extension in Appendix C.4. Equation numbers in
  kalman.cpp refer to that text. The magnetometer rows are dropped.

  State: x = [eta^n, delta_w]
      eta^n   (3) -> orientation deviation, NAVIGATION frame (rad)
      delta_w (3) -> gyro bias (rad/s), kept in the state (not reset)

  The attitude estimate q~ is the linearisation point. After each measurement
  update eta^n is moved into q~ (4.56) and reset to zero.

  The accel is NOT normalised: the measurement model (4.51) is the accel vector
  in m/s², and wave acceleration is covered by the measurement noise Sigma_a.

  NOISE AS DENSITIES. Kok gives Sigma_w, Sigma_dw and Sigma_a per sample. They
  are set from noise densities here, so the filter bandwidth does not depend on
  the sample rate:
      Sigma_w  = sigmaG^2 / dt,  Sigma_dw = sigmaB^2 * dt,  Sigma_a = sigmaA^2 / dt

  Consequences of measuring gravity alone (Kok, Example 5.3):
    - yaw is unobservable; the third column of H is zero;
    - the gyro bias component along gravity is not identifiable while the buoy
      is level. Roll/pitch bias, the part that matters for vertical acceleration,
      converges.

  Quaternion convention is the one from rotation.h: q = [w,x,y,z], body -> world.
*/

struct KalmanAhrsParams {
  float sigmaG;    // gyro noise density [rad/s/sqrt(Hz)]
  float sigmaB;    // gyro bias random walk [rad/s^2/sqrt(Hz)]
  float sigmaA;    // accel noise density [m/s^2/sqrt(Hz)]
  float p0Angle;   // initial orientation uncertainty [rad]
  float p0Bias;    // prior on the gyro bias [rad/s]
};

// sigmaA = sqrt(2e-5) * g: the accel noise that gave the lowest low-frequency
// noise floor on the Skjærhalden sessions (variant B).
static constexpr KalmanAhrsParams kKalmanParams = {
    /* sigmaG  */ 0.005f,        // rad/s/sqrt(Hz), ~0.3 deg/s/sqrt(Hz)
    /* sigmaB  */ 1.0e-5f,       // rad/s^2/sqrt(Hz)
    /* sigmaA  */ 0.0438567f,    // m/s^2/sqrt(Hz)
    /* p0Angle */ 5.0f * (float)M_PI / 180.0f,    // 5 deg
    /* p0Bias  */ 1.0f * (float)M_PI / 180.0f,    // 1 deg/s
};

class KalmanAhrs {
 public:
  explicit KalmanAhrs(const KalmanAhrsParams &p) : p_(p) { reset(); }

  // Identity attitude, zero bias, P = diag(p0Angle^2, p0Bias^2).
  void reset(void);

  // Seed the attitude from one accel sample (gravity -> roll/pitch, yaw = 0).
  // Leaves the bias estimate and the covariance alone.
  void initFromAccel(float ax, float ay, float az);

  // One filter step. Gyro in rad/s, accel in m/s^2, dt in seconds.
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  const float *quaternion(void) const { return q_; }   // [w,x,y,z], unit length
  const float *gyroBias(void) const { return b_; }     // rad/s, body frame

  // Name for the logs
  static constexpr const char *kName = "Kalman";

 private:
  void predict(float gx, float gy, float gz, float dt);
  void correct(float ax, float ay, float az);

  KalmanAhrsParams p_;
  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float b_[3] = {0.0f, 0.0f, 0.0f};
  float P_[6][6] = {};
  float dt_ = 0.0f;   // dt of the last predict(), for Sigma_a
};

#endif  // KALMAN_H
