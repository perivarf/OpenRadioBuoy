#ifndef KALMAN_H
#define KALMAN_H

/*
  Quaternion Kalman AHRS for a 6-axis IMU: a multiplicative / error-state EKF.
  The attitude is carried as a unit quaternion and never enters the covariance;
  what the filter actually estimates is a 6-element ERROR state

      [ dTheta (3) | dBias (3) ]
        ^ small-angle attitude error, body frame (rad)
                     ^ gyro bias error (rad/s)

  which is injected into the quaternion after each correction and then reset to
  zero. That indirection is the point: a quaternion has four components but only
  three degrees of freedom, so filtering it directly gives a singular covariance
  and fights its own normalisation.

  Prediction integrates the bias-corrected gyro. Correction treats the
  accelerometer as a measurement of the GRAVITY DIRECTION - normalised, so only
  the direction is measured, exactly like the atan2 tilt it replaces.

  ADAPTIVE MEASUREMENT NOISE. This is the whole reason the filter is worth
  running, and what the first version of this file lacked:

      R = r0 * (dtRef/dt) * (1 + lambdaA*((|a|-g)/g)^2 + lambdaW*(|w|/w0)^2)

  The accelerometer is a gravity reference only while |a| ~ g. On this buoy |a|
  reaches ~3 g, and gravity is least trustworthy exactly when the buoy rotates
  fast, since the attitude then changes within one sample. R inflates during
  slams and fast rolling, leaving attitude to the gyro; in calm stretches it
  drops back and the accel anchors the slow drift.

  MEASURED, on Skjaerhalden 20260731_110314 through tools/postprocess.py:
  with a FIXED R this filter left a flat 8.5e-4 (m/s^2)^2/Hz tilt-leakage floor
  below 0.25 Hz and reported Hs 0.222 m where Madgwick and SFLP both said 0.097 m.
  The same filter WITH the adaptive R lands at 0.135 m. lambdaW carries essentially
  all of that (-34 % on the noise floor); lambdaA measured as a no-op, since slams
  are too brief to land in the 0.08-0.20 Hz band, and is kept only because it costs
  two multiplies and documents the intent.

  RATE INVARIANCE. The dtRef/dt factor keeps the filter bandwidth independent of
  the logging rate. Process noise is Q = sigmaG^2*dt, so a fixed R would give a
  steady-state gain K ~ sqrt(sigmaG^2*dt/R) and a time constant dt/K proportional
  to sqrt(dt) - two captures logged at different rates would then be compared
  through two different filters. Scaling R with 1/dt cancels it exactly. R is then
  a noise DENSITY rather than a per-sample variance, which is also the honest
  reading: each ImuRow is a window average, and a shorter window averages away
  less noise.

  Two consequences of measuring gravity alone, both inherent and not worth
  fighting on this hardware:
    - yaw is unobservable, so it holds whatever the seed left it at;
    - the gyro bias component ALONG gravity is unobservable too, so bias about the
      vertical axis is never corrected. Roll/pitch bias, the part that matters for
      vertical acceleration, converges normally.

  Ported from tools/kalman.py, which holds the parameter sweeps behind
  the defaults in wave_config.h. The two are meant to stay the same estimator:
  tools/mekf.py mirrors THIS file, so postprocess.py's MEKF column and
  its Kalman column diverging is the signal that they have drifted apart. The
  chi^2 innovation gate in kalman.py is deliberately not ported - it is off
  (gate = 0) in every configuration that has been run.

  Deliberately free of Arduino / wave_config.h dependencies (rotation.h, matrix.h
  and <math.h> only), so it compiles on a host next to postprocess.py. The tuning
  is supplied by the caller - on the drifter that is kKalmanParams.

  Quaternion convention is the shared one from rotation.h: q = [w,x,y,z],
  body -> world.
*/

// Tuning. Grouped in a struct rather than a constructor argument list because
// ten positional floats cannot be read at the call site - and because the set
// belongs together: it is one tuning, swept as a whole (see kalman.py).
struct KalmanAhrsParams {
  float sigmaG;    // gyro noise density [rad/s/sqrt(Hz)]
  float sigmaB;    // gyro bias random walk [rad/s^2/sqrt(Hz)]
  float r0;        // base variance of the accel DIRECTION, at dtRef
  float dtRef;     // sample interval the tuning was swept at [s]
  float lambdaA;   // weight on the |a| deviation from 1 g
  float lambdaW;   // weight on |w| - where the gain is
  float w0;        // normalisation for |w| [rad/s]
  float gravity;   // [m/s^2]; ACCEL MUST BE IN THE SAME UNIT - see update()
  float p0Angle;   // initial attitude uncertainty [rad]
  float p0Bias;    // initial bias uncertainty [rad/s]
};

class KalmanAhrs {
 public:
  explicit KalmanAhrs(const KalmanAhrsParams &p) : p_(p) {
    if (p_.w0 < 1.0e-9f) p_.w0 = 1.0e-9f;   // w0 divides in measurementNoise()
    reset();
  }

  // Identity attitude, zero bias, P = diag(p0Angle^2, p0Bias^2). A NON-zero P0
  // is what lets the first corrections actually move the state: with P = 0 the
  // filter would take about a second at 100 Hz to trust the accelerometer again,
  // and would spend that second integrating raw gyro.
  void reset(void);

  // Seed the attitude from one accel sample (gravity -> roll/pitch, yaw = 0).
  // Leaves the bias estimate and the covariance alone.
  void initFromAccel(float ax, float ay, float az);

  // One filter step. gyro in rad/s, dt in seconds, and accel in the SAME UNIT AS
  // params.gravity (m/s^2 on the drifter) - unlike a plain normalised-gravity
  // AHRS this filter is not scale-free, because the adaptive R measures how far
  // |a| is from 1 g. Feed it mg and every sample looks like a 100 g slam.
  // An all-zero accel vector carries no gravity direction, so that step predicts
  // only.
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  const float *quaternion(void) const { return q_; }   // [w,x,y,z], unit length
  const float *gyroBias(void) const { return b_; }     // rad/s, body frame

  // The adaptive measurement variance for an accel vector of this magnitude, at
  // the rotation rate and dt of the last predict(). Public because it is the one
  // number that explains why a capture came out the way it did.
  float measurementNoise(float accelNorm) const;

  // Name for the capture logs. Lives on the class so the filter selected in
  // wave_config.h and the string written to ses.csv cannot disagree.
  static constexpr const char *kName = "Kalman";

 private:
  void predict(float gx, float gy, float gz, float dt);
  void correct(float ax, float ay, float az);

  KalmanAhrsParams p_;
  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float b_[3] = {0.0f, 0.0f, 0.0f};
  float P_[6][6] = {};
  // |w| and dt from the last predict(). The adaptive R needs both - how fast we
  // are rotating right now, and how long the sample averaged over - and neither
  // is visible to correct() on its own.
  float wNorm_ = 0.0f;
  float dt_ = 0.0f;
};

#endif  // KALMAN_H
