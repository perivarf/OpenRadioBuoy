#ifndef MADGWICK_H
#define MADGWICK_H

/*
  Madgwick 6-axis AHRS (IMU variant: gyro + accel, no magnetometer), split out of
  wave_analysis.cpp so the filter is a unit of its own - one object owns the
  quaternion and the gain, and can be reused or tested without pulling in the
  Welch/wave-parameter chain.

  Deliberately free of Arduino, LSM6DSV16X and wave_config.h dependencies (only
  rotation.h and <math.h>), so it compiles on a host for offline comparison
  against tools/postprocess.py. The gain is supplied by the caller - on
  the drifter that is kMadgwickBeta from wave_config.h.

  Quaternion convention is the shared one from rotation.h: q = [w,x,y,z],
  body -> world, yaw unobservable without a magnetometer and left at whatever
  the initialisation set it to.
*/

class Madgwick {
 public:
  explicit Madgwick(float beta) : beta_(beta) {}

  // Identity orientation. Start of a capture: call this, then seed the attitude
  // with initFromAccel() on the first sample.
  void reset(void);

  // Seed q from one accel sample (gravity -> roll/pitch, yaw = 0) so the filter
  // starts near the true attitude instead of converging from identity.
  void initFromAccel(float ax, float ay, float az);

  // One filter step. gyro in rad/s, accel in any unit (normalised internally),
  // dt in seconds. An all-zero accel vector (free fall / dropout) skips the
  // gravity correction, leaving pure gyro integration for that step.
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  const float *quaternion(void) const { return q_; }   // [w,x,y,z], unit length
  float beta(void) const { return beta_; }
  void  setBeta(float beta) { beta_ = beta; }

  // Name for the capture logs. Lives on the class so the filter selected in
  // wave_config.h and the string written to ses.csv cannot disagree.
  static constexpr const char *kName = "Madgwick";

 private:
  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float beta_;
};

#endif  // MADGWICK_H
