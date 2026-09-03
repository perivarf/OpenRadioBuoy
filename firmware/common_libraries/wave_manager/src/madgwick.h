#ifndef MADGWICK_H
#define MADGWICK_H

/*
  Madgwick 6-axis AHRS (IMU variant: gyro + accel, no magnetometer)

  Quaternion q = [w,x,y,z],
  body -> world, yaw unobservable without a magnetometer and left at whatever the initialisation set it to.
*/

static constexpr float kMadgwickBeta = 0.05f;

class Madgwick {
 public:
  explicit Madgwick(float beta) : beta_(beta) {}

  // Identity orientation. Call this at start of a capture
  void reset(void);

  // Seed q from one accel sample (gravity -> roll/pitch, yaw = 0) so the filter
  // starts near the true attitude instead of converging from identity.
  void initFromAccel(float ax, float ay, float az);

  // One filter step. gyro in rad/s, accel in any unit (normalised internally),
  // dt in seconds
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  const float *quaternion(void) const { return q_; }   // [w,x,y,z], unit length
  float beta(void) const { return beta_; }
  void  setBeta(float beta) { beta_ = beta; }

  // Name for the logs
  static constexpr const char *kName = "Madgwick";

 private:
  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float beta_;
};

#endif  // MADGWICK_H
