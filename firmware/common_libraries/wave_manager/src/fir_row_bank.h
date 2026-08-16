#ifndef FIR_ROW_BANK_H
#define FIR_ROW_BANK_H

#include "fir.h"      // FirDecimator - one per channel
#include "imu_row.h"  // ImuRow - what eval() fills

/*
  The ten series decimated per row - everything imu.csv logs, all through the same
  filter, so an offline AHRS sees the antialiasing the on-device one saw.

  ax,ay,az: accel in mg, body frame
  nx,ny,nz: linear accel in mg, gravity frame, gravity removed, rotated by the SFLP quaternion
  gx,gy,gz: gyro in mdps, body frame
  vacc: vertical linear accel in m/s^2, gravity frame, gravity removed, rotated
*/
class FirRowBank {
 public:
  explicit FirRowBank(const float *coeffs)
      : ax_(coeffs), ay_(coeffs), az_(coeffs),
        nx_(coeffs), ny_(coeffs), nz_(coeffs),
        gx_(coeffs), gy_(coeffs), gz_(coeffs), vacc_(coeffs) {}

  void reset(void);

  // One raw sample into all ten delay lines. Accel/NED in mg, gyro in mdps, vacc in
  // m/s^2 - the units the row is logged in, so no scaling happens after filtering.
  void push(float ax, float ay, float az,
            float nx, float ny, float nz,
            float gx, float gy, float gz, float vacc);

  // Evaluate all ten and fill the value fields of r. Also fills the unfiltered
  // vacc pair from the delay lines' centre taps, so filtered and unfiltered land on
  // one time base.
  void eval(ImuRow &r) const;

 private:
  FirDecimator ax_, ay_, az_;
  FirDecimator nx_, ny_, nz_;
  FirDecimator gx_, gy_, gz_;
  FirDecimator vacc_;
};

#endif  // FIR_ROW_BANK_H
