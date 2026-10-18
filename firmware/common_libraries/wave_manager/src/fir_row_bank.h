#ifndef FIR_ROW_BANK_H
#define FIR_ROW_BANK_H

#include "fir.h"      // FirDecimator - one per channel
#include "imu_row.h"  // ImuRow - what eval() fills
#include "log_config.h"       // waveModeImuCsv()
#include "analysis_config.h"  // kFirAx ... kFirGz

/*
  The ten series decimated per row - everything imu.csv logs, all through the same
  filter, so an offline AHRS sees the antialiasing the on-device one saw.

  ax,ay,az: accel in mg, body frame
  nx,ny,nz: linear accel in mg, gravity frame, gravity removed, rotated by the SFLP quaternion
  gx,gy,gz: gyro in mdps, body frame
  vacc: vertical linear accel in m/s^2, gravity frame, gravity removed, rotated
*/

// One imu.csv channel. On = false is an empty type: no delay line, no code, eval() = 0.
template <bool On> struct CsvFir {
  explicit CsvFir(const float *c) : f(c) {}
  void  reset() { f.reset(); }
  void  push(float x) { f.push(x); }
  float eval() const { return f.eval(); }
  float center() const { return f.center(); }
  FirDecimator f;
};
template <> struct CsvFir<false> {
  explicit CsvFir(const float *) {}
  void  reset() {}
  void  push(float) {}
  float eval() const { return 0.0f; }
  float center() const { return 0.0f; }
};

// Enabled only when imu.csv is written AND the channel is switched on in analysis_config.h
template <bool Flag> using CsvChan = CsvFir<waveModeImuCsv() && Flag>;

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
  CsvChan<kFirAx> ax_; CsvChan<kFirAy> ay_; CsvChan<kFirAz> az_;
  CsvChan<kFirNx> nx_; CsvChan<kFirNy> ny_; CsvChan<kFirNz> nz_;
  CsvChan<kFirGx> gx_; CsvChan<kFirGy> gy_; CsvChan<kFirGz> gz_;
  FirDecimator vacc_;
};

#endif  // FIR_ROW_BANK_H
