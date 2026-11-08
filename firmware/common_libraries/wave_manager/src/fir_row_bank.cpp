#include "fir_row_bank.h"

#include "constants.h"
#include "wave_config.h"

// Channels that are compiled out (CsvFir<false>) make these calls no-ops.

void FirRowBank::reset(void) {
  ax_.reset(); ay_.reset(); az_.reset();
  nx_.reset(); ny_.reset(); nz_.reset();
  gx_.reset(); gy_.reset(); gz_.reset();
  vacc_.reset();
}

void FirRowBank::push(float ax, float ay, float az,
                      float nx, float ny, float nz,
                      float gx, float gy, float gz, float vacc) {
  ax_.push(ax); ay_.push(ay); az_.push(az);
  nx_.push(nx); ny_.push(ny); nz_.push(nz);
  gx_.push(gx); gy_.push(gy); gz_.push(gz);
  vacc_.push(vacc);
}

void FirRowBank::eval(ImuRow &r) const {

  // Eval gives value at centre of tap, i.e. aligned with center()
  
  // vacc_ is the only channel the wave chain currently reads.
  r.vaccFir = vacc_.eval();
  r.vacc    = vacc_.center();

  // The rest are written to imu.csv only. A channel that is off (not writing imu.csv, or
  // switched off in analysis_config.h) evaluates to 0, so every field of the reused
  // pendingRow_ is defined either way.
  r.ax = ax_.eval(); r.ay = ay_.eval(); r.az = az_.eval();
  r.axnSflp = nx_.eval(); r.aynSflp = ny_.eval(); r.aznSflp = nz_.eval();
  r.gx = gx_.eval(); r.gy = gy_.eval(); r.gz = gz_.eval();
  r.vaccSflpFir = r.aznSflp * kMg2Ms2;   // filtered

  r.vaccSflp = nz_.center() * kMg2Ms2;   // unfiltered
}
