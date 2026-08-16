#include "fir_row_bank.h"

#include "constants.h"
#include "wave_config.h"

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

  // Eval gives value at centre of tap, so the series are aligned
  // for both filtered and unfiltered.

  // vacc_ is the only channel the wave chain reads - StreamAnalyzer::ingest takes
  // r.vaccFir from the row and nothing else that comes from this bank - so it is
  // evaluated unconditionally, filtered and unfiltered alike.
  r.vaccFir = vacc_.eval();
  r.vacc    = vacc_.center();

  /*
    The other nine are imu.csv columns and nothing else. Each of them has exactly one
    reader: the print run in WaveManager::onRow, behind its early return on a closed
    file. In WaveLogMode::Raw that file is never opened, and evaluating them anyway was
    9/10 of TIM_FIR - 210 of 234 ms/s, the largest single per-second cost in the capture
    loop - spent on nobody.

    The gate is the COMPILE-TIME mode, not the runtime imuCsvActive_. That flag is also
    false when the file merely failed to open, and a Csv capture must not quietly lose
    nine columns because the sd-card had a bad day; it has to fail the way it does today.

    Only the convolution is skipped. push() still runs for all ten (ImuSampler::update),
    so every delay line stays warm and this remains a change to eval() alone - see
    fir.h on why the two are separate and which of them carries the cost.
  */
  if constexpr (wave_mode_imu_csv()) {
    // Filtered
    r.ax = ax_.eval(); r.ay = ay_.eval(); r.az = az_.eval();
    r.axnSflp = nx_.eval(); r.aynSflp = ny_.eval(); r.aznSflp = nz_.eval();
    r.gx = gx_.eval(); r.gy = gy_.eval(); r.gz = gz_.eval();
    r.vaccSflpFir = r.aznSflp * kMg2Ms2;

    // Unfiltered
    r.vaccSflp = nz_.center() * kMg2Ms2;
  } else {
    // Zeroed rather than left alone: pendingRow_ is reused across rows, so a field
    // nothing writes would carry an indeterminate value under a real column name. No
    // file can ever receive these - their only writer is gated on the same constant -
    // but the row leaves here fully defined either way.
    r.ax = r.ay = r.az = 0.0f;
    r.axnSflp = r.aynSflp = r.aznSflp = 0.0f;
    r.gx = r.gy = r.gz = 0.0f;
    r.vaccSflpFir = r.vaccSflp = 0.0f;
  }
}
