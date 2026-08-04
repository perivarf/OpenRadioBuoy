#ifndef IMU_SAMPLER_H
#define IMU_SAMPLER_H

#include <Arduino.h>
#include <SPI.h>
#include <LSM6DSV16XSensor.h>
#include "wave_config.h"
#include "fir.h"
#include "fir_coeffs.h"   // kFirCoeffsStage1 - generated, see ORB_test/tools/gen_fir_table.py
#include "quat_delay.h"

/*
  One IMU row per kWindowMs, ported from ORB_test Imu.h.
  Shared type: ImuSampler produces it, the analyzer and CSV logger consume it.

  ONE TIME BASE PER ROW. Every value here describes the same instant: the centre of
  the window, carried back by the decimating FIR's group delay (kFirS1DelayS, 66.7 ms
  at 960 Hz). That includes the quaternions, which are not filtered but ARE delayed
  by the same amount - see quat_delay.h for why those are two different decisions.
  The row's timestamp, winStartMs, still names the window on the plain 10 ms grid; it
  is a label for the window, not a claim about which instant the values describe.
*/
struct ImuRow {
  uint32_t winStartMs;              // window start, relative ms from capture start
  uint16_t n;                       // accel samples in the window (quality metric only)
  float ax, ay, az;                 // FIR-decimated accel (mg, body frame)
  float axn, ayn, azn;              // FIR-decimated linear accel (mg, world/NED, gravity removed)
  float gx, gy, gz;                 // FIR-decimated gyro (mdps)
  float qw, qx, qy, qz;             // on-chip SFLP quaternion, delay-matched to the above
  uint8_t braking;                  // 1 if linear |a| > threshold long enough
  uint8_t fifoOvf;                  // 1 if the FIFO overflowed while this window was open
  float mqw, mqx, mqy, mqz;         // Madgwick/Kalman quaternion, delay-matched to the above
  float vaccMadgwick, vaccSflp;     // UNFILTERED vertical linear accel (m/s^2) at the same instant
  float vaccFir, vaccSflpFir;       // the same two series, FIR-decimated
  uint8_t sflpNan;                  // 1 if a NaN SFLP quaternion was rejected in the window
};

// Callback invoked when a window closes. Const again: the row leaves ImuSampler
// complete. It used to be non-const because the analyzer ran the AHRS and had to
// back-fill the orientation fields; the AHRS now lives here, on the raw stream.
using ImuRowSink = void (*)(const ImuRow &);

/*
  The ten series decimated per row. Which ten is not arbitrary: everything logged to
  imu.csv goes through the same filter, so an offline AHRS fed these columns sees the
  same antialias filter the on-device one saw, and no column is a boxcar mean while
  its neighbour is filtered.

  Cost note: a decimating FIR is paid for at its OUTPUT rate. push() is a store, run
  at 960 Hz; eval() is the expensive part and runs at 100 Hz. That is what makes ten
  of them affordable - see fir.h.
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

/*
  IMU driver for the LSM6DSV family. Ported from ORB_test/src/Imu, adapted for the
  buoy: shares the global Arduino SPI object (already brought up by sd_writer on
  SPI1) instead of owning its own bus, and drains the FIFO by polling its fill
  level - no INT1 pin, since the interrupt line is not routed on this PCB.

  This class owns the AHRS. It has to: the raw samples exist nowhere else, and
  running the orientation filter on window means integrated the gyro at a resolution
  it was never measured at. The analyzer downstream is now purely the wave chain.
*/
class ImuSampler {
 public:
  ImuSampler();

  // Init sensor: ODR/FS/filter, FIFO + SFLP batching. Does NOT start the FIFO
  // stream (see startStreaming). Assumes the shared SPI bus is already begun.
  bool begin(Print &dbg);

  // Put the FIFO into STREAM/continuous mode so it starts filling.
  void startStreaming();

  // Flush the hardware FIFO (BYPASS -> STREAM) and clear pending state.
  void resetFifo();

  // Drain all pending FIFO words once (call repeatedly during a capture).
  void update(Print &dbg);

  // Reset windowing for a new capture. captureStartMs is the capture t=0.
  void resetWindowing(uint32_t captureStartMs);

  void setRowSink(ImuRowSink sink) { rowSink_ = sink; }

  uint32_t overflowCount() const { return nOverflow_; }

  // Windows where no raw sample landed on the centre and the FIR had to be read at
  // the window edge instead. Non-zero means FIFO gaps; logged to ana.csv.
  uint32_t firLateEvalCount() const { return nFirLateEval_; }

 private:
  void closeWindow();

  // Evaluate the FIR bank + read the delayed quaternions into pendingRow_.
  void latchRowValues();

  // Print the effective accel/gyro rate + mean magnitudes at most every
  // imu_debug_print_period ms (the ex-reportOncePerSecond, now interval-driven).
  void debugPrintStatus(Print &dbg);

  LSM6DSV16XSensor imu_;
  ImuRowSink rowSink_ = nullptr;

  uint32_t nOverflow_ = 0;
  uint32_t nFirLateEval_ = 0;

  // Debug-rate counters (accumulated per print interval, then reset). The magnitude
  // sums are kept SQUARED and rooted once per print: a sqrt per sample was ~1920
  // double sqrt/s on a soft-float core, spent entirely on a debug line.
  uint32_t dbgLastPrint_ = 0;
  uint32_t nAccDbg_ = 0, nGyrDbg_ = 0;
  double   sumAccMag2_ = 0.0, sumGyrMag2_ = 0.0;
  // Microseconds spent in the AHRS update and in the FIR evaluation since the last
  // print. These are the two new loads in the drain loop and the reason FIFO
  // overflow is the thing to watch after this change, so they are measured rather
  // than estimated: the print reports them as a percentage of wall time.
  uint32_t usAhrs_ = 0, usFir_ = 0;

  // Windowing state: kWindowMs windows off a monotonic accel counter.
  uint32_t sessionStartMs_ = 0;
  bool     logStarted_ = false;
  uint32_t accelIdx_ = 0;
  double   sampleTms_ = 0.0;
  double   samplePeriodMs_ = 1000.0 / kAccelOdrHz;
  int32_t  curWinIdx_ = -1;
  uint16_t winNAcc_ = 0;
  bool     winBraking_ = false;
  bool     winSflpNan_ = false;
  bool     winFifoOvf_ = false;
  bool     winFirDone_ = false;
  uint16_t brakeRun_ = 0;
  float    latestQw_ = 1, latestQx_ = 0, latestQy_ = 0, latestQz_ = 0;

  // AHRS on the raw stream, capped at kAhrsRateCapHz. latestG*_ pairs the gyro with
  // the accel sample that drives the update - they arrive as separate FIFO tags, so
  // the freshest gyro word is the best available match (one sample of skew at most).
  WaveAhrs ahrs_ = makeWaveAhrs();
  bool     ahrsSeeded_ = false;
  uint16_t ahrsN_ = 0;
  float    latestGx_ = 0, latestGy_ = 0, latestGz_ = 0;

  FirRowBank              fir_{kFirCoeffsStage1};
  QuatDelay<kQuatDelaySlots> qDelay_;
  ImuRow                  pendingRow_{};
};

#endif  // IMU_SAMPLER_H
