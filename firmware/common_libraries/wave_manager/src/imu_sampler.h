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
  One IMU row per kRowPeriodMs, ported from ORB_test Imu.h.
  Shared type: ImuSampler produces it, the analyzer and CSV logger consume it.

  ONE TIME BASE PER ROW. Every value here describes the same instant: the centre of
  the window, carried back by the decimating FIR's group delay (kFirS1DelayS, 66.7 ms
  at 960 Hz). That includes the quaternions, which are not filtered but ARE delayed
  by the same amount - see quat_delay.h for why those are two different decisions.
  The row's timestamp, winStartMs, still names the window on the plain kRowPeriodMs grid; it
  is a label for the window, not a claim about which instant the values describe.
*/
struct ImuRow {
  uint32_t winStartMs;                  // window start, relative ms from capture start
  uint16_t n;                           // accel samples in the window (quality metric only)
  float ax, ay, az;                     // FIR-decimated accel (mg, body frame)
  float axnSflp, aynSflp, aznSflp;      // FIR-decimated linear accel rotated by the SFLP
                                        // quaternion (mg, world frame, gravity removed)
  float gx, gy, gz;                     // FIR-decimated gyro (mdps)
  float qwSflp, qxSflp, qySflp, qzSflp; // on-chip SFLP quaternion, delay-matched to the above
  uint8_t braking;                      // 1 if linear |a| > threshold long enough
  uint8_t fifoOvf;                      // 1 if the FIFO overflowed while this window was open
  float qw, qx, qy, qz;                 // the SELECTED WaveAhrs (Madgwick/Kalman) quaternion,
                                        // delay-matched to the above
  float vacc, vaccSflp;                 // UNFILTERED vertical linear accel (m/s^2) at the same
                                        // instant: selected method, and SFLP
  float vaccFir, vaccSflpFir;           // the same two series, FIR-decimated
  uint8_t sflpNan;                      // 1 if a NaN SFLP quaternion was rejected in the window
};

// Callback invoked when a window closes. Const again: the row leaves ImuSampler
// complete. It used to be non-const because the analyzer ran the AHRS and had to
// back-fill the orientation fields; the AHRS now lives here, on the raw stream.
using ImuRowSink = void (*)(const ImuRow &);

// Raw-log sink: one filled block of <stamp>_raw.bin. See wave_raw_log in
// wave_config.h for the record layout.
//
// Returns false if the block did not reach the card in full. The sampler cannot fix
// that, but it must KNOW: a partial block desynchronises every byte after it, so the
// next sync record carries kRawFlagWriteFail and the decoder can stop trusting the
// file at the right place instead of quietly reading payload bytes as tags.
using RawBlockSink = bool (*)(const uint8_t *data, uint16_t len);

/*
  The ten series decimated per row. Which ten is not arbitrary: everything logged to
  imu.csv goes through the same filter, so an offline AHRS fed these columns sees the
  same antialias filter the on-device one saw, and no column is a boxcar mean while
  its neighbour is filtered.

  Cost note: a decimating FIR is paid for at its OUTPUT rate. push() is a store, run
  at kImuOdrHz; eval() is the expensive part and runs at kRowOdrHz. That is what
  makes ten of them affordable, and it also means lowering the ROW rate does not make
  them cheaper by much - lowering the ODR does - see fir.h.
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
  SPI1) instead of owning its own bus, and drains the FIFO by polling its fill level.
  INT1 is routed on the PCB but unused: no pin for it is defined in common_config.h
  and nothing attaches to it, so the drain cadence is the main loop's, not the
  sensor's. Wiring it up is the obvious lever if polling ever proves too coarse.

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

  // Raw log (wave_raw_log): the sink is handed whole blocks, never single records -
  // a 7-byte write per FIFO word would put ~1200 SdFat calls a second inside the
  // drain. At kRawBlockBytes it is ~20 a second instead.
  void setRawSink(RawBlockSink sink) { rawSink_ = sink; }

  // Push the partial block. Call at end of capture, or the tail is lost.
  void flushRaw(void);

  uint32_t overflowCount() const { return nOverflow_; }

  // FIFO fills for the WHOLE capture, cleared only by resetWindowing. nOverflow_ is
  // the debug print's own counter and is zeroed every time it prints, so it cannot
  // answer "was this capture clean?" - this one can, and goes to ana.csv.
  uint32_t overflowTotal() const { return nOverflowTotal_; }

  // Windows where no raw sample landed on the centre and the FIR had to be read at
  // the window edge instead. Non-zero means FIFO gaps; logged to ana.csv.
  uint32_t firLateEvalCount() const { return nFirLateEval_; }

  // Blocks the raw sink failed to write in full. Judge the FILE by this, the way
  // overflowTotal() judges the CAPTURE: zero here with a non-zero overflowTotal is a
  // rough but readable capture, whereas non-zero here means raw.bin is desynchronised
  // past the first failure no matter how clean the sensor was.
  uint32_t rawWriteFailCount() const { return nRawWriteFail_; }

 private:
  // INT1 watermark plumbing. attachInterrupt takes a plain function, so the ISR is a
  // static trampoline that finds the one instance through s_self - the same pattern
  // WaveManager uses for its row sink, and the one ORB_test's Imu uses here. The ISR
  // does nothing but set the flag; all work happens in update().
  static ImuSampler *s_self;
  static void isrTrampoline();
  volatile bool fifoFlag_ = false;
  uint32_t lastDrainMs_ = 0;   // deadline for the INT1 gate; see kFifoPollFallbackMs

  // Raw auto-incrementing register burst on the shared SPI bus (ORB_test's
  // Imu::imuBurstRead). One CS-low transfer for len consecutive registers.
  void imuBurstRead(uint8_t startReg, uint8_t *buf, uint8_t len);

  // Pop one FIFO word atomically: tag + 6 payload bytes in a single transfer.
  // Returns tag_sensor. See the definition for why the split the driver does is
  // not merely slower but wrong under FIFO pressure.
  uint8_t readFifoWord(uint8_t payload[6]);

  void closeWindow();

  // Evaluate the FIR bank + read the delayed quaternions into pendingRow_.
  void latchRowValues();

  // Print the effective accel/gyro rate + mean magnitudes at most every
  // imu_debug_print_period ms (the ex-reportOncePerSecond, now interval-driven).
  void debugPrintStatus(Print &dbg);

  // Raw log helpers. rawAppend flushes whenever the block is full, so a record may
  // straddle a block boundary - the file is a byte stream, not an array of blocks.
  void rawAppend(const uint8_t *p, uint8_t n);
  void rawEmitWord(uint8_t tag, const uint8_t payload[6]);
  void rawEmitSync(uint16_t nWords, uint16_t flags);

  LSM6DSV16XSensor imu_;
  ImuRowSink rowSink_ = nullptr;
  RawBlockSink rawSink_ = nullptr;
  uint8_t  rawBuf_[kRawBlockBytes];
  uint16_t rawLen_ = 0;
  uint32_t nRawWriteFail_ = 0;
  // Sticky: set the moment a block is lost, cleared only once a sync record has
  // actually carried it into the file. Without the stickiness the report could itself
  // be the write that fails, and the loss would go unrecorded in the stream.
  bool     rawWriteFailPending_ = false;

  uint32_t nOverflow_ = 0;
  uint32_t nOverflowTotal_ = 0;
  uint32_t nFirLateEval_ = 0;

  // Debug-rate counters (accumulated per print interval, then reset). The magnitude
  // sums are kept SQUARED and rooted once per print: a sqrt per sample was ~1920
  // double sqrt/s on a soft-float core, spent entirely on a debug line.
  uint32_t dbgLastPrint_ = 0;
  uint32_t nAccDbg_ = 0, nGyrDbg_ = 0;
  double   sumAccMag2_ = 0.0, sumGyrMag2_ = 0.0;

  // Windowing state: kRowPeriodMs windows off a monotonic accel counter.
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

  // AHRS on the raw stream, one update per accel sample. latestG*_ pairs the gyro
  // with the accel sample that drives the update - they arrive as separate FIFO
  // tags, so the freshest gyro word is the best available match (one sample of skew).
  WaveAhrs ahrs_ = makeWaveAhrs();
  bool     ahrsSeeded_ = false;
  float    latestGx_ = 0, latestGy_ = 0, latestGz_ = 0;

  FirRowBank              fir_{kFirCoeffsStage1};
  QuatDelay<kQuatDelaySlots> qDelay_;
  ImuRow                  pendingRow_{};
};

#endif  // IMU_SAMPLER_H
