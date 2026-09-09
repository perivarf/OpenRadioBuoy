#include "wave_analysis.h"

#include <math.h>
#include <string.h>

#include "welch_window.h"  // kWelchWindowTable - generated, see the header for why

/*
  Contains the wave chain (FFT / Welch / spectral moments) plus
  the second decimation stage for imu (ingest). 
*/

// -----------------------------------------------------------------------------
// FFT + Welch
// -----------------------------------------------------------------------------
static float  gRe[kWelchSegLen], gIm[kWelchSegLen];
static double gS2 = 0.0;
static bool   gS2Ready = false;

// In-place bit-reversal permutation (same order as recursive FFT would build up).
// n must be a power of two.
// Ref (bit reverse): Cormen, Leiserson, Rivest, Stein, "Introduction to Algorithms", 3rd ed., §30.3 (ITERATIVE-FFT)
// Ref ARM - reverse bits: https://support.arm.com/documentation/111108/2026-06/Base-Instructions/RBIT--Reverse-bits-?lang=en
static void bitReversePermute(float *re, float *im, int n) {
  
  // The ARM RBIT instruction reverses all 32 bits, but we only need log2(n) bits reversed.
  const int shift = 32 - __builtin_ctz(n);   // 32 - log2(n)

  for (unsigned k = 0; k < (unsigned)n; k++) {
    unsigned j = __RBIT(k) >> shift;         // rev(k)
    if (k < j) {
      float tr = re[k]; re[k] = re[j]; re[j] = tr;
      float ti = im[k]; im[k] = im[j]; im[j] = ti;
    }
  }
}


// Iterative radix-2 Cooley-Tukey FFT (in-place), forward. n = power of two.
// Ref: Cormen, Leiserson, Rivest, Stein, "Introduction to Algorithms", 3rd ed., §30.3 (ITERATIVE-FFT)
static void fft(float *re, float *im, int n) {

  bitReversePermute(re, im, n);

  // Do the FFT
  for (int len = 2; len <= n; len <<= 1) {

    // Euler's formula 
    // (wr=re(w) and wi=im(w) for the twiddle factor w = exp(-2πi/len)
    float ang = -2.0f * (float)M_PI / (float)len;
    float wr = cosf(ang), wi = sinf(ang);

    // The outer loop iterates over the "butterfly" stages, 
    // each of which combines pairs of elements.
    for (int k = 0; k < n; k += len) {

      // cwr and cwi are the current twiddle factors, starting at 1
      // Each iteration of the inner loop multiplies by the twiddle factor to get the next one.
      float cwr = 1.0f, cwi = 0.0f;

      for (int j = 0; j < len / 2; j++) {

        // Calculate the indices of the pair of elements to combine
        int a = k + j;
        int b = k + j + len / 2;

        // Complex product of the twiddle factor and the second half of the pair
        float vr = re[b] * cwr - im[b] * cwi;
        float vi = re[b] * cwi + im[b] * cwr;

        // Update the pair of elements in place
        re[b] = re[a] - vr; 
        im[b] = im[a] - vi;
        
        re[a] += vr;        
        im[a] += vi;

        // Update the twiddle factor for the next iteration
        float nwr = cwr * wr - cwi * wi;
        cwi = cwr * wi + cwi * wr;
        cwr = nwr;
      }
    }
  }
}

/*
  Sum of the squared window weights.
  This is the normalisation that gives a windowed estimate
  the right absolute level. Only done once per boot
*/
static void sumSquaredWelchWeights() {
  if (gS2Ready) return;
  double s2 = 0.0;
  for (int i = 0; i < kWelchSegLen; i++) {
    const float w = kWelchWindowTable[i];
    s2 += (double)w * w;
  }
  gS2 = s2;
  gS2Ready = true;
}

/*
  Window one segment, FFT, accumulate one-sided PSD into psdAcc[0..N/2].

  Runs once every kWelchSegLen / kWelchOverlapDiv samples - 25.6 s at 10 Hz - and is the
  longest uninterruptible stretch in the capture loop. Its cost is therefore a
  FIFO question. It should be called when the FIFO is drained.

  The window is a table lookup, not sinf per sample - see welch_window.h.

  seg is the ring buffer, start is the index of the first sample to be processed
  psdAcc is the accumulator for the one-sided PSD, which is updated in place. It must be
  zeroed before the first call. It is of length kWelchSegLen / 2 + 1, since the Nyquist bin is included.

  Welch, P. (1967). The use of Fast Fourier Transform for the estimation of 
  power spectra: a method based on time averaging over sort, modified periodoghrams. 
  IEEE Transactions on Audio and Electroacoustics, 15(2), 70–73. https://doi.org/10.1109/TAU.1967.1161901

  // Ref: Heinzel, Rüdiger, Schilling (2002), "Spectrum and spectral density
// estimation by the DFT", MPI für Gravitationsphysik. https://dcc.ligo.org/LIGO-T1400010/public

*/
static void accumSegment(const float *seg, uint16_t start, float *psdAcc) {
  
  // Calling sumSquaredWelchWeights() to calculate the sum of squared window weights.
  // It is done here to ensure that it is called at least once. 
  // The sums will only be calculated once, and the result is cached for future calls.
  sumSquaredWelchWeights();

  const int N = kWelchSegLen;
  uint16_t j = start;

  // Welch: "Modified periodogram"
  // - the window is applied to each segment before FFT, and the PSD is averaged over segments.
  for (int i = 0; i < N; i++) {
    gRe[i] = seg[j] * kWelchWindowTable[i];
    gIm[i] = 0.0f;
    if (++j == kWelchRingLen) j = 0;
  }

  // Perform the FFT
  fft(gRe, gIm, N);

  // Calculate the one-sided PSD and accumulate it
  for (int k = 0; k <= N / 2; k++) {

    // Magnitude squared of the complex FFT output
    float mag2 = gRe[k] * gRe[k] + gIm[k] * gIm[k];

    // Heinzel et al. The one-sided PSD is obtained by doubling the power of all bins except for DC and Nyquist
    float scale = (k == 0 || k == N / 2) ? 1.0f : 2.0f;

    // Heinzel et al. The PSD is normalized by the sampling frequency and the sum of squared window weights
    psdAcc[k] += scale * mag2 / ((float)kWelchInputOdrHz * (float)gS2);
  }
}

// Low-frequency half-cosine taper (i.e. a smooth transition from kTaperF1 to kTaperF2).
// See Kohout / Tucker & Pitt 2001.
static inline float lowFreqTaper(float f) {
  if (f <= kTaperF1) return 0.0f;
  if (f >= kTaperF2) return 1.0f;
  return 0.5f * (1.0f - cosf((float)M_PI * (f - kTaperF1) / (kTaperF2 - kTaperF1)));
}

// -----------------------------------------------------------------------------
// StreamAnalyzer
// -----------------------------------------------------------------------------
void StreamAnalyzer::begin(void) {
  fir2_.reset();
  curBucket_ = -1; bucketDone_ = false;
  nWelch_ = nData_ = nBrake_ = nWarm_ = 0;
  head_ = tail_ = fill_ = 0; segPending_ = false;
  nSeg_ = 0; nRingFull_ = 0;
  for (int k = 0; k <= kWelchSegLen / 2; k++) psdAcc_[k] = 0.0f;

  // Calling sumSquaredWelchWeights() for the first time, so it is done outside the capture loop and not in the FIF loop.
  sumSquaredWelchWeights();
}

// Push one sample into the ring. No FFT from here - this runs inside the FIFO pop loop.
// A full segment only raises the flag; processPendingSegment() below does the work.
void StreamAnalyzer::pushWelch(float sample) {
  // Safety valve, not the normal path. If the ring is full, the FFT has to run inside the pop loop after all
  // However, should not be possible with the kWelchRingLen since it includes margin.
  if (fill_ == kWelchRingLen) {
    nRingFull_++;
    processPendingSegment();
  }

  ring_[head_] = sample;
  if (++head_ == kWelchRingLen) head_ = 0;
  fill_++;
  if (fill_ >= kWelchSegLen) segPending_ = true;
}

// The deferred half of pushWelch: FFT + accumulate PSD, then release one step of the
// ring (1-1/kWelchOverlapDiv => 75% overlap keeps the rest). 
// Called from the capture loop with the FIFO just drained
bool StreamAnalyzer::processPendingSegment(void) {
  if (!segPending_) return false;
  segPending_ = false;

  accumSegment(ring_, tail_, psdAcc_);
  nSeg_++;

  const uint16_t step = kWelchSegLen / kWelchOverlapDiv;
  tail_ += step;
  if (tail_ >= kWelchRingLen) tail_ -= kWelchRingLen;
  fill_ -= step;
  return true;
}

void StreamAnalyzer::ingest(const ImuRow &r) {
  nData_++;
  if (r.braking) nBrake_++;

  long t = (long)r.winStartMs;
  
  const float v = isfinite(r.vaccFir) ? r.vaccFir : 0.0f;

  // Stage 2 is fed on every row, warm-up included. The delay line has to be full by
  // the time the first Welch sample is taken
  fir2_.push(v);

  // Warm-up: the AHRS has not converged yet (and the FIR is still filling), so the
  // vertical accel is less reliable. The rows are kept out of the Welch segment
  if (t < (long)wave_measurement_filter_warm_up) {
    nWarm_++;
    return;
  }

  // Decimate to kWelchInputOdrHz

  // First we calculate the bucket number
  // In case of a new bucket, we update the current bucket and reset the bucketDone_ flag
  long bucket = t / kWelchInputPeriodMs;
  if (bucket != curBucket_) {
    curBucket_ = bucket;
    bucketDone_ = false;
  }

  // Evaluate the FIR at the centre of the bucket and push it into the Welch ring buffer
  // The FIR delay is kFirHalf = (kFirNtap - 1) / 2 measurements.
  if (!bucketDone_ && t >= bucket * (long)kWelchInputPeriodMs + (long)kFirS2CenterMs) {
    float s = fir2_.eval();
    pushWelch(isfinite(s) ? s : 0.0f);
    nWelch_++;
    bucketDone_ = true;
  }
}

bool StreamAnalyzer::finalize(WaveParams &params, uint16_t *spectrumOut) {

  // If any pending segments, process them now
  processPendingSegment();


  // Parameters are initialized to -1.0f for the wave parameters and 0.0 for the spectral moments.
  params = {-1.0f, -1.0f, -1.0f, -1.0f, 0.0f, 0.0, 0.0, 0.0};

  // Reset the spectrum output
  for (size_t j = 0; j < welch_bins; j++) spectrumOut[j] = 0;
  if (nSeg_ == 0) return false;


  const int N = kWelchSegLen;
  const float df = (float)kWelchInputOdrHz / N;
  const float invSeg = 1.0f / (float)nSeg_;

  // Spectral moments + peak, over the elevation PSD (acc PSD / omega^4 * taper^2).
  float peakEta = 0.0f, peakF = 0.0f;

  // Loop over the one-sided PSD bins, skipping DC (k=0) since it is not a wave
  for (int k = 1; k <= N / 2; k++) {

    // Frequency corresponding to the k-th bin
    float f = k * df;

    // Skip frequencies above the maximum wave frequency
    if (f > kWaveFMax) break;

    // Get low-frequency taper for the current frequency. 
    // If the taper is zero or negative, skip this bin.
    float taper = lowFreqTaper(f);
    if (taper <= 0.0f) continue;

    float w = 2.0f * (float)M_PI * f; //omega

    // Calculate the elevation PSD (eta) from the acceleration PSD, normalizing by the number of segments (psdAcc is sum of nSeq_ PSDs)
    // and applying the taper (sqared since going from acc to elevation).
    float psdEta = (psdAcc_[k] * invSeg) / (w * w * w * w) * (taper * taper);

    // Calculate the spectral moments m0, m2, m4 and find the peak frequency and value
    params.m0 += (double)psdEta * df;                       // m0 = integral of S(f) df
    params.m2 += (double)psdEta * f * f * df;               // m2 = integral of f^2 S(f) df
    params.m4 += (double)psdEta * f * f * f * f * df;       // m4 = integral of f^4 S(f) df
    if (psdEta > peakEta) { peakEta = psdEta; peakF = f; }  // Find the peak frequency and value
  }

  // Calculate the significant wave height (Hs) and other wave parameters
  if (params.m0 > 0) params.hs = 4.0f * sqrtf((float)params.m0);                          // Hs = 4 * sqrt(m0)
  if (params.m0 > 0 && params.m2 > 0) params.tz = sqrtf((float)(params.m0 / params.m2));  // Tz = sqrt(m0/m2)
  if (params.m2 > 0 && params.m4 > 0) params.tc = sqrtf((float)(params.m2 / params.m4));  // Tc = sqrt(m2/m4)
  if (peakF > 0) params.tp = 1.0f / peakF;                                                // peak period from the spectral peak

  /*
    The spectrum sent on wire is acceleration with no taper. 
  */

  // Normalisation peak: the largest unaveraged acceleration PSD bin inside the transmitted range
  float peakAcc = 0.0f;
  for (size_t k = welch_bin_min == 0 ? 1 : welch_bin_min; k < welch_bin_max; k++) {
    const float p = psdAcc_[k] * invSeg;
    if (p > peakAcc) peakAcc = p;
  }

  params.maxValue = peakAcc;

  // Quantise: acceleration PSD over bins welch_bin_min..max, normalised to peakAcc
  // so it fits uint16; the absolute peak is stored in params.maxValue.
  //
  // Each wire bin is the average of kSpecBinGroup consecutive PSD bins,
  // that is what lets kSpecNBins span the whole wave band within one message. 
  // Only kSpecNBins is sent, not welch_bins, to save airtime
  // 
  // Normalising against the unaveraged peak. The receiving side reconstructs
  // the acceleration PSD on the far side as (value/65535)^2 * maxValue. 
  //
  
  if (peakAcc > 0.0f) {
    for (size_t j = 0; j < kSpecNBins; j++) {
      float acc = 0.0f;
      for (size_t g = 0; g < kSpecBinGroup; g++) {

        const int k = (int)welch_bin_min + (int)(j * kSpecBinGroup + g);
        
        // k == 0 is DC. The segment mean is removed before the FFT, so that bin holds
        // no wave information - only whatever offset survived detrending
        if (k == 0) continue;

        // Calculate the PSD-acceleration for this bin
        const float psd = psdAcc_[k] * invSeg;
        if (psd > 0.0f) acc += psd;
      }

      // Calculating normalization (average value of the group divided by peak acceleration)
      float norm = (acc / (float)kSpecBinGroup) / peakAcc;
      if (norm > 1.0f) norm = 1.0f; // In case of rounding errors

      // Store the quantized value in the output spectrum (squared root of the normalized value [0,1] multiplied by 65535)
      spectrumOut[j] = (uint16_t)lroundf(sqrtf(norm) * 65535.0f);
    }
  }
  return true;
}
