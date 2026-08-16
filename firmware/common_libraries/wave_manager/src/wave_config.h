#ifndef WAVE_CONFIG_H
#define WAVE_CONFIG_H

#include <Arduino.h>
#include <LSM6DSV16XSensor.h>

#include "config.h"
#include "madgwick.h"
#include "kalman.h"
#include "fir.h"
#include "fir_coeffs.h"
/*
  IMU + wave-analysis tuning
*/

// Only ONE orientation method is computed per capture. The selection itself
// imu.csv can be logged raw so that post-evaluation is posstible

// Per-capture SD logging: one directory "<stamp>", never renamed. Up to six files
// inside, all prefixed "<stamp>_": imu, gps, ses, cfg, spec, ana.

// An interrupted capture is told apart by what is MISSING, not by the folder name:
// spec/ana are written by processReading and ses.csv gets its summary block there,
// so a folder without ana.csv - or a ses.csv with no stop_utc_epoch - never reached
// the end. That fact is a plain consequence of the write order and needs no separate
// marker to be kept in sync with it.

// Misc timing calculations:

// With kFifoDepthWords (256), rates f_imu and f_sflp the FIFO fills up in 
// TimeToFillUpImu = kFifoDepthWords / (2·f_imu + f_sflp) = (for instance) 256 / (2·480 + 240) = 256 / 1200 = 213 ms

// Draining N words depends on SPI rate. One word is 6B payload (x,y,z x 2B) + 1B tag. Address tag is 1B, so 8B total.
// The floor would be
// t_spi = N · (sizeof(addresstag) + sizeof(payload) + sizeof(tag)) B · 8 bit / f_spi  = 256 · 64 bit / 8 MHz = 2.05 ms

// I.e. draining the FIFO is fast enough, the bottleneck is calculating AHRS filter + writing to SD card.
// Writing this to SD card depends on the card speed class etc

// Writing RAW-data to SD. 
// 
// N values (for instance 256) would be 256 · 8 B = 2 kB. A class 10 card is guaranteed to write at least 10 MB/s, so the floor is
// t_write_sd = N · 8 B · 8 bit / 10 MB/s = 256 · 64 bit / 10 MB/s = 1.64 ms
// 
// At the same time, we are also limited by SPI speed of 12Mhz, which gives transfer time
// t_transfer_sd = N · 8 B · 8 bit / 12 MHz = 256 · 64 bit / 12 MHz = 1.37 ms

// So if we want i.e. 50% duty cycle for IMU-processing, we need to use at most 
// TimeToFillUpImu * 50% = 106 ms to drain the FIFO, calculate AHRS and write to SD.


static constexpr bool     wave_log_csv                    {true};
static constexpr uint16_t wave_csv_sync_rows              {1024}; // File.sync() cadence

// Row widths for preAllocate(). Without preAllocate, too much time is spent on allocating
// clusters, and potentially overruns the FIFO (512 words is ~240 ms of headroom at 960 Hz).
// It must run before the first byte is written,
// and the file stays oversized until truncate() at close. 
// The widths are worst case, as it is performed before the hot path, and truncated after

static constexpr uint16_t wave_imu_row_bytes_max          {256};
// 96 held the compact integer gps.csv (~60 B/row). The decoded 16-column format is
// ~95 B on a real fix and 139 in the worst case every field can print (signed
// lat/lon at 6 decimals, four velocities at 4, three accuracies that are uint32
// millimetres), so the old budget would be outgrown MID-CAPTURE - and that is the
// one failure preAllocate cannot absorb: see the extent comment in wave_manager.cpp
// startSession, and the 2026-08-12 captures that died ~7 min in.
static constexpr uint16_t wave_gps_row_bytes_max          {160};

static constexpr char     wave_log_dir[]     = "waves";  // parent dir for session folders
#define WAVE_IMU_PREFIX      "imu"
#define WAVE_GPS_PREFIX      "gps"
#define WAVE_SESSION_PREFIX  "ses"
#define WAVE_SPEC_PREFIX     "spec"
#define WAVE_ANA_PREFIX      "ana"
#define WAVE_CFG_PREFIX      "cfg"
#define WAVE_RAW_PREFIX      "raw"


// How long takeReading() waits for a valid GNSS solution before it gives up and
// aborts the capture. 
static constexpr uint32_t wave_gps_fix_timeout {120000};  // ms

// Whether that gps timeout ABORTS the capture or merely ends the wait. 
static constexpr bool wave_measurement_require_gps {true};

// -----------------------------------------------------------------------------
// IMU: datarate (ODR) + power mode for accel AND gyro.
// -----------------------------------------------------------------------------

static constexpr uint16_t kImuOdrHz    = 480;   // 120/240/480/960 Hz
static constexpr uint8_t  kImuLowPower = 0;     // 1 = low power (ODR<=240), 0 = high performance
static_assert(kImuOdrHz == 120 || kImuOdrHz == 240 || kImuOdrHz == 480 || kImuOdrHz == 960,
              "kImuOdrHz must be 120, 240, 480 or 960 Hz");
static_assert(!(kImuLowPower && kImuOdrHz > 240),
              "Low-power is only valid for ODR <= 240 Hz");

static constexpr LSM6DSV16X_ACC_Operating_Mode_t kImuAccMode =
    kImuLowPower ? LSM6DSV16X_ACC_LOW_POWER_MODE1 : LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE;
static constexpr LSM6DSV16X_GYRO_Operating_Mode_t kImuGyrMode =
    kImuLowPower ? LSM6DSV16X_GYRO_LOW_POWER_MODE : LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE;

// SPI clock for the IMU. LSM6DSV16X is rated max 10 MHz - do NOT exceed (corrupt
// FIFO reads / NaN). 8 MHz drains the FIFO ~4x faster than 2 with margin to 10 MHz.
static constexpr uint32_t kImuSpiHz = 8000000;

// Upper frequency of the analysed band
static constexpr float kWaveFMax = 1.0f;

// -----------------------------------------------------------------------------
// Accel LPF2. The cutoff is specified as a FRACTION OF ODR (CTRL8.hp_lpf2_xl_bw),
// so pinning one enum makes it move with kImuOdrHz:
// STRONG is 9.6 Hz at 960 Hz but 1.2 Hz at 120 Hz, on top of the analysed band.
// The divisors are datasheet values
// -----------------------------------------------------------------------------

static constexpr bool kUseLpf2 = true;

// How far above kWaveFMax the cutoff must sit. 4x keeps the in-band droop small while
// still landing near the 5 Hz Nyquist of the 10 Hz series this is decimated to, so
// LPF2 does the anti-alias work up front rather than instead of the FIR stages.
static constexpr float kLpf2Margin = 4.0f;
static constexpr float kLpf2MinHz  = kLpf2Margin * kWaveFMax;   // 4.0 Hz @ kWaveFMax 1.0

// Strongest LPF2 (largest divisor) whose cutoff still clears kLpf2MinHz. The hardware
// offers only these eight divisors, so this picks from the list rather than computing
// a number. At kWaveFMax 1.0 that is 200/100/45/20 for ODR 960/480/240/120.
static constexpr uint16_t lpf2DivForOdr(uint16_t odr) {
  return (float)odr >= 800.0f * kLpf2MinHz ? 800
       : (float)odr >= 400.0f * kLpf2MinHz ? 400
       : (float)odr >= 200.0f * kLpf2MinHz ? 200
       : (float)odr >= 100.0f * kLpf2MinHz ? 100
       : (float)odr >=  45.0f * kLpf2MinHz ?  45
       : (float)odr >=  20.0f * kLpf2MinHz ?  20
       : (float)odr >=  10.0f * kLpf2MinHz ?  10
       :                                        4;
}
static constexpr uint16_t kLpf2Div = lpf2DivForOdr(kImuOdrHz);

// Divisor -> CTRL8.hp_lpf2_xl_bw register value
static constexpr uint8_t lpf2BwForDiv(uint16_t div) {
  return div ==   4 ? LSM6DSV16X_XL_ULTRA_LIGHT   // ODR/4
       : div ==  10 ? LSM6DSV16X_XL_VERY_LIGHT    // ODR/10
       : div ==  20 ? LSM6DSV16X_XL_LIGHT         // ODR/20
       : div ==  45 ? LSM6DSV16X_XL_MEDIUM        // ODR/45
       : div == 100 ? LSM6DSV16X_XL_STRONG        // ODR/100
       : div == 200 ? LSM6DSV16X_XL_VERY_STRONG   // ODR/200
       : div == 400 ? LSM6DSV16X_XL_AGGRESSIVE    // ODR/400
       :              LSM6DSV16X_XL_XTREME;       // ODR/800
}
static constexpr uint8_t kLpf2Bw       = lpf2BwForDiv(kLpf2Div);
static constexpr float   kLpf2CutoffHz = (float)kImuOdrHz / kLpf2Div;  // 4.8 Hz @ 960 Hz

static_assert(kLpf2CutoffHz >= kLpf2MinHz,
              "no LPF2 divisor clears kWaveFMax by kLpf2Margin - raise kImuOdrHz, "
              "lower kWaveFMax, or accept a smaller margin");

// -----------------------------------------------------------------------------
// Full-scale range. The enum value is the number passed to Set_X_FS/Set_G_FS (g / dps),
// and the raw-FIFO sensitivity is derived from it below, so range and scaling cannot
// drift apart. A larger range captures bigger motion at coarser resolution.
// -----------------------------------------------------------------------------

enum class AccelFS : uint8_t  { G2 = 2, G4 = 4, G8 = 8, G16 = 16 };
enum class GyroFS  : uint16_t { DPS125 = 125, DPS250 = 250, DPS500 = 500,
                                DPS1000 = 1000, DPS2000 = 2000, DPS4000 = 4000 };

static constexpr AccelFS kAccelFS = AccelFS::G2;      // +-2 g
static constexpr GyroFS  kGyroFS  = GyroFS::DPS1000;  // +-1000 dps

// LSM6DSV16X sensitivities (datasheet), selected from the ranges above.
static constexpr float accSensMgPerLsb(AccelFS fs) {
  return fs == AccelFS::G2 ? 0.061f
       : fs == AccelFS::G4 ? 0.122f
       : fs == AccelFS::G8 ? 0.244f
       :                     0.488f;   // G16
}
static constexpr float gyrSensMdpsPerLsb(GyroFS fs) {
  return fs == GyroFS::DPS125  ? 4.375f
       : fs == GyroFS::DPS250  ? 8.75f
       : fs == GyroFS::DPS500  ? 17.5f
       : fs == GyroFS::DPS1000 ? 35.0f
       : fs == GyroFS::DPS2000 ? 70.0f
       :                         140.0f;  // DPS4000
}
static constexpr float kAccSensMgPerLsb   = accSensMgPerLsb(kAccelFS);
static constexpr float kGyrSensMdpsPerLsb = gyrSensMdpsPerLsb(kGyroFS);

// -----------------------------------------------------------------------------
// FIFO watermark + INT1. The sensor drives INT1 high at kFifoWatermark words, so the
// drain runs on the sensor's cadence and update() costs a flag test otherwise.
//
// The budget every value here is spent against: accel + gyro + SFLP are batched at
// their own ODRs, so the FIFO takes 2*kImuOdrHz + kSflpOdrHz words a second - 2160 at
// 960 Hz, 1200 at 480. Divided into kFifoDepthWords that gives the time the drain has
// before the oldest word is overwritten: 118 ms at 960 Hz, 213 ms at 480. At 64 words
// the watermark burst arrives every ~30 ms (960 Hz) / ~53 ms (480), well inside either.
//
// NB: INT1 er ikke her for responstidens skyld - løkka i wave_manager kaller update()
// hver løkkerunde uansett. Det INT1 gjør, er å HOLDE IGJEN dreneringen til
// kFifoWatermark ord har samlet seg. Det er en batching-mekanisme, og vaktmerket er
// batchstørrelsen. Se kFifoWatermark.
//
// false velger den ANDRE utløseren, ikke fravær av en: update() leser da FIFO-nivået og
// holder igjen på samme vaktmerke. Fram til 2026-08-15 sto det «ren polling» her, og
// koden gjorde nettopp det - den drenerte hver løkkerunde uansett nivå. Det gikk
// upåaktet så lenge løkka var treg, men da GPS-arbeidet falt fra 17 til 4.4 ms per
// runde, ble det 169 dreneringer i sekundet med 6.9 ord hver: 2.9 kB/s sync-poster i
// råloggen som preAllocate aldri budsjetterte for. Batchingen må komme fra vaktmerket,
// ikke fra at løkka tilfeldigvis er treg nok.
//
// SATT TIL true 2026-08-16, og fristen er samtidig tatt ut av INT1-porten i update().
// Det er et bevisst valg, og prisen skal stå her:
//
//   * Gevinsten er statuslesingen på de kallene som ikke drenerer - ~0.55 % CPU målt
//     over et 20 s vindu (7371 lesinger a 15 us). Overrun-latchingen blir også renere:
//     FIFO_OVR_LATCHED leses kun ved drenering, så pendingOvrLatched_-viderebæringen i
//     port 2 er ikke lenger i bruk.
//   * Prisen er TAPT FLANKE. Avbruddet er nivåstyrt men koblet på RISING: ender en
//     drenering med FIFO-en fortsatt over vaktmerket, faller aldri linja, ingen ny
//     flanke kommer, og strømmen dør stille. Målt 2026-08-04: 0 Hz, forever, no warning.
//   * Med fristen ute er re-armingen i update() (etter dreneringen, `if (after.level >=
//     kFifoWatermark) fifoFlag_ = true`) DET ENESTE som henter en tapt flanke tilbake.
//     Den dekker tilfellet som betyr noe i praksis - etterspillet av en sd-stall, der
//     nivået ER over vaktmerket. Den dekker IKKE en flanke som forsvinner mens nivået
//     ligger under: elektrisk glipp, eller en ISR som ikke fyrer. Da er capturen tapt
//     til neste reset, og det er den tilstanden fristen fantes for å bryte.
//
// INT1 gir for øvrig ikke begrenset responstid, og bør ikke leses som om den gjør det:
// ISR-en setter kun fifoFlag_. Selve dreneringen kjører fortsatt fra capture-løkka, bak
// den samme sd-skrivingen og den samme Welch-FFT-en. Avbruddet endrer NÅR man får vite
// at FIFO-en er klar, ikke når man får handle på det - og derfor er dette ikke et
// argument for å heve kFifoWatermark.
// -----------------------------------------------------------------------------

static constexpr bool kImuUseInt1 = true;

// FIFO-dybden i ord. IKKE 512, som denne fila hevdet fram til 2026-08-14:
//
//   "The LSM6DSV embeds 1.5 KB of data in FIFO (up to 4.5 KB with the compression
//    feature enabled)"                          - DS13476 rev 5, section 6.10, p.42
//
// Hvert nivå bærer 6 databyte (tag-byten leses ut i tillegg, derav 7 byte per ord
// over bussen), så 1536 / 6 = 256 nivåer. De 9 bitene i FIFO_STATUS' nivåfelt er der
// for de KOMPRIMERTE modusene (4.5 kB = 3x så mange sampler); de sier ingenting om
// dybden i denne konfigurasjonen, og å utlede 512 av feltbredden var nettopp feilen.
//
// Målt, ikke bare lest: nivået metter på nøyaktig 256 i tre uavhengige fangster
// (20260813_182728 og begge 20260814-øktene), truffet 12-18 ganger hver, aldri over.
// Det halverer hvert timing-budsjett under denne overskriften.
static constexpr uint16_t kFifoDepthWords = 256;

// Watermark in FIFO words. FIFO_Set_Watermark_Level takes a uint8_t (FIFO_CTRL1.WTM
// is 8 bits), so this must stay under 256 - which is the whole buffer, not half of it.
//
// Dette tallet er ikke bare en dreneringstakt: det er det STÅENDE nivået i FIFO-en, og
// dermed hvor lite ledig plass som er igjen når noe som helst blokkerer. Ved 480 Hz og
// 1200 ord/s:
//
//   WTM 128 -> 128 ledige nivåer = 107 ms budsjett; tapt INT1-flanke (80 ms) legger
//              96 ord til de 128, altsaa 224 av 256, og 27 ms igjen til sd-skrivingen.
//   WTM  64 -> 192 ledige nivåer = 160 ms budsjett; tapt flanke gir 160 av 256.
//
// Ved 960 Hz var 128 direkte ugyldig: 128 + 80 ms * 2160 ord/s = 301 ord i en FIFO som
// rommer 256. Én tapt flanke garanterte tap. Assert-en under er skrevet om for å fange
// nettopp det - den gamle så bort fra det stående nivået og slapp det gjennom.
//
// Prisen er halvert dreneringsintervall (53 ms ved 480 Hz) og dobbelt så mange
// sync-poster i råloggen: 17 byte per drenering, 1.9 % -> 3.8 % av filstørrelsen.
static constexpr uint16_t kFifoWatermark = 128;
static_assert(kFifoWatermark > 0 && kFifoWatermark < 256,
              "FIFO_CTRL1.WTM is 8 bits - a larger watermark would be truncated");

// Den øvre grensen på det stående nivået, og fra 2026-08-16 den ENESTE grensen som
// gjelder i den aktive stien. Den sto tidligere nede ved kMaxDrainIntervalMs og var
// utledet av hva en tapt frist koster; med INT1 og ingen frist i porten finnes ikke
// den størrelsen lenger, og en utledning som later som den gjør er verre enn ingen.
//
// Derfor en ren brøk av dybden i stedet. Den sier det eneste som fortsatt er sant på
// kompileringstidspunktet: vaktmerket er det stående nivået, og det som blir igjen -
// kFifoDepthWords minus kFifoWatermark - er alt dreneringen har på seg. Brøken er et
// VALG om hvor mye av bufferet som skal være reserve, ikke en måling.
//
// 80 % gir 204 som tak. Ved dagens 128 står 128 ord igjen, altså 107 ms ved 1200 ord/s.
// Til sammenlikning er Welch-FFT-en 88 ms av dem og en sd-stall er målt til 516.
// Kompilatoren kan ikke se noen av de to tallene - de finnes bare som tim_welch_us_max
// og tim_flush_us_max i ses.csv, og det er der denne grensen faktisk kontrolleres.
static constexpr uint16_t kFifoWatermarkMaxPct = 80;
static_assert(100u * (uint32_t)kFifoWatermark
                  < kFifoWatermarkMaxPct * (uint32_t)kFifoDepthWords,
              "the watermark is the STANDING level - leaving under 20% of the FIFO free "
              "gives the drain no room for an sd-stall or the Welch FFT to land in");

// INT1_CTRL (0x0D): which events the sensor drives out on the INT1 pin. Raw register
// values because the Arduino wrapper exposes no setter for them, only Write_Reg.
static constexpr uint8_t kInt1CtrlReg = 0x0D;
static constexpr uint8_t kInt1FifoTh  = 0x08;  // bit 3: FIFO watermark reached

// Deadline after which update() drains whether or not its trigger fired. The trigger is
// a hint about WHEN to drain and never about whether: a single lost edge would otherwise
// stop the capture permanently - measured 2026-08-04, 0 Hz for the rest of the session.
//
// Den grensen er den samme i begge modi, og det er bare UTLØSEREN som skiller dem: en
// INT1-flanke når kImuUseInt1 er satt, FIFO-nivået i statuslesingen når den ikke er det.
// Derfor én konstant, ikke to.
//
// kMaxDrainIntervalMs utledes av ordraten, og ordraten trenger kSflpOdrHz, som
// deklareres med fusjonsinnstillingene lenger ned. Konstanten, utledningen og begge
// grensene står derfor samlet der. Søk opp kMaxDrainIntervalMs.

// FIFO_STATUS1/FIFO_STATUS2. Read as a 2-byte burst rather than through the wrapper:
// lsm6dsv16x_fifo_status_get drops FIFO_OVR_LATCHED, and that bit is reset by the very
// read that drops it, so going through the wrapper makes it permanently unobservable.
static constexpr uint8_t kFifoStatus1Reg = 0x1B;

// FIFO_DATA_OUT_TAG. The six payload registers follow it (0x79..0x7E), and reading
// 0x7E is what pops the word - so a 7-byte auto-incrementing burst from here takes
// tag and payload out together, which is what makes the pairing atomic.
static constexpr uint8_t kFifoDataOutTagReg = 0x78;

// -----------------------------------------------------------------------------
// BLOKKLESING: hvor mange FIFO-ord som hentes i én SPI-transaksjon
// -----------------------------------------------------------------------------
// Fram til 2026-08-16 leste pop-løkka ETT ord per transaksjon, målt til 28 us. Bare
// ~11 av dem er buss - 8 byte ved 6 MHz, som er der spi_init faktisk lander (se
// imuBurstRead). De øvrige ~17 er transaksjonsoverhead: beginTransaction, to
// digitalWrite på CS, adressebyten som entrer spi_transfer for seg, endTransaction.
// Den overheaden betales én gang per BURST i stedet for én gang per ord.
//
//   32 ord -> 17/32 = 0.5 us overhead per ord, altså ~9.8 us mot 28. Gulvet er 9.3
//   (ren busstid for 7 byte ved 6 MHz), så 32 henter nesten hele gevinsten. 64 ville
//   gitt 9.6 - 0.2 us per ord for dobbelt så mye RAM.
//
// FORUTSETNINGEN: at adressen ruller fra 0x7E tilbake til 0x78, slik at én
// sammenhengende lesing gir PÅFØLGENDE FIFO-ord og ikke det samme ordet om igjen.
//
// VERIFISERT PÅ MASKINVARE 2026-08-16, ikke bare mot databladet. En midlertidig selvtest
// talte to ting over en hel capture og fikk 0 på begge: ord med tag_sensor utenfor
// {1, 2, 0x13} etter posisjon 0 i en burst (som ville fanget at adressen stopper på 0x7E
// og gjentar en databyte), og burst der alle ord var byte-identiske med det første (som
// ville fanget en rullering uten at FIFO-en avanserer). Testen er fjernet igjen; dette
// avsnittet er det som er igjen av den, og det er grunn nok til å ikke gjenta øvelsen.
//
// Sett kFifoBurstWords = 1 for å gå tilbake til ett ord per transaksjon. Koden håndterer
// det uten andre endringer, og det er den riktige første testen hvis noen senere mistenker
// FIFO-lesingen for å levere feil data.
static constexpr uint16_t kFifoBurstWords = 32;
static_assert(kFifoBurstWords > 0 && kFifoBurstWords <= kFifoDepthWords,
              "a burst cannot be empty, nor larger than the FIFO it reads from");

static constexpr uint16_t kAccelOdrHz = kImuOdrHz;

// -----------------------------------------------------------------------------
// THE RATE CHAIN. Every rate is named for what CONSUMES it, and carries no value in
// its name - kVacc10HzBucketMs stops being true the moment someone edits it.
//
//   kImuOdrHz         raw accel/gyro out of the FIFO
//     -> kAhrsInputOdrHz   the orientation filter (= kImuOdrHz, undivided)
//     -> kRowOdrHz         FIR stage 1; the rows written to imu.csv
//     -> kWelchInputOdrHz  FIR stage 2; the series fed to Welch
//
// Each stage states its ODR and DERIVES its period. The static_asserts by the FIR
// tables make an illegal decimation a build error, not a slow time-base drift.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Row rate: the output of FIR stage 1 and the input to stage 2, so every row feeds
// the wave analysis whether or not imu.csv is written. The VALUE is chosen for the
// sd-card, not the analysis.
// The wave chain is unaffected by the choice, since stage 2 decimates to
// kWelchInputOdrHz regardless. It caps offline reanalysis in WaveLogMode::Csv -
// while WaveLogMode::Raw log every FIFO word at kImuOdrHz.
// Notice that if LPF2 is used, then at kLpf2CutoffHz is a ceiling for postprocessing
// 
// -----------------------------------------------------------------------------

static constexpr uint16_t kRowOdrHz    = 100;
static constexpr uint16_t kRowPeriodMs = 1000 / kRowOdrHz;
static_assert(1000 % kRowOdrHz == 0,
              "kRowOdrHz must divide 1000 - the row grid is kept in whole ms");

// IMU debug: how often update() prints the effective accel/gyro sample rate + mean
// magnitudes (ms) to the serial monitor. Only emitted when debug_serial is set.
// 0 disables the printout.
// The line also carries the time left of the capture, so one interval governs both
// "is the drain healthy" and "is this thing still counting down".
static constexpr uint32_t imu_debug_print_period = {20*s_2_ms};  // 20 s

// -----------------------------------------------------------------------------
// Tidsmåling av den varme stien (wave_timing.h). Et diagnoseverktøy, ikke en driftslogg:
// slå den av igjen før en utsetting som skal vare, slik at målingen ikke er med i de
// tallene den selv skulle forklare.
//
// Hva den svarer på: budsjettet over sier at en fangst mister ord først når det går mer
// enn kFifoDepthWords / kFifoWordsPerSec uten en drenering. Råloggen viser AT slike
// opphold finnes - sync-posten stempler hver drenering med millis - men ikke HVA tiden
// gikk med. Bøttene i wave_timing.h måler hver del av løkka mot det samme budsjettet:
// SPI per ord, Madgwick per sample, FIR per rad, råloggens sd-skriving, den periodiske
// sync-en, GPS-raden, og hele iterasjonen, så en stall kan tilskrives noe konkret.
//
// PRISEN, og den skal leses før tallene brukes: målingen koster to micros()-kall per
// FIFO-ord og to per accel-sample, altså 300-400 kall på en drenering på ~128 ord.
// micros() er en SysTick-avlesning på ~0.5-1 us, så det er ~0.3 ms lagt til en drenering
// på ~5 ms - rundt 6 %. TIM_SPI og TIM_AHRS er derfor systematisk litt for høye, og
// TIM_POP bærer overheaden fra begge. Godt nok til å finne en stall på hundrevis av ms;
// ikke godt nok til å sitere 18 us/ord som en eksakt verdi.
//
// Rapporteres to steder: en [TIM]-linje på seriemonitoren hver imu_debug_print_period,
// og en aggregatblokk i ses.csv ved capture-slutt (den overlever en feltkjøring uten
// seriemonitor). ses.csv-blokka skrives fra processReading, så en avbrutt fangst får
// ingen - samme forbehold som resten av sammendraget.
//
// Begge steder rapporteres n, MIDDEL og MAKS. Maks er den som betyr noe når spørsmålet er
// overrun: en stall er per definisjon en haleverdi, og et middel over tusenvis av
// dreneringer deler en utligger på 300 ms ned i støyen. Middelet sier hva løkka normalt
// koster, maks sier om noe i den kan ha tømt budsjettet.
//
// Målt kostnad ved å slå den AV igjen (orb_drifter, 2026-08-15): flash faller tilbake til
// utgangspunktet, mens de ~312 byte som wave_timing-globalen legger i .bss blir stående -
// en global med ekstern linkage fjernes ikke selv om ingen leser den. Mot 12 kB ledig RAM
// er det uvesentlig, men det er ikke null, og det er verdt å vite før noen leter etter
// forskjellen.
// -----------------------------------------------------------------------------
static constexpr bool wave_timing_enabled {true};

// Raw IMU log: the 6-byte payload of every FIFO word verbatim, where imu.csv is the
// FIR-decimated record. Not decoded on the way out - the sensitivities are in the
// header, and this runs inside the drain. No per-word timestamp: word order is the time
// axis, and one sync record per drain pins it to the clock.
//
// LAYOUT, little-endian:
//   header  kRawHeaderBytes, once at the top
//   word    1 B tag_sensor + 6 B payload                          = kRawWordBytes
//   sync    1 B kRawSyncTag + u32 t_us + u32 accel_n + u32 millis
//                           + u16 n_words + u16 flags             = kRawSyncBytes
// tag_sensor is 5 bits, so it can never collide with kRawSyncTag.

// Which IMU log a capture writes - independent files, not two formats of one thing.
//   Csv  - 15.7 kB/s, what the offline chain reads today.
//   Raw  - 8.6 kB/s, needs raw_to_csv.py before the usual tools work.
//   Both - the only mode where the reconstruction can be checked against imu.csv.
//          Madgwick 480 + SFLP 240 + Both overflows.

enum class WaveLogMode : uint8_t { Csv = 0, Raw = 1, Both = 2 };
static constexpr WaveLogMode wave_log_mode = WaveLogMode::Raw;

static constexpr bool wave_mode_imu_csv(void) {
  return wave_log_mode == WaveLogMode::Csv || wave_log_mode == WaveLogMode::Both;
}
static constexpr bool wave_mode_imu_raw(void) {
  return wave_log_mode == WaveLogMode::Raw || wave_log_mode == WaveLogMode::Both;
}
static constexpr uint32_t kRawMagic          = 0x4257524FUL;  // "ORWB" little-endian
static constexpr uint8_t  kRawFormatVersion  = 2;
static constexpr uint8_t  kRawSyncTag        = 0xFF;
static constexpr uint8_t  kRawWordBytes      = 7;
static constexpr uint8_t  kRawSyncBytes      = 17;
static constexpr uint8_t  kRawHeaderBytes    = 32;
static constexpr uint16_t kRawBlockBytes     = 512;   // SD block; buffered, not per word

// Råbufferet må romme en HEL drenering, ikke bare en SD-blokk.
//
// Skrivingen lå før inne i pop-løkka: bufferet fyltes opp midt i tømmingen av FIFO-en
// og ble skrevet der og da. Stanset kortet i det øyeblikket, sto det fortsatt uhentede
// ord igjen i FIFO-en, og halve overskrivningsbudsjettet var allerede brukt før klokka
// begynte å telle. Nå hentes alle ordene først og skrives etterpå, så en stall treffer
// en TOM FIFO: 256 ledige nivåer i stedet for ~128, altså 213 ms i stedet for 107 ved
// 480 Hz.
//
// Om årsaken, siden den ble feildiagnostisert her først: analysen 2026-08-14 tolket de
// periodiske tapene (~446 s fra hverandre) som at kortet stanset 700-900 ms av seg selv,
// og konkluderte med at 213 ms uansett ikke rakk. Det var for kategorisk. Etter at
// vaktmerket gikk fra 128 til 64 og fristen ble utledet av ordraten, falt tapene
// dramatisk - fra 24-42 per økt ved WTM 128 til 1 på ~20 min, målt 2026-08-15. Marginen
// var altså hovedsaken: ved WTM 128 sto FIFO-en 224 av 256 full etter en tapt flanke, og
// da holder en vanlig blokkskriving.
//
// Men de forsvant ikke HELT, slik denne kommentaren hevdet fram til 2026-08-15, og det
// gjenstående tapet kan ikke forklares av en tapt flanke: fristen på 106 ms etterlater
// 128 av 256 nivåer, så en overrun krever over 213 ms uten drenering. Noe bruker den
// tiden. wave_timing_enabled er lagt inn for å finne ut hva - ikke fordi konklusjonen
// over var feil, men fordi den var basert på fravær av tap og nå har et moteksempel.
// Verste drenering: en full FIFO tømt i én runde, pluss dens sync-post. Ikke et
// teoretisk tilfelle - det er nettopp dreneringen ETTER en sd-stall, når FIFO-en har
// bygd seg opp mens kortet var opptatt. Bufferet må tåle akkurat den runden.
static constexpr uint16_t kRawWorstDrainBytes =
    (uint16_t)kFifoDepthWords * kRawWordBytes + kRawSyncBytes;   // 256*7 + 17 = 1809

// -----------------------------------------------------------------------------
// SKRIVEGRENSE: hvor mye som samles opp før råloggen skrives til kortet
// -----------------------------------------------------------------------------
// Fram til 2026-08-15 skrev flushRaw() etter HVER drenering. Med vaktmerke 64 er det
// 64*7 + 17 = 465 B, ca. 18 ganger i sekundet - alle under en sektor og ingen på en
// sektorgrense. To ting fulgte av det:
//
//   * Hver skriving gikk gjennom SdFats ene 512-bytes datacache med memcpy og
//     read-modify-write. En skriving som dekker hele sektorer tar SdFats direkte vei
//     i stedet og skyver sektorene rett fra rawBuf_ til kortet, uten cache. Det er
//     også slutten på at råloggen slåss med gps.csv om den samme cache-blokken.
//   * Én sektor per kommando. Justerte multisektor-skrivinger blir én CMD25-sekvens,
//     og busy-fasen mellom blokker inne i en slik sekvens er kortere enn en full
//     commit per blokk.
//
// Det dette IKKE gjør: færre blokkskrivinger. 8 kB/s er 16 sektorer i sekundet
// uansett, og kortets garbage collection er proporsjonal med data skrevet. Utliggerne
// på ~516 ms blir like sannsynlige. Gevinsten ligger i middelverdien, ikke i halen -
// overrunene trenger fortsatt FIFO-margin eller et annet kort.
//
// Hvorfor 2 sektorer og ikke 4 eller 8: hovedgevinsten (utenom cachen) inntreffer
// allerede ved én hel sektor. Fra 2 til 4 amortiseres bare kommando-overhead videre,
// mens bufferet - og dermed RAM - vokser lineært. Med 64 kB på WLE5-en er 2 sektorer
// mesteparten av gevinsten til halve kostnaden.
static constexpr uint16_t kRawFlushThreshold = 2 * kRawBlockBytes;   // 1024

// rawBuf_ må romme resten som kan ligge igjen når en drenering starter, pluss én
// verste drenering oppå den. Resten er høyst kRawFlushThreshold - 1: enten skrev
// flushRaw() og etterlot mindre enn en sektor, eller så nådde den ikke grensa og lot
// alt ligge. 1023 + 1809 = 2832.
//
// IKKE rundet opp til hele sektorer - det er skrivelengden som må være sektorjustert,
// ikke bufferet, og en avrunding hit ville bare vært 240 B RAM uten funksjon. Utledet
// og ikke skrevet som tall, så de tre konstantene ikke kan komme i utakt.
static constexpr uint16_t kRawBufBytes =
    kRawFlushThreshold - 1 + kRawWorstDrainBytes;   // 1024 - 1 + 1809 = 2832

static_assert(kRawSyncTag > 0x1F,
              "the sync tag must not collide with a FIFO tag_sensor (top 5 bits)");
static_assert(kRawBlockBytes >= kRawSyncBytes + kRawWordBytes,
              "raw block must hold at least a sync record plus one word");
static_assert(kRawFlushThreshold % kRawBlockBytes == 0 && kRawFlushThreshold > 0,
              "the flush threshold must be a whole number of sectors, or the write is "
              "never sector aligned and SdFat falls back to its cache - which is the "
              "entire point of having a threshold");
static_assert(kRawBufBytes >= (uint32_t)kRawFlushThreshold - 1 + kRawWorstDrainBytes,
              "rawBuf_ must hold the sub-threshold remainder PLUS one full drain - a "
              "flush inside the pop loop is the very thing this buffer exists to avoid");

// Sync-record flag bits; FifoOvf means the sensor
// overwrote words - the file is intact, a stretch of time is missing. WriteFail means
// this file lost bytes, which desynchronises every byte after it, so it has to be
// recorded IN the stream: the damage is positional.

static constexpr uint16_t kRawFlagFifoOvf   = 0x0001;
static constexpr uint16_t kRawFlagWriteFail = 0x0002;

// Bench test: enqueue a fabricated WaveResult instead of running a capture, so the
// serialise -> LoRa -> SD path can be exercised at once. Only the source is faked.

#ifndef DEBUG_WAVE_MSG
#define DEBUG_WAVE_MSG 0
#endif

// -----------------------------------------------------------------------------
// Braking wave detection (linear |a| over threshold long enough within a window).
// TODO - find some good values from literature
// -----------------------------------------------------------------------------
static constexpr float  kBrakeGThreshold = 0.5f;
static constexpr double kBrakeThresholdMg2 =
    (double)(kBrakeGThreshold * 1000.0) * (kBrakeGThreshold * 1000.0);
static constexpr uint16_t kBrakeMinMs = 5;
static constexpr uint16_t kBrakeMinSamples =
    ((kBrakeMinMs * kAccelOdrHz + 999) / 1000) < 1 ? 1
    : (uint16_t)((kBrakeMinMs * kAccelOdrHz + 999) / 1000);

// -----------------------------------------------------------------------------
// Analysis: Madgwick 6-axis AHRS -> vertical acceleration.
// -----------------------------------------------------------------------------
static constexpr float    kMadgwickBeta      = 0.05f;
static constexpr float    kGravity           = 9.80665f;
// Welch input rate: the end of the chain. ODR is what is SET here; the period is
// derived, so the two cannot drift apart the way kVacc10HzBucketMs/kVaccFsHz could.
static constexpr uint16_t kWelchInputOdrHz    = 10;
static constexpr uint16_t kWelchInputPeriodMs = 1000 / kWelchInputOdrHz;
static_assert(1000 % kWelchInputOdrHz == 0,
              "kWelchInputOdrHz must divide 1000 - the bucket grid is kept in whole ms");
static constexpr float    kMg2Ms2            = kGravity / 1000.0f;            // mg -> m/s^2
static constexpr float    kMdps2Rads         = 1.0e-3f * (float)M_PI / 180.0f; // mdps -> rad/s

// The AHRS runs on every raw sample, straight off the FIFO - no divider and no FIR
// ahead of it, so the gyro is integrated at the rate it was measured at and the
// quaternion delay is a plain sample count. kAhrsInputOdrCapHz is the ceiling where the
// filter stops fitting the CPU budget, set for Madgwick; KalmanAhrs is ~20x dearer.

// TODO: find good values for KalmanAhrs and make the ceiling a function of the filter type.

static constexpr uint16_t kAhrsInputOdrCapHz = 960;
static constexpr float    kAhrsInputOdrHz    = (float)kImuOdrHz;
static_assert(kImuOdrHz <= kAhrsInputOdrCapHz,
              "the AHRS runs on every raw sample - kImuOdrHz above kAhrsInputOdrCapHz does "
              "not fit the CPU budget; lower the ODR rather than re-introducing a divider");

// FIR decimation, two stages (example):
//   stage 1: kImuOdrHz -> kRowOdrHz         960 -> 100, D = 9.6   (imu.csv columns)
//   stage 2: kRowOdrHz -> kWelchInputOdrHz  100 -> 10,  D = 10    (the Welch series)
// Taps come from fir_coeffs.h, generated from firwin_lowpass() in tools/gen_fir_table.py
//
// Stage 1 is generally not an integer decimation, so it runs on the time-driven
// kRowPeriodMs grid, evaluated at the raw sample nearest the centre - at most half a
// raw period of jitter. Stage 2 must be exact, or the bucket centres slide against the
// row grid; asserted below, where the offline mirror only warns.
//
// Both stages use cutoff = fs_out/2, so the taps depend on D alone, as 1/(2*D) - two
// stage-1 setups with the same ratio share a table. The key is 10*D to keep a factor
// like 9.6 an exact integer comparison; no table selects nullptr and the assert fires.

static constexpr float    kFirS1CutoffHz  = 0.5f * kRowOdrHz;   // fs_out/2, fir.py's convention
static constexpr float    kFirS2CutoffHz  = 0.5f * kWelchInputOdrHz;
static constexpr uint16_t kFirS2Decim     = kWelchInputPeriodMs / kRowPeriodMs;  // rows per bucket
static constexpr uint16_t kFirS2CenterMs  = (kFirS2Decim / 2) * kRowPeriodMs;  // = fir.py's dec//2
static constexpr uint16_t kFirS1CenterMs  = kRowPeriodMs / 2;                  // same convention
static constexpr float    kFirS1DelayS    = (float)kFirHalf / (float)kImuOdrHz;
static constexpr float    kFirS2DelayS    = (float)kFirHalf / (float)kRowOdrHz;

static constexpr uint16_t kFirS1DecimX10 = (uint16_t)((10u * kImuOdrHz) / kRowOdrHz);
static constexpr uint16_t kFirS2DecimX10 = (uint16_t)(10u * kFirS2Decim);
static constexpr const float *kFirCoeffsStage1 = firTapsForDecimX10(kFirS1DecimX10);
static constexpr const float *kFirCoeffsStage2 = firTapsForDecimX10(kFirS2DecimX10);

static_assert(10u * kImuOdrHz % kRowOdrHz == 0,
              "kImuOdrHz/kRowOdrHz must land on a whole tenth - the table key is 10*D");
static_assert(kFirCoeffsStage1 != nullptr,
              "no FIR table for kImuOdrHz/kRowOdrHz - add the ratio to DEFAULT_DECIM "
              "in tools/gen_fir_table.py and regenerate fir_coeffs.h");
static_assert(kFirCoeffsStage2 != nullptr,
              "no FIR table for kRowOdrHz/kWelchInputOdrHz - add the ratio to DEFAULT_DECIM "
              "in tools/gen_fir_table.py and regenerate fir_coeffs.h");

static_assert(kFirNtap % 2 == 1,
              "odd tap count - the group delay must be a whole number of samples");
static_assert(kWelchInputPeriodMs % kRowPeriodMs == 0,
              "stage 2 must decimate a whole number of rows - fir.py assumes the same");
static_assert(kRowOdrHz <= kImuOdrHz,
              "the row rate is a DECIMATION of the raw stream - it cannot exceed kImuOdrHz");
static_assert(kWelchInputOdrHz <= kRowOdrHz,
              "the Welch input is a DECIMATION of the rows - it cannot exceed kRowOdrHz");
static_assert(wave_measurement_filter_warm_up > (uint32_t)kFirNtap * kRowPeriodMs + 2000,
              "warm-up must cover FIR start-up (1.29 s) plus AHRS convergence");

// ---- Orientation delay ----
// The quaternions are NOT filtered: attitude is already the output of a heavy
// low-pass (Madgwick's accel correction has a multi-second time constant, and the
// gyro feeding it is band-limited by the sea), so there is nothing to alias, and a
// linear FIR over quaternion components neither preserves |q| = 1 nor survives the
// +-q sign ambiguity. What they DO need is the same delay: the FIR ahead of ax..gz
// is causal, so those columns describe the signal kFirHalf raw samples earlier than
// the row's timestamp. Holding a plain latest-quaternion would make the row describe
// two different instants - ~4 deg of error at 1 Hz / 10 deg tilt. QuatDelay carries
// the attitude the same distance back; see quat_delay.h.
// The AHRS steps once per raw sample, so the delay is the FIR group delay itself -
// no conversion, nothing that has to divide evenly, and zero residual phase error.
static constexpr uint16_t kQuatDelaySteps = kFirHalf;              // 64 raw samples
static constexpr uint16_t kQuatDelaySlots = kQuatDelaySteps + 1;   // 65 (2080 B)

// ---- Welch spectrum -> wave parameters ----
// On the 10 Hz vertical-acceleration series a 1024-sample segment gives
// df = 10/1024 = 0.009766 Hz and a 102.4 s segment, so a 30 min capture averages 67
// segments at 75% overlap. 2048 resolved finer but halved the segment count (32),
// leaving a noisier PSD, and cost ~14 kB more RAM (segBuf_ + psdAcc_ + the FFT
// scratch buffers) - which matters here. A fetch-limited fjord peaks at 0.15-0.6 Hz;
// 1024 places ~31 bins across a 0.3 Hz peak. However, welch_bins in common_config.h is 
// the wire-format capacity, so the transmitted spectrum is coarser than the analysis
static constexpr uint16_t kWelchSegLen     = 1024;
static constexpr uint16_t kWelchOverlapDiv = 4;      // step = seglen/4 => 75% overlap
static_assert((kWelchSegLen & (kWelchSegLen - 1)) == 0, "kWelchSegLen must be a power of two");

// -----------------------------------------------------------------------------
// RINGSLAKK: hvorfor segmentbufferet er 8 samples større enn et segment
// -----------------------------------------------------------------------------
// accumSegment() er det lengste uavbrutte strekket i capture-løkka - 88 ms målt
// 2026-08-15, se kommentaren over funksjonen i wave_analysis.cpp. Fram til da kjørte
// den inne i FIFO-pop-løkka, altså med opptil kFifoWatermark - 1 ord fortsatt i
// FIFO-en, og hadde bare (kFifoDepthWords - kFifoWatermark) / kFifoWordsPerSec =
// 165 ms å gjøre seg ferdig på. Nå er den utsatt til capture-løkka, der dreneringen
// er ferdig og hele dybden står til rådighet: 220 ms. Samme arbeid, 55 ms mer margin.
//
// Utsettelsen krever at segmentet ikke overskrives i mellomtiden, og DET er alt
// slakken er til for. Ikke et helt steg på 256 samples - bare de samplene som rekker
// å ankomme mellom at segmentet blir fullt (inne i pop-løkka) og at det utsatte
// kallet kjører (rett etter at samme update() har returnert). Vinduet er altså resten
// av én drenering, og verste drenering er en full FIFO:
//
//   kFifoDepthWords / kFifoWordsPerSec * kWelchInputOdrHz = 256/1166 * 10 = 2.2 samples
//
// 8 er den med margin. Kostnaden er 8 floats = 32 B, mot 4096 B for en full kopi av
// segmentet - som er grunnen til at det ble en ringbuffer og ikke en dobbeltbuffer.
static constexpr uint16_t kWelchRingSlack = 8;
static constexpr uint16_t kWelchRingLen   = kWelchSegLen + kWelchRingSlack;   // 1032
// Selve grensen kan ikke sjekkes her: den trenger kFifoWordsPerSec, som utledes langt
// nede i FIFO-seksjonen. static_assert-en står derfor der, ved den konstanten.

static constexpr float kPsdDfHz = (float)kWelchInputOdrHz / kWelchSegLen;  // 0.009766 Hz per bin

// kWaveFMax, the upper edge of the analysed band, is defined up by the LPF2 block -
// the divisor is derived from it, so it has to come first.

// Low-frequency half-cosine taper (Kohout / Tucker & Pitt 2001) on the elevation PSD.
static constexpr float kTaperF1 = 0.03f;   // T=0 below
static constexpr float kTaperF2 = 0.05f;   // T=1 above
static_assert(kTaperF1 < kTaperF2 && kTaperF2 <= kWaveFMax, "need kTaperF1 < kTaperF2 <= kWaveFMax");

// ---- Transmitted spectrum slice ----
// Each wire bin is the band average of kSpecBinGroup PSD bins, so the moments and Tp
// keep the full kPsdDfHz while the message spans kPsdMinFreq..kPsdMaxFreq.
// Averaging, so the energy is preserved

static constexpr bool kSendPsd = true; // To send PSD or not
static constexpr float kPsdMinFreq = 0.03f;
static constexpr float kPsdMaxFreq = 1.0f;
static_assert(kPsdMinFreq < kPsdMaxFreq, "the transmitted band would be empty");
static_assert(kPsdMaxFreq <= kWaveFMax,
              "the transmitted spectrum would reach past the analysed band and be "
              "normalised against a peak that never saw it - raise kWaveFMax or lower "
              "kPsdMaxFreq");

// PSD bins spanned by kPsdMaxFreq. Truncated rather than rounded: 1.0 Hz is 102.4
// bins, and bin 102 (0.9961 Hz) is the last one that still fits inside the request.
// Everything below floors as well, so the transmitted band never exceeds kPsdMaxFreq.
static constexpr size_t kPsdMaxBin = (size_t)(kPsdMaxFreq / kPsdDfHz);

static constexpr size_t kPsdMinBinFloor = (size_t)(kPsdMinFreq / kPsdDfHz); // First PSD bin whose lower edge clears kPsdMinFreq
static constexpr size_t kPsdMinBin =
    kPsdMinBinFloor + ((float)kPsdMinBinFloor * kPsdDfHz < kPsdMinFreq ? 1u : 0u);

static constexpr size_t welch_bin_min {kPsdMinBin};

static_assert(kPsdMaxBin > welch_bin_min,
              "kPsdMinFreq and kPsdMaxFreq are inside the same PSD bin - widen the "
              "band, or raise kWelchSegLen so the bins get finer");

// welch_bins (common_config.h) is the the size of wave_spectrum[] 
// and max wave_message_size budgets, not the actual count (num_bins)
// num_bins rides in the message, and the spectrum is the last field, so
// shipping fewer bins simply makes the message shorter (see readings.h).
//
// kSpecBinGroup is chosen as the smallest that squeezes welch_bin_min..kPsdMaxBin into
// the capacity, so the transmitted resolution is the finest the budget allows
// The count is then however many whole groups fit inside kPsdMaxBin,
// but limited to corresponding kPsdMaxFreq
static constexpr size_t kSpecBinGroup =
    (kPsdMaxBin - welch_bin_min + welch_bins - 1) / welch_bins;
static constexpr size_t kSpecNBins     = (kPsdMaxBin - welch_bin_min) / kSpecBinGroup;
static constexpr size_t welch_bin_max  = welch_bin_min + kSpecNBins * kSpecBinGroup;
static constexpr float  kSpecBandMinHz = welch_bin_min * kPsdDfHz;
static constexpr float  kSpecBandMaxHz = welch_bin_max * kPsdDfHz;

static_assert(kSpecBinGroup >= 1,
              "kPsdMaxFreq is below a single PSD bin - raise it, or raise kWelchSegLen "
              "so the bins get finer");
static_assert(kSpecNBins >= 1, "empty transmitted bin range");
static_assert(kSpecNBins <= welch_bins,
              "more transmitted bins than wave_spectrum[] holds - welch_bins in "
              "common_config.h is the wire-format capacity and the base station sizes "
              "its buffer from it, so it cannot be raised on this side alone");
static_assert(welch_bin_max <= kWelchSegLen / 2 + 1,
              "transmitted bins must fit inside the one-sided PSD");
static_assert(kSpecBandMaxHz <= kPsdMaxFreq,
              "flooring is what keeps the transmitted band inside kPsdMaxFreq - this "
              "cannot fire unless the derivation above was changed");
static_assert(kSpecBandMinHz >= kPsdMinFreq,
              "rounding up is what keeps the transmitted band above kPsdMinFreq - this "
              "cannot fire unless the derivation above was changed");

// Frequency axis of the transmitted spectrum, as the receiver must reconstruct it:
//   f_j = kSpecFMinHz + j * kSpecBinWidthHz,  j = 0 .. kSpecNBins-1
static constexpr float kSpecBinWidthHz = kSpecBinGroup * kPsdDfHz;          // 0.019531 Hz
static constexpr float kSpecFMinHz     = (welch_bin_min + 0.5f * (kSpecBinGroup - 1)) * kPsdDfHz;
static constexpr float kSpecFMaxHz     = kSpecFMinHz + (kSpecNBins - 1) * kSpecBinWidthHz;

static constexpr size_t kSpecTxBins = kSendPsd ? kSpecNBins : 0; // Number of sent PSD bins

enum class WindowType { Hann, Hamming };
static constexpr WindowType kWelchWindow = WindowType::Hann;

// -----------------------------------------------------------------------------
// Kalman: quaternion error-state EKF with an adaptive measurement noise; see kalman.h
// for what R's three terms do
// -----------------------------------------------------------------------------
static constexpr KalmanAhrsParams kKalmanParams = {
    /* sigmaG  */ 0.005f,        // rad/s/sqrt(Hz), ~0.3 deg/s/sqrt(Hz)
    /* sigmaB  */ 1.0e-5f,       // rad/s^2/sqrt(Hz)
    /* r0      */ 1.0e-3f,       // was 1e-5 - see the 2026-08-04 sweep above
    /* dtRef   */ 0.020f,        // s - the rate the ORIGINAL sweep above was run at
    /* lambdaA */ 0.0f,
    /* lambdaW */ 2.0f,
    /* w0      */ 1.0f,          // rad/s
    /* gravity */ kGravity,
    /* p0Angle */ 5.0f * (float)M_PI / 180.0f,    // 5 deg
    /* p0Bias  */ 1.0f * (float)M_PI / 180.0f,    // 1 deg/s
};

// -----------------------------------------------------------------------------
// THE orientation selection, at compile time: only the selected filter is linked and
// only its state occupies RAM. The two share an API, so makeWaveAhrs() - which exists
// because the constructors differ - is the only thing that has to change with it.
// -----------------------------------------------------------------------------

// Use chip built in AHRS (SFLP fusion) instead of the AHRS above. 
static constexpr bool wave_use_sflp = false;

// If not use built-in AHRS, select the software AHRS to run
using WaveAhrs = Madgwick;
inline WaveAhrs makeWaveAhrs(void) { return WaveAhrs{kMadgwickBeta}; }


// Built in SFLP settings. The SFLP fusion is a 6-axis AHRS that runs on the chip, and outputs a quaternion at a fixed rate. 
static constexpr bool kEnableSflp = true;                  // If we are to enable the SFLP fusion or not.
static constexpr float   kSflpOdrHz = 240.0f;              // on-chip fusion rate
static constexpr uint8_t kSflpRotationTag = 0x13;          // FIFO tag: SFLP rotation vector

// Words the FIFO takes in a second: accel and gyro at kImuOdrHz, plus the rotation
// vector at the fusion's own rate when it is batched. This is the denominator behind
// every timing claim in the INT1 section above.

static constexpr uint32_t kFifoWordsPerSec =
    2u * (uint32_t)kImuOdrHz + (kEnableSflp ? (uint32_t)kSflpOdrHz : 0u);

// Deferred from the Welch section: kWelchRingSlack needs kFifoWordsPerSec above.
//
// Slakken må dekke samplene som ankommer mens et fullt segment venter på det utsatte
// accumSegment-kallet, og det vinduet er verste drenering: en full FIFO. Brøken er
// rundet OPP, siden en halv sample fortsatt krever en hel slot. Denne kan ryke om noen
// senker ODR-en (færre ord/s => lengre drenering) eller hever kWelchInputOdrHz.
static_assert(kWelchRingSlack >= ((uint32_t)kFifoDepthWords * kWelchInputOdrHz
                                  + kFifoWordsPerSec - 1u) / kFifoWordsPerSec,
              "kWelchRingSlack must cover one worst-case drain's worth of Welch "
              "samples, or a full segment is overwritten before the deferred "
              "accumSegment consumes it - see the ring-slack section above");

// Deferred from the INT1 section: it is kFifoWordsPerSec above that it needs.
//
// En fast verdi i millisekunder klarer ikke to ODR-er: 80 ms lot 480 Hz stå igjen med
// 96 ord etter en tapt utløser, men 960 Hz med 19 - og senket man den til 50 for å
// redde 960, kom den under vaktmerkets egen takt ved 480 og ble den normale utløseren
// i stedet for reserven. Fristen må derfor utledes, ikke settes.
//
// UTLEDET AV HVA, og her endret formen seg 2026-08-15: den var 2*kFifoWatermark, altså
// «tiden det tar å samle to vannmerker til». Det ga begge grensene under per
// konstruksjon, men bandt fristen til BUNKESTØRRELSEN - og det er ikke den fristen
// vokter. Den vokter bufferet. Et vaktmerke er et valg om hvor ofte man vil dreneres;
// dybden er en hard grense på hvor lenge man har råd til å la være. Nå er den en
// brøkdel av tiden FIFO-en bruker på å fylles fra tom, som er den størrelsen den
// faktisk måles mot.
//
// Prisen for det byttet skal stå her: de to static_assert-ene under er ikke lenger
// oppfylt uansett hva - de er ekte grenser som kan ryke om noen endrer vaktmerket eller
// dybden. Den nedre er den skjøre: fristen må bli værende OVER vaktmerkets egen takt,
// ellers slutter den å være en reserve. Ved WTM 64 er det 127 mot 53 ms, altså god
// klaring, men marginen er ikke lenger gratis.
//
// Brøken er 3/5. Taket er 3/4: den øvre assert-en legger et stående vaktmerke oppå
// etterfyllingen, og 64 + 0.8*256 = 268 sprenger et buffer på 256. Det leddet er ren
// margin og ikke en tilstand koden kan havne i - fristen telles fra lastDrainMs_, satt
// når FIFO-en nettopp er tømt, så nivået ved utløp ER etterfyllingen og de to kan aldri
// legges sammen. Marginen får bli stående: fristen er en backstop som i praksis aldri
// skal fyre, så det koster ingenting å ligge lavt.
//
// Navnet var kFifoPollFallbackMs fram til samme dato. Det beskrev bare INT1-modusen -
// «fall tilbake til polling» - mens grensen er den samme i begge, og «Poll» ville
// dessuten stått rett ved siden av en statuslesing som skjer 17x hyppigere enn fristen
// selv. Det den faktisk begrenser er intervallet mellom DRENERINGER, i begge modi.

// Tiden FIFO-en bruker på å gå fra tom til full. Hele tidsbudsjettet i denne fila måles
// mot dette tallet, og ses.csv skriver det ut som tim_fifo_budget_us.
static constexpr uint32_t kFifoFillMs =
    (uint32_t)kFifoDepthWords * 1000u / kFifoWordsPerSec;   // 213 ms @ 480 Hz, 118 @ 960

static constexpr uint32_t kMaxDrainIntervalMs = 3u * kFifoFillMs / 5u;  // 127 ms @ 480 Hz

// Den VALGTE fristen, som andel av den maksimale over. Skillet er verdt de to linjene:
// kMaxDrainIntervalMs er utledet av FIFO-dybden og er en egenskap ved maskinvaren, mens
// dette er et valg om hvor mye av den marginen man vil bruke. Å senke prosenten strammer
// fristen uten å røre brøken over, som er den man må resonnere om på nytt hver gang.
static constexpr uint32_t kDrainIntervalPct = 100;
static constexpr uint32_t kDrainIntervalMs  = kDrainIntervalPct * kMaxDrainIntervalMs / 100u;

// Trivielt sann for enhver prosent under 100, og det er hele poenget: den vokter knappen,
// ikke utledningen. Settes kDrainIntervalPct over 100 er det en frist som er lengre enn
// FIFO-en overlever, og da skal builden stoppe i stedet for å la tallet se lovlig ut.
static_assert(kDrainIntervalMs <= kMaxDrainIntervalMs,
              "the chosen drain deadline exceeds what the FIFO depth allows - "
              "kDrainIntervalPct must not go above 100");

// kMaxDrainIntervalMs FORLATER IKKE DENNE FILA fra 2026-08-16. Den er taket, ikke
// fristen: koden leser kDrainIntervalMs, og det eneste stedet maksverdien opptrer er i
// utledningen av den og i assert-en over. Den skal heller ikke brukes direkte - da er
// prosentknappen omgått.
//
// Selve fristen er samtidig tatt ut av INT1-porten i update(), så kDrainIntervalMs leses
// nå bare av port 2, som kImuUseInt1 kompilerer bort. Se INT1-avsnittet lenger oppe for
// hvorfor og hva det koster. Begge konstantene er beholdt fordi pollingstien trenger dem
// den dagen noen slår den på igjen.
//
// De to static_assert-ene som sto her er FJERNET, ikke deaktivert. De utledet en øvre
// grense på vaktmerket fra hva en tapt frist koster, og uten frist finnes ikke den
// størrelsen - en tapt flanke koster ubegrenset etterfylling, ikke 152 ord. Grensen som
// erstattet dem er en ren brøk av dybden og står ved kFifoWatermark, der den hører
// hjemme: den handler om det stående nivået og trenger verken frist eller ordrate.

// A missed trigger costs kDrainIntervalMs of undrained FIFO on top of whatever was
// already there, and that alone must not fill the buffer - what is left over is the
// margin the sd-card writes have to fit inside.
//
// Lærdommene de fjernede assert-ene bar, bevart fordi de gjelder igjen den dagen
// pollingen slås på:
//   * grensa er kFifoDepthWords (256), ikke 512 - en tidlig utgave godtok dobbelt så
//     mye som bufferet rommer, og ville sluppet gjennom nettopp det den skulle fange.
//   * kFifoWatermark måtte være med i summen. Uten det leddet var 960 Hz med WTM 128
//     lovlig, mens regnestykket er 128 + 173 = 301 ord i en FIFO som rommer 256: én
//     tapt flanke garanterte tap.
//   * fristen har også en NEDRE grense: faller den under vaktmerkets egen takt, slutter
//     den å være en reserve og blir den normale utløseren. Da dreneres det på tid i
//     stedet for på fyllingsgrad, og hver drenering koster en sync-post i råloggen
//     uansett hvor få ord den hentet.

// Just a test that if we are to use SFLP as basis for PSD, then we need to have SFLP enabled.
static_assert(kEnableSflp || !wave_use_sflp,
              "wave_use_sflp feeds the wave chain from the on-chip fusion, and "
              "kEnableSflp has switched that block off - enable it, or select the "
              "software AHRS");

// Choice of filter to config file as text.
static constexpr const char *wave_orientation_name = wave_use_sflp ? "SFLP" : WaveAhrs::kName;

#endif  // WAVE_CONFIG_H
