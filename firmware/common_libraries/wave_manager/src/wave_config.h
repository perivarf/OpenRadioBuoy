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
// hver ~2 ms uansett (delay(2)), så ren polling ville drenert like raskt. Det INT1
// gjør, er å HOLDE IGJEN dreneringen til kFifoWatermark ord har samlet seg. Det er en
// batching-mekanisme, og vaktmerket er batchstørrelsen. Se kFifoWatermark.
//
// false falls back to pure polling
// -----------------------------------------------------------------------------

static constexpr bool kImuUseInt1 = false;

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
static constexpr uint16_t kFifoWatermark = 64;
static_assert(kFifoWatermark > 0 && kFifoWatermark < 256,
              "FIFO_CTRL1.WTM is 8 bits - a larger watermark would be truncated");

// INT1_CTRL (0x0D): which events the sensor drives out on the INT1 pin. Raw register
// values because the Arduino wrapper exposes no setter for them, only Write_Reg.
static constexpr uint8_t kInt1CtrlReg = 0x0D;
static constexpr uint8_t kInt1FifoTh  = 0x08;  // bit 3: FIFO watermark reached

// Deadline after which update() drains whether or not INT1 fired: the interrupt is a
// hint about when to drain and never whether. A single lost edge would otherwise stop
// the capture permanently - measured 2026-08-04, 0 Hz for the rest of the session.
//
// kFifoPollFallbackMs utledes av ordraten, og ordraten trenger kSflpOdrHz, som
// deklareres med fusjonsinnstillingene lenger ned. Konstanten, utledningen og begge
// grensene står derfor samlet der. Søk opp kFifoPollFallbackMs.

// FIFO_STATUS1/FIFO_STATUS2. Read as a 2-byte burst rather than through the wrapper:
// lsm6dsv16x_fifo_status_get drops FIFO_OVR_LATCHED, and that bit is reset by the very
// read that drops it, so going through the wrapper makes it permanently unobservable.
static constexpr uint8_t kFifoStatus1Reg = 0x1B;

// FIFO_DATA_OUT_TAG. The six payload registers follow it (0x79..0x7E), and reading
// 0x7E is what pops the word - so a 7-byte auto-incrementing burst from here takes
// tag and payload out together, which is what makes the pairing atomic.
static constexpr uint8_t kFifoDataOutTagReg = 0x78;

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
static constexpr uint16_t kRawBufBytes =
    (uint16_t)kFifoDepthWords * kRawWordBytes + kRawSyncBytes;   // 256*7 + 17 = 1809

static_assert(kRawSyncTag > 0x1F,
              "the sync tag must not collide with a FIFO tag_sensor (top 5 bits)");
static_assert(kRawBlockBytes >= kRawSyncBytes + kRawWordBytes,
              "raw block must hold at least a sync record plus one word");
static_assert(kRawBufBytes >= (uint32_t)kFifoDepthWords * kRawWordBytes + kRawSyncBytes,
              "rawBuf_ must hold one full drain - a flush inside the pop loop is the "
              "very thing this buffer exists to avoid");

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

// Deferred from the INT1 section: it is kFifoWordsPerSec above that it needs.
//
// Fristen er «tiden det tar å samle to vannmerker til». En fast verdi klarer ikke to
// ODR-er: 80 ms lot 480 Hz stå igjen med 96 ord etter en tapt flanke, men 960 Hz med
// 19 - og senket man den til 50 for å redde 960, kom den under vaktmerkets egen takt
// ved 480 og ble den normale utløseren i stedet for reserven. Utledet av ordraten
// forsvinner begge problemene: etterfyllingen lander på
//
//     kFifoWatermark + 2*kFifoWatermark = 192 av 256 nivåer
//
// UANSETT ODR - 106 ms ved 480 Hz, 59 ms ved 960 - så det som er igjen til sd-kortet er
// alltid en fjerdedel av bufferet. Begge static_assert-ene under er dermed oppfylt per
// konstruksjon, og står igjen som dokumentasjon av grensene framfor som feller.
//
// Merk at formen også setter et tak på vaktmerket: 3*kFifoWatermark må under
// kFifoDepthWords, altså WTM <= 85. Den øvre assert-en er det som håndhever det.


static constexpr uint32_t kFifoPollFallbackMs =
    2u * (uint32_t)kFifoWatermark * 1000u / kFifoWordsPerSec;

// A lost edge costs kFifoPollFallbackMs of undrained FIFO on top of whatever was
// already there, and that alone must not fill the buffer - what is left over is the
// margin the sd-card writes have to fit inside.
//
// De to leddene er begge nødvendige, og det er den lærdommen denne assert-en bærer:
//   * grensa er kFifoDepthWords (256), ikke 512 - forrige utgave godtok dobbelt så mye
//     som bufferet rommer, og ville sluppet gjennom nettopp det den skulle fange.
//   * kFifoWatermark må være med. Uten det leddet var 960 Hz med WTM 128 lovlig, mens
//     regnestykket er 128 + 173 = 301 ord i en FIFO som rommer 256: én tapt flanke
//     garanterte tap. Vaktmerket ER det stående nivået, ikke bare en dreneringstakt.
static_assert(kFifoWatermark + kFifoPollFallbackMs * kFifoWordsPerSec / 1000u
                  < (uint32_t)kFifoDepthWords,
              "watermark + a lost INT1 edge would by itself overrun the FIFO - lower "
              "kFifoWatermark or kFifoPollFallbackMs, or batch fewer words per second");

// kFifoPollFallbackMs har også en NEDRE grense, som ikke er åpenbar: faller den under
// vaktmerkets egen takt, slutter den å være en reserve og blir den normale utløseren.
// Da dreneres det på tid i stedet for på fyllingsgrad, og hver drenering koster en
// sync-post i råloggen uansett hvor få ord den hentet.
static_assert(kFifoPollFallbackMs * kFifoWordsPerSec / 1000u > kFifoWatermark,
              "kFifoPollFallbackMs fires before the watermark does - it would become "
              "the normal drain trigger instead of the fallback it is meant to be");

// Just a test that if we are to use SFLP as basis for PSD, then we need to have SFLP enabled.
static_assert(kEnableSflp || !wave_use_sflp,
              "wave_use_sflp feeds the wave chain from the on-chip fusion, and "
              "kEnableSflp has switched that block off - enable it, or select the "
              "software AHRS");

// Choice of filter to config file as text.
static constexpr const char *wave_orientation_name = wave_use_sflp ? "SFLP" : WaveAhrs::kName;

#endif  // WAVE_CONFIG_H
