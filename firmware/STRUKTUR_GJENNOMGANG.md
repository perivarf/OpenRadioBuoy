# Strukturgjennomgang av firmware

Gjennomgang av `OpenRadioBuoyFork/firmware` (~9 100 linjer, eksklusive `.pio` og
vendored `OneWire`). Ingenting er endret — dette er kun en liste over det som ser ut til
å kunne forbedres, sortert etter hvor mye det haster.

Datert 2026-08-04, mot `wave_build_seq 2`.

---

## 1. Reelle feil, ikke stilspørsmål

Disse bør vurderes først, fordi de kan gi galt resultat eller korrupt minne — ikke bare
være ubehagelige å lese.

### 1.1 `debugInfoLogCount[4]` indekseres med verdiene 3–7

[sd_writer.h:79](common_libraries/sd_writer/src/sd_writer.h#L79) deklarerer

```cpp
uint32_t debugInfoLogCount[4] = {0,0,0,0};
```

men [sd_writer.cpp:107-111](common_libraries/sd_writer/src/sd_writer.cpp#L107-L111) indekserer
den med `DEBUG_MODE_NOTECARD_RESET`(3) … `DEBUG_MODE_BUOY_RESCUE`(7):

```cpp
debugInfoLogCount[DEBUG_MODE_NOTECARD_RESET] = ...;   // indeks 3 - siste gyldige
debugInfoLogCount[DEBUG_MODE_BST_HEARTBEAT]  = ...;   // indeks 4 - UTENFOR
...
debugInfoLogCount[DEBUG_MODE_BUOY_RESCUE]    = ...;   // indeks 7 - UTENFOR
```

Fire skrivinger utenfor arrayet, rett inn i nabofeltene i `SDWriter`. Samme feil i
`startDebugging()` der tellerne inkrementeres. Årsaken er at `OLB_SD_*` (0–2) og
`DEBUG_MODE_*` (3–7) deler ett nummerrom, men bare de siste har en array bak seg.

Se også [§3.1](#31-modus-konstanter-som-burde-vært-enum) — en `enum class` med egen
indeksering ville gjort feilen umulig.

### 1.2 Sørlig breddegrad og vestlig lengdegrad blir korrupt

[gps_manager.cpp:468](common_libraries/gps_manager/src/gps_manager.cpp#L468):

```cpp
reading.lat = (uint32_t)(scale_factor * (pvt.lat_e7 * 1e-7));
```

`pvt.lat_e7` er `int32_t` og korrekt fortegnet fra UBX, men `GPS_Data::lat` er
`uint32_t`. En negativ verdi gir udefinert oppførsel i float→unsigned-konverteringen og
i praksis et enormt positivt tall. Mottakersiden leser det tilbake som `uint32_t` og
lagrer i `GPS_Reading::lat`, som er `int32_t`
([message_parser.cpp](common_libraries/message_tools/src/message_parser.cpp)) — så
fortegnet er tapt før det kommer dit.

Virker på Skjærhalden (59°N, 11°Ø). Feiler på sørlige halvkule og vest for Greenwich.

Merk at `msg_insert_int` allerede finnes for fortegnede verdier og brukes av
thermo_manager. GPS-en bruker `msg_insert_uint`.

### 1.3 `operationDone` er `static` i en header

[lora_transceiver.h:16](common_libraries/lora_transceiver/src/lora_transceiver.h#L16):

```cpp
static volatile bool operationDone = false;
```

`static` på filnivå i en header gir **én kopi per oversettelsesenhet**. ISR-en `setFlag()`
ligger i `lora_transceiver.cpp` og setter den kopien; enhver annen `.cpp` som inkluderer
headeren får sin egen som aldri endres. Det fungerer i dag kun fordi all bruk tilfeldigvis
er i samme `.cpp`.

Bør være `extern volatile bool` i headeren og definisjonen i `.cpp`. Helst
`std::atomic<bool>` eller minst `volatile sig_atomic_t`.

### 1.4 `int8_t` som størrelsestype

[stats.h:27-28](common_libraries/stats/src/stats.h#L27-L28):

```cpp
int8_t oldsize = data.size();
int8_t newsize = data.size();
```

Overflower ved mer enn 127 elementer. `readings_per_measurement` er 15 i dag, så det
går bra — men typen er feil uansett, og `max_number_of_measurements` er 40 og kan økes.
`size_t` eller `int` koster ingenting her.

Samme funksjon returnerer `double` men avslutter med `return (T) d_mean;` — kaster til
`T` (ofte `uint32_t`), trunkerer desimalene, og konverterer implisitt tilbake til
`double`. Enten returner `T`, eller ikke kast.

### 1.5 `GPS_Manager::updateTransmitMessage` returnerer konstanten, ikke faktisk lengde

[gps_manager.cpp](common_libraries/gps_manager/src/gps_manager.cpp) avslutter med
`return GPS_message_size;`, mens thermo og wave returnerer `offset` — det som faktisk ble
skrevet. Skulle `GPS_message_size` noen gang bli feil beregnet, sender GPS-en et
feilstørrelses-buffer uten at noe klager. De to andre ville avslørt det.

---

## 2. Duplisering

### 2.1 To sett med reading-structs for samme data

| avsender-side | mottaker-side | forskjell |
|---|---|---|
| `GPS_Data` ([gps_manager.h:16](common_libraries/gps_manager/src/gps_manager.h#L16)) | `GPS_Reading` ([readings.h:22](common_libraries/message_tools/src/readings.h#L22)) | `uint32_t` vs `int32_t` på lat/lng/vel/direction |
| `temperatureReading` ([thermo_manager.h:28](common_libraries/thermo_manager/src/thermo_manager.h#L28)) | `temperature_Reading` ([readings.h:14](common_libraries/message_tools/src/readings.h#L14)) | feltrekkefølge, `num_sensors` finnes bare på mottaker |
| `WaveResult` ([wave_manager.h:18](common_libraries/wave_manager/src/wave_manager.h#L18)) | `wave_analysis_Reading` ([readings.h:48](common_libraries/message_tools/src/readings.h#L48)) | `float` vs fixed-point `uint32_t` |

Wave-paret er det eneste der forskjellen er *begrunnet* (float på device, skalert heltall
på wire). De to andre er samme data i to former, og §1.2 er direkte konsekvens av det.

Forslag: én struct per målestørrelse i `readings.h`, brukt på begge sider. Der wire-formen
må avvike, gjør konverteringen eksplisitt i én funksjon i stedet for å ha to typer som
ser like ut.

### 2.2 `updateTransmitMessage` er skrevet tre ganger

De tre implementasjonene har identisk form: tag-byte → `msg_insert_*` per felt →
`'E'` → `pop_front()` → returner lengde. Forskjellen er feltlisten.

Forslag: en liten `MessageBuilder`-hjelper som eier `msgB`, `offset` og
grensesjekken, slik at hver manager kun lister feltene sine:

```cpp
MessageBuilder b(msgB, sizeof msgB, 'G');
b.u16(d.readingID).i32(d.lat).i32(d.lng).u32(d.vel).u32(d.direction).time(d.timestamp);
return b.finish();          // skriver 'E', returnerer faktisk lengde
```

Det fjerner også §1.5, siden `finish()` alltid returnerer det som ble skrevet.

### 2.3 Feltvis kopiering i `processReadings`

[gps_manager.cpp:530-560](common_libraries/gps_manager/src/gps_manager.cpp#L530-L560)
gjentar den samme ti-linjers blokken fire ganger (lat, lng, vel, direction), med en
kommentar som innrømmer det:

```cpp
// As it is, afaik, not possible to iterate over struct variables
// We instead have to perform the iteration though code repetition
```

Det er mulig — en array av pekere-til-medlem løser det:

```cpp
static constexpr uint32_t GPS_Data::*kFields[] = {
    &GPS_Data::lat, &GPS_Data::lng, &GPS_Data::vel, &GPS_Data::direction};
for (auto f : kFields) { ... packet[i].*f ... }
```

Alternativt legg de fire i en `uint32_t vals[4]` i structen og gi dem navngitte
aksessorer.

### 2.4 Manager-mønsteret er ikke uttrykt noe sted

`GPS_Manager`, `Thermo_Manager` og `WaveManager` har alle: `begin()`, `wake()`/`sleep()`,
en «ta målinger»-metode, `processReadings()`, `updateTransmitMessage()`, en `msgB`-buffer
og en `etl::deque` av resultater. Ingen felles base eller konsept — så `main.cpp` må vite
at GPS-en heter `performNReadings`, thermo `takeReadings` og wave `takeReading`.

Forslag: et felles `SensorManager`-grensesnitt (ikke nødvendigvis virtuelt — en
konvensjon dokumentert ett sted holder langt), slik at `loop()` kan behandle dem likt og
nye sensorer arver rekkefølgen gratis.

### 2.5 Taper-logikken finnes i to kopier

`lowFreqTaper()` er `static` i
[wave_analysis.cpp:87](common_libraries/wave_manager/src/wave_analysis.cpp#L87), og den
samme halvcosinusen er skrevet ut på nytt inline i `spec.csv`-skrivingen i
[wave_manager.cpp:406-407](common_libraries/wave_manager/src/wave_manager.cpp#L406-L407).
Endres `kTaperF1/F2`-formelen ett sted, glir spec.csv fra spekteret analysen faktisk
brukte. Flytt den til `wave_config.h` eller eksponer den fra `wave_analysis.h`.

### 2.6 NED-rotasjonen duplisert

[imu_sampler.cpp:149-151](common_libraries/wave_manager/src/imu_sampler.cpp#L149-L151)
skriver ut hele body→world-rotasjonsmatrisen inline, mens
[rotation.cpp:42](common_libraries/wave_manager/src/rotation.cpp#L42) `verticalAccel()`
har tredje rad av den samme. Sampleren trenger alle tre aksene (bremsedeteksjon), så en
`rotateToWorld(q, a, out[3])` i `rotation.h` ville dekket begge og fjernet risikoen for at
de to divergerer.

---

## 3. Enums og typebruk

### 3.1 Modus-konstanter som burde vært enum

[sd_writer.h:15-22](common_libraries/sd_writer/src/sd_writer.h#L15-L22) er åtte løse
`static constexpr uint8_t` i ett nummerrom, lagret i `uint8_t mode`. To disjunkte
begreper er blandet: kortets tilstand (inaktiv/skriv/les) og hvilken debug-fil som skrives.
Det er årsaken til §1.1.

Forslag: `enum class SdMode : uint8_t { Inactive, Write, Read };` og
`enum class DebugLog : uint8_t { NotecardReset, BstHeartbeat, NotehubSync, BuoyComm, BuoyRescue, COUNT };`
med `debugInfoLogCount[(size_t)DebugLog::COUNT]`. Da er §1.1 en kompileringsfeil.

Samme gjelder [common_config.h:24-26](include/common_config.h#L24-L26) —
`BUOY_MODE`/`BST_MODE`/`MOORED_MODE` er en enum.

### 3.2 `GpsPollState` er allerede riktig gjort

[gps_manager.h:127](common_libraries/gps_manager/src/gps_manager.h#L127) bruker en scoped
`enum GpsPollState : uint8_t`. Det er mønsteret de andre bør følge — verdt å nevne fordi
det viser at konvensjonen finnes i kodebasen allerede.

### 3.3 `AccelFS`/`GyroFS` er det beste eksempelet

[wave_config.h:126-128](common_libraries/wave_manager/src/wave_config.h#L126-L128) bruker
`enum class` der enum-verdien *er* tallet som sendes til driveren, med sensitiviteten
derivert fra samme kilde. Den ideen — «konstanten og dens konsekvens kan ikke gli fra
hverandre» — er verdt å kopiere til `sd_writer` og `lora_transceiver`.

### 3.4 Magiske tall

- `1000.0f` (mg per g) skrives ut i [imu_sampler.cpp:152](common_libraries/wave_manager/src/imu_sampler.cpp#L152)
  selv om `kMg2Ms2` og `kGravity` finnes. En `kMgPerG` ville gjort raden lesbar.
- `0x1FFF0000UL` i [drifter/src/main.cpp:45](drifter/src/main.cpp#L45) er dokumentert i
  kommentar — bra, men hører hjemme som navngitt konstant.
- `10000` som `message_send_time` gjentas fire steder i `task_transmit()`.
- `delay(500)` / `delay(200)` gjentas gjennom hele `task_transmit()` uten navn.

### 3.5 `String` i et ellers ETL-basert prosjekt

`StringMessage` ([messages.h:43](common_libraries/message_tools/src/messages.h#L43)) og
`SDWriter::startLogging(String)` bruker Arduino `String`, som allokerer på heap.
Resten av prosjektet bruker konsekvent `etl::vector`/`etl::deque`/`etl::string` nettopp
for å unngå fragmentering. De to `String`-overloadene ser ut til å kunne fjernes til
fordel for `const char *`-varianten som allerede finnes side om side.

---

## 4. Filstruktur

### 4.1 `wave_manager` er blitt to biblioteker i ett

Katalogen inneholder nå to lag som ikke deler avhengigheter:

| lag | filer | avhenger av |
|---|---|---|
| ren matematikk | `fir`, `quat_delay`, `madgwick`, `kalman`, `rotation`, `matrix` | kun `<math.h>`/`<stdint.h>` |
| hardware/IO | `imu_sampler`, `wave_analysis`, `wave_manager`, `wave_config` | Arduino, SPI, LSM6DSV16X, SdFat |

Det øverste laget er bevisst Arduino-fritt og kompilerer på host — det er dokumentert i
hver av filene. Men skillet er usynlig i katalogstrukturen.

Forslag: `wave_manager/src/dsp/` (eller et eget `common_libraries/wave_dsp/`) for det
rene laget. Da blir det synlig at en `#include <Arduino.h>` der er en regresjon, og
host-testing av `fir.cpp` blir en naturlig del av oppsettet i stedet for en engangsjobb.

### 4.2 `ImuRow` drar hele driveren med seg

`wave_analysis.h` inkluderer `imu_sampler.h` utelukkende for `ImuRow`, og får dermed
`SPI.h` og `LSM6DSV16XSensor.h` på kjøpet. Analysatoren rører ingen av dem.

En egen `imu_row.h` ville brutt den koblingen. *(Merk: dette ble foreslått og avvist
2026-08-04 — det ble vurdert som feil å endre firmware-struktur for testbarhetens skyld.
Tas med her fordi koblingen også er et strukturpoeng uavhengig av testing, men det er en
avgjort sak.)*

### 4.3 To `config.h` med samme navn

`drifter/src/config.h` og `basestation/src/config.h` heter det samme og velges av
`-I drifter/src` respektive `-I basestation/src` i `platformio.ini`. Det virker, men
gjør det umulig å se fra en `#include "config.h"` hvilken fil som treffes, og et bibliotek
som inkluderer den kompileres forskjellig i de to miljøene uten at det er synlig.

Forslag: `drifter_config.h` / `basestation_config.h`, eller flytt alt delt til
`common_config.h` og la target-filene kun inneholde det som *faktisk* skiller seg.

### 4.4 `examples/` har egne `platformio.ini`

`examples/read_GPS` og `examples/read_temperatures` har hver sin `platformio.ini` og
`config.h`, uavhengig av hovedprosjektet. De vil råtne stille når API-et endres, siden
ingenting bygger dem. Enten legg dem inn som miljøer i hoved-`platformio.ini`, eller
merk dem tydelig som frosne.

### 4.5 Vendored `OneWire`

`common_libraries/thermo_manager/OneWire/` er et tredjepartsbibliotek (1 363 linjer) sjekket
inn i kildetreet, blandet med prosjektets egen kode. Det er 15 % av kodebasen målt i
linjer. Bør enten være en `lib_deps`-avhengighet, eller ligge tydelig adskilt fra egen
kode slik at «hvor mye kode eier vi» er et svarbart spørsmål.

---

## 5. Metoder

### 5.1 `setup()` gjør elleve ting

[drifter/src/main.cpp:88-257](drifter/src/main.cpp#L88-L257) er ~170 linjer:
serial, bootloader-meny, RTC, watchdog, SD, radio, ETL-feilhåndtering, GPS-fix-venting,
klokkesynk, termistorer, IMU, deployment-melding, lavstrømsoppsett. Hver blokk har en
kommentar som i praksis er et funksjonsnavn.

Forslag: trekk ut `initStorage()`, `initRadioOrHalt()`, `waitForFirstFix()`,
`sendDeploymentMessage()`. Ikke for lengdens skyld, men fordi rekkefølgen mellom dem er
betydningsfull (SD før IMU pga. delt SPI) og da blir den lesbar på ett skjermbilde.

### 5.2 `task_transmit()` blander protokoll og logging

[drifter/src/main.cpp:347-461](drifter/src/main.cpp#L347-L461) veksler mellom
radiokall, SD-logging, `delay()` og debug-print i samme flyt. De to `while`-løkkene har
identisk struktur (send → logg → logg signal → tell → delay) for ulike køer.

Forslag: `sendQueue(tag, buffer, size, count)` som gjør send/logg/tell, kalt tre ganger.

### 5.3 Duplisert overload-par gjennom hele `SDWriter`

`startLogging`, `startReading`, `logString` finnes hver i `String`- og
`const char *`-variant. Se §3.5 — hvis `String`-varianten fjernes forsvinner halvparten
av `SDWriter`s offentlige flate.

### 5.4 `filter_vector` gjør en tilnærmet middelverdi

[stats.h:24-43](common_libraries/stats/src/stats.h#L24-L43) trekker hvert forkastet punkts
bidrag fra middelverdien og reskalerer, i stedet for å regne middelverdien på nytt over
de gjenværende. Resultatet er ikke identisk med et rent filtrert gjennomsnitt, og
avviket vokser med antall forkastede punkter. Hvis tilnærmingen er bevisst (den er
billigere) bør det stå; hvis ikke er en andre passering enklere å forsvare.

Ved `newsize == 0` returneres 0 — altså «alle punkter er uteliggere» gir posisjon null.
For lat/lng er det en verdi som ser gyldig ut.

### 5.5 `msg_insert_uint` / `msg_extract_uint` bruker 64-bits divisjon

[parser_utils.h](common_libraries/parser_utils/src/parser_utils.h) bygger opp en
`uint64_t fac` og deler med 256 per byte. På Cortex-M4 uten 64-bits maskinvaredivisjon er
det et bibliotekkall per byte. Skift og maske gjør det samme:

```cpp
for (uint8_t i = 0; i < sizeof(T); i++)
  msg[start + i] = (uint8_t)(number >> (8 * (sizeof(T) - 1 - i)));
```

Ligger ikke i en varm løkke, så dette er lesbarhet mer enn ytelse.

### 5.6 `msg_insert_int` bruker sign-magnitude

Fortegn skrives som en `'P'`/`'N'`-byte foran tallet — ett ekstra byte per fortegnet felt,
og to representasjoner av null. Toerkomplement ville brukt samme antall byte som
`uint` og fjernet særtilfellet. Endringen er wire-format-brytende, så den hører sammen med
en versjonsbump.

### 5.7 Blokkerende `while`-løkker uten timeout

`setup()` har `while (nofix)` og `while (!LORA.connectToBaseStation(...))`, begge med
`IWatchdog.reload()` inni. De kan i prinsippet kjøre i det uendelige uten at watchdogen
avbryter — som er tilsiktet ved utsetting, men verdt en eksplisitt kommentar om at
watchdogen bevisst holdes i live her.

`while(1);` ved radiofeil ([main.cpp:137](drifter/src/main.cpp#L137)) fryser derimot
*uten* `reload()`, så watchdogen restarter boya i en evig reboot-løkke. Det er antakelig
ønsket oppførsel, men den er implisitt.

---

## 6. Navnekonvensjoner

Kodebasen bruker minst fire konvensjoner samtidig:

| stil | eksempler |
|---|---|
| `PascalCase` | `WaveParams`, `ImuRow`, `SDWriter`, `OneWire` |
| `Snake_Pascal` | `GPS_Manager`, `Thermo_Manager`, `LoRa_Transceiver`, `Message_Data`, `UBX_PVT` |
| `snake_case` | `temperatureReading` (egentlig camelCase), `wave_analysis_Reading`, `beacon_Reading` |
| `camelCase` | `buoyInfoReading`, `buoyInitMessage`, `fileLine` |

Globale instanser er like blandet: `gps_manager`, `thermo_manager`, `sd_writer`,
`wave_manager` — men `LORA` og `MESSAGE_PARSER` i store bokstaver.

Konstanter er mer konsekvente, men delt i to: `kCamelCase` i `wave_config.h` (`kImuOdrHz`,
`kWelchSegLen`) mot `snake_case` ellers (`welch_bins`, `max_message_length`,
`scale_factor`). Skillet følger omtrent «wave_manager mot resten», og siden `wave_config.h`
selv blander begge (`kPsdDfHz` ved siden av `welch_bin_min`) er det ikke en regel man kan
lese seg til.

Headerguard matcher heller ikke alltid filnavnet: `lora_transceiver.h` bruker
`LORA_MANAGER_H`.

Forslag: velg én konvensjon per kategori og skriv den ned. Dette er den billigste
opprydningen i lista, og den eneste som ikke kan innføre feil.

---

## 7. Konfigurasjon

### 7.1 `#define Serial mySerial` er en global makro

[common_config.h:17-18](include/common_config.h#L17-L18):

```cpp
#undef Serial
#define Serial mySerial
```

Den treffer *alle* oversettelsesenheter som inkluderer `config.h`, inkludert
tredjepartsbiblioteker (RadioLib, SdFat, TimeLib) som måtte referere `Serial`. Det
fungerer i dag, men er den typen makro som gir uforståelige feil den dagen et bibliotek
oppdateres. Et alternativ er å la `mySerial` hete `mySerial` overalt og la
`debugSerialPrint`-funksjonene i `sd_writer` være den ene inngangen.

### 7.2 Mutable globale konfigurasjonsvariabler

Det meste er `static constexpr`, men noen er `static` uten `const`:

```cpp
static uint32_t minimal_transmission_period {2*s_2_ms};   // drifter/src/config.h:20
static float    LoRa_freq_receive           {863};
static uint32_t base_measurement_period     {0*s_2_ms};
```

De ser ut som konfigurasjon, men er variabler — og siden de er `static` i en header har
hver `.cpp` sin egen kopi. Endrer noen én av dem i kjøretid (basestasjonen kan justere
måleperioden), ser bare den ene oversettelsesenheten det. Bør enten være `constexpr`, eller
ekte variabler med `extern` + definisjon i én `.cpp` — slik `LORA.measurement_period`
allerede er gjort.

### 7.3 Kommentert-ut kode

- [drifter/src/main.cpp:240-247](drifter/src/main.cpp#L240-L247) — åtte linjer
  SD-gjenopprettingskode.
- [main.cpp:444-447](drifter/src/main.cpp#L444-L447) — `receiveDesiredMeasrements` (med
  skrivefeil) og `fetchRequestedMeasurements`.
- `parser_utils.h:16` — en tidligere `fac`-beregning.
- `readings.h:1,8` — `// #include "etl/deque.h"`, `// typedef byte BuoyID[8];`

Git husker dette. I fila er det bare støy som må leses forbi.

---

## 8. Det som allerede er bra

Verdt å notere, både fordi det er sant og fordi det viser hvilken standard resten kan
måles mot:

- **`wave_config.h` begrunner tallene sine.** Sveip, målte verdier, og hvorfor et
  alternativ ble forkastet. Det er uvanlig og svært verdifullt.
- **`static_assert`-gjerdene.** `fir_coeffs.h` nekter å bygge hvis ratene endres uten
  regenerering; `welch_bin_max` sjekkes mot `welch_bins`, mot PSD-lengden og mot
  `kWaveFMax`. Flere av dem fanget faktiske feil under arbeidet 2026-08-04.
- **Det Arduino-frie DSP-laget.** `madgwick`, `kalman`, `rotation`, `fir`, `quat_delay`
  kompilerer på host, som gjorde impulsrespons- og forsinkelsestesten mulig uten
  hardware.
- **Deriverte konstanter.** `kPsdDfHz = kVaccFsHz / kWelchSegLen`, `kLpf2Bw` fra
  `kLpf2Div`, sensitivitet fra `kAccelFS`. Konstanten og konsekvensen kan ikke gli fra
  hverandre.

---

## 9. Foreslått rekkefølge

| # | tiltak | omfang | risiko |
|---|---|---|---|
| 1 | `debugInfoLogCount`-overskriving (§1.1) | små | ingen — ren feilretting |
| 2 | `operationDone` til `extern` (§1.3) | små | ingen |
| 3 | GPS lat/lng fortegn (§1.2) | middels | **wire-format**: begge sider må flashes |
| 4 | `int8_t`→`size_t` i `filter_vector` (§1.4) | små | ingen |
| 5 | `enum class` for SD-modus og debug-logg (§3.1) | små | ingen — gjør §1.1 umulig |
| 6 | Navnekonvensjon skrevet ned og fulgt for ny kode (§6) | små | ingen |
| 7 | Taper-duplikatet (§2.5) og NED-rotasjonen (§2.6) | små | ingen |
| 8 | `MessageBuilder` (§2.2), fjerner også §1.5 | middels | testes mot basestasjonen |
| 9 | Én reading-struct per målestørrelse (§2.1) | stor | henger sammen med §3 |
| 10 | Del `wave_manager` i dsp/ og io/ (§4.1) | middels | kun include-stier |
| 11 | Del opp `setup()` og `task_transmit()` (§5.1, §5.2) | middels | vanskelig å teste uten hardware |

Punkt 1, 2, 4 og 5 er isolerte og kan gjøres uten å røre noe annet. Punkt 3 og 8 endrer
wire-format og hører sammen med en `wave_build_seq`-bump og samtidig flashing av begge
enheter.
