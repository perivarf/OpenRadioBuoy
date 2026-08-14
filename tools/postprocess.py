#!/usr/bin/env python3
"""Offline etterprosessering av en ORB-loggeøkt - speiler firmware-ens
DEBUG_POSTPROCESS-sti (analysis::postProcess / StreamAnalyzer).

Leser <stamp>_imu.csv for en gitt økt og kjører NØYAKTIG samme kjede som
analysis.cpp:
  Madgwick 6-akse AHRS  ->  vertikal lineær accel  ->  desimering (--decimate-hz,
                                                        default 10 Hz / 100 ms)
  ->  Welch-PSD (Hann/Hamming-vindu, 75 % overlap, Σw²-normalisering)
  ->  acc->elevasjon (÷ω⁴) med lavfrekvens-taper (T²)
  ->  spektralmomenter m0/m2/m4  ->  Hs = 4√m0, Tz = √(m0/m2), Tc = √(m2/m4).

Lavfrekvens-avskjæringen er valgbar (--cutoff):
  taper : fast Kohout/Tucker-Pitt halv-cosinus mellom f1 og f2 (default).
  auto  : cut-off-frekvensen f_c finnes automatisk som første spektrale minimum
          over 0.05 Hz i elevasjonsspekteret (SciPy find_peaks på -S, minste
          avstand i frekvens-bins og prominens 0.05), og spekteret nulles under
          f_c. Basert på Rabault et al. (2022, fig. 7), Tucker (1958) og
          Waseda et al. (2018). NB: bommer deteksjonen blir amplitudene (og Hs)
          ufysisk store - vi faller da tilbake til fast taper og advarer.

Orienteringsmetodene beregnes og sammenlignes (som post-buildet):
  - Madgwick: rekonstruert her fra rå accel/gyro (som postProcess gjør).
  - SFLP:     on-chip-quaternionens vertikal-accel, lest fra az_ned_sflp-kolonnen
              (het az_ned før build_seq 3 - se imu_col_names).
  - Kalman:   error-state-KF med ADAPTIV R (kalman.py). Referansetuningen; siden
              2026-08-03 kjører firmware samme filter - se MEKF under.
  - NXP:      NXP-ens 12-tilstands Kalman, altså filteret SFY-bøya kjører, via
              den kompilerte ahrs_fusion-modulen (kalman_nxp.py). Med her for å
              kunne sette ORB-kjeden opp mot SFY-kjeden på samme rådata. NB:
              fast rate, ikke rate-invariant - se kalman_nxp.py.
  - MEKF:     bøyas EGET Kalman-filter, altså kalman.py med parametrene fra
              wave_config.h (mekf.py). Firmwarens KalmanAhrs er en port av
              kalman.py, så kolonnen er identisk med Kalman-kolonnen når de to
              parametersettene er like - og forskjellig nøyaktig når drifteren er
              tunet annerledes. Se mekf.py.

Skriver <stamp>_spec_python.csv og <stamp>_ana_python.csv i øktkatalogen (samme
format som firmware) og printer sammenligningstabellen til stdout. _python-
suffikset er der fordi firmware skriver sine egne <stamp>_spec.csv/_ana.csv i
samme katalog - de skal ALDRI overskrives herfra.

Bruk:
    python3 postprocess.py <sti>
        <sti> = øktkatalogen (f.eks. .../20260714_084020 eller ..._tmp),
                ELLER direkte til <stamp>_imu.csv.

DIAGNOSTIKK (skrives alltid): svarer på om lavfrekvensenden er sjø eller ikke.
  1. Støygulv - median accel-PSD i NOISE_BAND + log-log-helning. Er S_acc flat
     (helning ~0), er S_eta = S_acc/ω⁴ nødvendigvis ∝ f⁻⁴: det SER ut som en
     lavfrekvenstopp ved taperens knekk, men inneholder ikke én bølge. Nivået
     holdes mot sensorens eget gulv (SENSOR_NOISE_UG) og mot m0.
  2. Tilt - hvor stor vinkelfeil δθ som kreves for å produsere gulvet (g·δθ),
     som andel av bøyas faktisk målte helning i samme bånd.
  3. Sentripetal - |a| er ORIENTERINGSFRI, så et gulv der kan ikke komme fra
     attitude-estimatet. Armlengden r som lar ω²·r forklare hele |a|-gulvet,
     pluss korrelasjonen mellom |a| og |ω|² i båndet. Høy korrelasjon + fysisk
     r => gulvet er ekte akselerasjon fra bøyas egen rugging, ikke bølger og
     ikke støy - og da hjelper verken taper, detrending eller bedre filter.

Konstantene under speiler settings.h. Overstyr seglengde/vindu m.m. med flagg:
    python3 postprocess.py <sti> --seglen 2048 --window hamming
    python3 postprocess.py <sti> --detrend none      # firmware-tro (ingen)
    python3 postprocess.py <sti> --noise-band-lo 0.08 --noise-band-hi 0.20
    python3 postprocess.py <sti> --decimate-hz 25 --skip-start 300

FJERNET: armlengde-kompensasjonen (f_P = f_IMU − ω×(ω×r) − α×r) med tilhørende
lukket-form-tilpasning, selvtest og rotasjons-koherens. Konklusjonen som gjorde den
overflødig: på Skjærhalden-øktene gjenfant selvtesten en injisert r til 0.01 cm med
94-99 % energireduksjon, mens ekte data ga 0.6 % - og vertikalkomponenten ble verre.
Stiv armlengde om et fast kroppspunkt forklarer altså IKKE lavfrekvensgulvet der, til
tross for korr(|a|,|ω|²) = +0.68. Sentripetal-diagnostikken (punkt 3 over) står igjen
og rapporterer fortsatt den impliserte r-en.

Avvik fra firmware (bevisste, for analyse):
  - --decimate fir: FIR-lavpass + desimering i stedet for bøtte-middel, samme kjede
    som sfy4-bøya (src/fir.rs, påført rett etter AHRS og før Welch). 
  - detrending per segment (--detrend, default linear; firmware har ingen)
  - hull i 10 Hz-serien fylles så tidsaksen blir uniform (firmware komprimerer
    dem stille); fylte bøtter flagges og kan forkaste segmentet
  - Kalman som tredje metode, NXP som fjerde, MEKF som femte, GPS vUp som sjette

Krever: numpy. (scipy kun for --cutoff auto, matplotlib kun for --plot.)
"""

import argparse
import math
import os
import subprocess
import sys
import time
from collections import namedtuple
from glob import glob

import numpy as np

# Rotasjonsmatematikk og AHRS-ene ligger i egne moduler ved siden av denne fila.
# sys.path utvides fordi skriptet også kjøres direkte fra andre kataloger (og
# fra Spyder med F5), der tools/ ikke er på importstien.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rotation import world_z          # noqa: E402
from madgwick import Madgwick         # noqa: E402
from kalman import Kalman             # noqa: E402
from kalman_nxp import KalmanNxp      # noqa: E402
from mekf import Mekf                 # noqa: E402
from welch import welch_psd, window_weights   # noqa: E402
import fir                            # noqa: E402

# --- Konstanter fra settings.h (endre her eller via CLI-flagg) ---------------
GRAVITY = 9.80665            # kGravity (m/s²)
MG2MS2 = GRAVITY / 1000.0    # mg -> m/s²
MDPS2RADS = 1.0e-3 * math.pi / 180.0  # mdps -> rad/s
MADGWICK_BETA = 0.2          # kMadgwickBeta -> 0.2 gir omtrentlig lik som Kalman/SFLP
# Analyseraten. Dette er STARTVERDIER: --decimate-hz skriver om begge to i main()
# før noe annet leser dem. Alle bruksstedene slår opp FS ved kall, så de følger
# automatisk. NB: en «fs=FS» i en funksjonssignatur ville IKKE fulgt med -
# default-argumenter bindes ved import. Send raten som argument i stedet
# (welch.py og fir.py gjør nettopp det).
# NB: 50 ms = 20 Hz, IKKE firmwarens 10 Hz. Firmware setter kWelchInputOdrHz = 10
# (kWelchInputPeriodMs = 100). Speilet ligger altså ETT hakk høyere enn enheten, og
# docstringen øverst i fila sier fortsatt «default 10 Hz / 100 ms» - én av de tre er
# feil, og det er ikke avklart hvilken. Konsekvensen er målbar: 50 ms går ikke opp i
# radperioden til en 50 Hz-logg (20 ms), så fir.py advarer og desimeringen glir mot
# tidsaksen. Ved 100 ms ville 100 % 20 == 0 gått opp for alle loggeratene vi har.
# Firmwarens static_assert (kWelchInputPeriodMs % kRowPeriodMs == 0) ville avvist
# denne konfigurasjonen ved kompilering; her er den bare en advarsel.
BUCKET_MS = 50              # kWelchInputPeriodMs på enheten - men se over
FS = 1000.0 / BUCKET_MS      # kWelchInputOdrHz-ekvivalenten = 20 Hz
SEGLEN = 512*2                # kWelchSegLen (post-build)
OVERLAP_DIV = 4              # kWelchOverlapDiv (4 => 75 % overlap)
WINDOW = "hann"             # kWelchWindow: "hann" | "hamming"
WAVE_FMAX = 5.0              # kWaveFMax (øvre båndgrense, Hz) - hevet for kort fjordchop

# Kohout 0.03-0.05. NB dønninger / hav
# Fjord? 0.08-0.12?
TAPER_F1 = 0.03             # kTaperF1 (T=0 under denne). Kohout 0.03; hevet: ingen dønning i fjord => kutt periode > ~12 s
TAPER_F2 = 0.05              # kTaperF2 (T=1 over denne). Kohout 0.05; hevet: full vekt fra ~8 s og ned


def default_seglen(fs):
    """Seglengde når --seglen ikke er oppgitt: fast trapp, ikke utregnet formel.

    Poenget er å holde SEGMENTVARIGHETEN (og dermed df) omtrent i ro når raten
    endres - ikke antall bins. Med fast seglen ville 50 Hz gitt 20.5 s segmenter
    og df = 0.049 Hz, og da er taperbåndet 0.1-0.2 Hz bare to bins bredt.
    Trappa holder varigheten i 82-102 s over hele det aktuelle rateområdet."""
    if fs <= 10.0:
        return 1024          # 102.4 s @ 10 Hz - dagens default, uendret
    if fs < 26.0:
        return 2048          # 102.4 s @ 20 Hz
    if fs <= 52.0:
        return 4096          # 81.9 s @ 50 Hz
    return 8192              # 81.9 s @ 100 Hz

# --- Fra øktas cfg.csv -------------------------------------------------------
# Firmware skriver <stamp>_cfg.csv med alle konstantene økta ble tatt opp med.
# Vi henter KUN logge-parametrene derfra - altså det som beskriver hvordan
# imu.csv/gps.csv ble til. Analysevalgene over (BUCKET_MS, SEGLEN, WAVE_FMAX,
# TAPER_*) er skriptets egne og skal IKKE leses fra økta: her avviker vi bevisst
# fra firmware (hevet fmax/taper for fjordchop), og en automatisk overstyring
# ville stille reversert den tuningen. Fra build 66 ligger firmwares versjon av
# de samme valgene i ana.csv, ikke cfg.csv, nettopp fordi de hører til analysen -
# write_ana skriver VÅRE verdier til _ana_python.csv på samme nøkler.
RAW_DT_MS = None             # kWindowMs fra cfg.csv; None => utled fra radene

# --- Automatisk cut-off (--cutoff auto) --------------------------------------
# f_c = første spektrale minimum over CUT_FMIN i elevasjonsspekteret, funnet med
# SciPy find_peaks på det negerte spekteret. Minsteavstand oppgis i frekvens-bins
# (kilden bruker 7 bins @52 Hz og 3 bins @26 Hz råsamplerate); her er df = fs/N =
# 10/1024 ~ 0.0098 Hz, så 3 bins ~ 0.029 Hz. Prominens 0.05 som i kilden.
CUTOFF_MODE = "taper"        # "taper" (fast f1/f2) | "auto" (find_peaks) | "gps"
CUT_FMIN = 0.05              # nedre søkegrense (Hz)
CUT_DISTANCE = 3             # minste avstand mellom minima [frekvens-bins]
CUT_PROMINENCE = 0.05        # prominens-krav på minimumet [m²/Hz]

# --- GPS-utledet cut-off (--cutoff gps) --------------------------------------
# GPS vertikalhastighet gir elevasjon med KUN ω⁻², mot IMU-ens ω⁻⁴. Ved 0.05 Hz
# er GPS-estimatet derfor ~ω² ≈ 1000x mindre følsomt for lavfrekvent feil, og
# duger som uavhengig fasit i lavfrekvensbåndet. f_c settes der IMU-spekteret
# slutter å løpe fra GPS: første frekvens der S_eta(IMU)/S_eta(GPS) <= GPS_RATIO
# i GPS_NCONSEC bins på rad (kravet om flere bins på rad demper bin-støy).
# NB: over ~0.3 Hz spriker de igjen fordi GPS-en (5 Hz + tracking-loop) glatter
# bort chop-en - GPS validerer lavfrekvensgrensen, ikke bølgebåndet.
# Kriteriet er RELATIVT og oppgitt i Hz, ikke absolutt og i bins. Grunnen: en
# absolutt terskel (f.eks. forhold <= 2) havner på bunnen av forholdskurven, som
# ligger rundt 2.0-2.4, og da avgjøres treffet av bin-støy framfor av signalet -
# f_c vandret 0.087/0.159/0.200/ikke-funnet over seglen 8192..512 på samme økt.
# Median-forholdet per bånd er derimot stabilt til ~10 % over samme spenn, så vi
# leter etter KNEKKPUNKTET i kurven relativt til dens eget minimum i stedet.
# Referansenivået er en LAV KVANTIL, ikke minimum: min() er selv støyfølsom og
# synker systematisk når segmentantallet faller (målt 2.02 -> 1.08 over seglen
# 512..8192 på samme økt), så terskelen krympet i takt med støyen. 10.-persentilen
# er stabil til ~3 % over samme spenn (2.35-2.43).
# GPS-taperen er en halv-cosinus-rampe f_c -> GPS_RAMP*f_c, ikke et hardt kutt.
# Et hardt kutt gir full vekt til bin-en rett over f_c, der elevasjonsspekteret er
# brattest og støyest, så Hs blir følsom for om f_c lander én bin til hver side.
# Målt spredning i Hs over seglen 512..8192: hardt kutt 14.6 %, x1.5 9.6 %,
# x2.0 5.8 %, x3.0 2.1 %. 2.0 er valgt fordi rampen da dekker nettopp båndet der
# IMU/GPS-forholdet fortsatt er forhøyet (4.8 ved f_c, 1.9 ved 2*f_c) - bredere
# ramper kjøper stabilitet ved å spise av det ekte bølgebåndet.
GPS_RAMP = 1.5
# Sikring mot runaway: rampens nedre kant klemmes mot denne grensen. Øvre kant
# følger av seg selv, siden f2 = GPS_RAMP*f1 - med 0.15 og rampe 2.0 kan f2 aldri
# overstige 0.3 Hz. Ved Tp ~ 1.5 s ligger bølgeenergien over 0.3 Hz, så en rampe
# som strakk seg dit ville spist av selve signalet. Normalt slår den ikke inn
# (f_c ~ 0.10 -> 0.10-0.20 Hz); den begrenser bare skaden hvis deteksjonen
# bommer grovt. 0 = av.
GPS_MAX_F1 = 0.15
# Søkebåndets ØVRE grense. Egen konstant, ikke WAVE_FMAX: en LAVFREKVENS-cut-off
# skal per definisjon ligge under bølgebåndet, mens WAVE_FMAX er hevet til 5 Hz
# for fjordchop-en. MÅLT på Skjærhallen-økta (20260731_110303): Kalman-metoden
# traff aldri kriteriet lavt nede, og første treff kom på 4.229 Hz - der ser
# GPS-en kun sitt eget støygulv, så det er en sammenligning av støy mot støy.
# Rampen 4.229-6.343 Hz nullet da hele spekteret (Hs = 0.000 m, Tz = 0.21 s).
# GPS_MAX_F1 klemmer bare skaden ned til 0.15 Hz; her sier vi i stedet at et
# treff der oppe IKKE er en cut-off. find_cutoff_gps returnerer None, og
# kalleren faller tilbake på den faste taperen - et ærlig "ikke funnet".
GPS_SEARCH_FMAX = 0.5
GPS_REL_FACTOR = 1.5         # terskel = denne x referansenivået
GPS_REF_QUANTILE = 10.0      # persentil av glattet kurve brukt som referansenivå
GPS_SMOOTH_HZ = 0.02         # glidende median over dette båndet før terskling
GPS_SPAN_HZ = 0.02           # forholdet må ligge under terskelen over dette spennet
GPS_NOISE_BAND = (1.2, 2.0)  # bånd for å estimere GPS-ens eget hvitstøy-gulv (Hz)
# GPS' EGEN cut-off: første spektrale minimum i GPS-elevasjonen (S_vUp/ω²), samme
# find_peaks som --cutoff auto. Sier hvor langt ned GPS-referansen selv er gyldig.
# Brukes ikke som taper - kun som gyldighetssjekk av referansen. Prominensen er
# absolutt (m²/Hz) og må trolig justeres per datasett/sted.
GPS_OWN_PROMINENCE = 0.005
# Minste GPS-SNR (S_vUp / eget støygulv) ved f_c for at referansen skal godtas.
GPS_MIN_SNR = 3.0

# --- Detrending per Welch-segment --------------------------------------------
# Firmware gjør INGEN detrending: accumSegment legger Hann rett på serien. vacc =
# wZ - g bærer accelerometerets skalafaktorfeil på 1 g (±1-3 %), bias (±20-40 mg)
# og bias-temperaturdriften (~0.1-0.5 mg/°C). Hann demper ren DC godt (~-80 dB i
# effekt ved bin 5), men IKKE en trend - en lineær drift over segmentet lekker som
# f⁻², rett inn i 0.04-0.08 Hz, der ÷ω⁴ forstørrer den maksimalt.
#   "none"   = firmware-tro (ingen fjerning)
#   "mean"   = trekk fra segmentets middel
#   "linear" = trekk fra minste-kvadraters rett linje (default her)
# Default avviker bevisst fra firmware; kjør --detrend none for firmware-tro tall.
DETREND = "linear"

# --- Desimering til 10 Hz: bøtte-middel eller FIR (speiler sfy-bøyas fir.rs) ---
# Firmware lager 10 Hz-serien ved å MIDLE alle rader i hver 100 ms-bøtte. Et slikt
# middel ER et filter - en boxcar på D = 10 samples - men et dårlig antialias-filter:
# amplituderesponsen |sin(πfD/fs)/(D·sin(πf/fs))| er null i multipla av 10 Hz og
# faller kun til ~-13 dB mellom nullpunktene. Alt over den nye Nyquist-frekvensen
# (5 Hz) BRETTES ned i bølgebåndet: energi ved f folder til |f − 10| Hz, så f.eks.
# rugging/slag ved 9.9 Hz lander på 0.1 Hz - midt i båndet der ÷ω⁴ forstørrer mest.
#
# sfy4-bøya (src/fir.rs + src/waves/buf.rs) gjør det riktige i stedet: en 129-taps
# firwin-lavpass legges på HVER akse rett etter AHRS-rotasjonen, og bare hvert
# D-te filtrerte sample beholdes (fir::Decimator). Nøyaktig samme kjede kan kjøres
# her: koeffisientene i firmware er bit-for-bit scipy.signal.firwin(129, 26, fs=208),
# og firwin_lowpass() under reproduserer den formelen for VÅR rate (100 Hz -> 10 Hz).
#
#   "mean" = firmware-tro gjennomsnitt av segment (default, uendret oppførsel)
#   "fir"    = FIR-lavpass på 100 Hz-radene, deretter behold hver D-te
DECIMATE_MODE = "fir"

# --- Hvor vertikal-accelen kommer fra ----------------------------------------
# To veier fram til den samme serien, og forskjellen er hvilken RATE AHRS-en så:
#
#   "csv"  Filteret replayes her, på radraten (100 Hz) - alt en imu.csv gir hvis
#          man bare ser på ax/gy-kolonnene. Dette var den eneste veien før.
#   "raw"  Kolonnen vacc_fir, der AHRS-en alt HAR kjørt på råstrømmen: om bord
#          ved kAhrsRateHz, eller offline av raw_to_csv fra raw.bin. Serien er
#          FIR-desimert til radrutenettet, samme trinn 1 som ax_mg.
#   "auto" raw når kolonnen finnes, ellers csv.
#
# Valget er ikke kosmetisk. Målt på 20260813_182728 med firmware-innstillinger:
# Hs 0.440 m fra vacc_fir (som treffer firmwarens egen ana.csv på 0.02 %) mot
# 0.473 m fra replayet - +7.5 %. Under taperen 0.15-0.3 Hz er forskjellen ~1 %,
# for avviket sitter i lavbåndet der attitydefeil dominerer. Derfor "auto" som
# default: finnes fasiten i fila, er det den som skal brukes.
#
# Bare den ene metoden bytter kilde - den AHRS-en fangsten faktisk kjørte
# (cfg.csv orientation_name). De andre radene i tabellen er fortsatt replay, og
# er der nettopp for å kunne sammenlignes med den.
VACC_SOURCE = "auto"

# Metodene, i den rekkefølgen de står i tabellen, og nøkkelen hver av dem har
# internt (psd_m, wp_m, tapers["m"], ...). Sto inline i write_ana; den listen er
# nå én, for src_/rate_-nøklene under MÅ dekke nøyaktig de samme metodene.
METHODS = (("madgwick", "m"), ("sflp", "s"), ("kalman", "k"),
           ("nxp", "n"), ("mekf", "e"))
# SFLP er ikke et AHRS vi kan kjøre: den kommer ferdig fra brikken (az_ned_sflp).
AHRS_METHODS = tuple((n, k) for n, k in METHODS if n != "sflp")

FIR_NTAP = fir.NTAP  # 129               # antall tap - som fir.rs' NTAP. MÅ være oddetall, se under


# Cut-off. fir.rs setter CUTOFF = OUT_FREQ/2, altså nøyaktig den nye Nyquist-
# frekvensen (26 Hz av 52 Hz ut). Det er bevisst raust: firwin legger -6 dB i selve
# cut-offen, så båndet like under 5 Hz brettes fortsatt delvis ned - fir.rs har en
# utkommentert TRUE_CUTOFF nettopp for dette. None => FS/2 (= 5 Hz), firmware-tro.
# Sett den lavere (f.eks. 3.5) for reell margin; bølgebåndet ligger uansett < 1 Hz.
FIR_CUTOFF = fir.CUTOFF  # None
# Gruppeforsinkelsen (NTAP−1)/2 samples kompenseres her, siden vi har hele serien.
# Firmware kan ikke det (strømmende) og bærer 0.64 s forsinkelse. Kompensasjonen er
# EKSAKT for odde NTAP: filteret er lineærfase, så forsinkelsen er et helt antall
# samples. Uten den ville IMU-serien ligget 0.64 s etter GPS-referansen og
# brems/overflow-maskene.
FIR_COMPENSATE_DELAY = fir.COMPENSATE_DELAY  # True

# --- Hull i 10 Hz-serien ------------------------------------------------------
# Både firmware og dette skriptet pushet tidligere ÉN sample per bøtteovergang,
# uansett hvor stort spranget var. Mangler det imu-rader (FIFO-overflow, blokkerende
# SD-flush, hoppede vinduer i Imu::fifo) blir et hull på flere sekunder til ett
# sample, og Welch-buffet får en ikke-uniform tidsakse mens df = fs/N antar eksakt
# 10 Hz. Ikke-uniform sampling smører energi bredbåndet, og ÷ω⁴ gir lavfrekvensenden
# det verste av det. Nå fylles hull med lineær interpolasjon slik at tidsaksen blir
# ekte uniform, og de fylte bøttene markeres i en maske så segmenter med for mye
# oppdiktet data kan forkastes (samme mekanikk som brems/fifo_ovf).
# NB: dette rettet også en reell skjevhet mot GPS - vg10 ble interpolert på de
# EKTE bøttesentrene (bidx10), mens IMU-serien var komprimert. Ved hull sammenlignet
# altså IMU/GPS-forholdet to ulike tidsakser.
GAP_REJECT = 0.01            # forkast segment hvis > denne andelen er utfylt hull
GAP_MAX_FILL = 50            # lengste hull som fylles [bøtter] (50 = 5 s); lengre
                             # hull fylles også, men er alltid flagget

# --- Støygulv-diagnostikk -----------------------------------------------------
# Er akselerasjons-PSD-en FLAT i et bånd, er innholdet der hvit støy - og da er
# elevasjonsspekteret (÷ω⁴) nødvendigvis ∝ f⁻⁴, altså en "topp" ved taperens knekk
# uten en eneste bølge i seg. Båndet velges mellom taperen og bølgebåndet.
NOISE_BAND = (0.08, 0.20)
# LSM6DSV16X accel-støytetthet, high-performance (datablad, typ). Kvantisering ved
# ±4 g bidrar 0.122/√12/√240 ~ 2.3 µg/√Hz i tillegg - neglisjerbart. Måler vi
# vesentlig mer enn dette, kommer støyen IKKE fra sensoren.
SENSOR_NOISE_UG = 70.0

# --- Tilt-lekkasje-diagnostikk ------------------------------------------------
# Vertikalfeil fra orienteringsfeil er g·sin(δθ) ~ g·δθ. Er tilt-spekteret i
# NOISE_BAND slik at g²·S_θ ≈ S_acc, er hele støygulvet gravitasjonslekkasje fra
# orienteringsestimatet - ikke sensorstøy og ikke bølger. I det båndet er reell
# bølge-indusert helning neglisjerbar, så sammenligningen er gyldig der.
TILT_DIAG = True

# Brems-forkasting (Rabault/SFY: brytende bølger gir lavfrekvent støy i elevasjonen).
# Hopp over Welch-segmenter der andelen brems-flaggede 10 Hz-bøtter > denne terskelen.
# Serien slettes ALDRI (ville forskjøvet timingen); vi lar bare være å akkumulere det
# forurensede segmentet i PSD-snittet. 0.0 = av (ta med alle). Overstyres --brake-reject.
BRAKE_REJECT = 0.05

# FIFO-overflow-forkasting (fifo_ovf-kolonnen, firmware build 55+). Et flagget
# vindu betyr at FIFO-en overskrev sine eldste samples - da MANGLER det data i
# serien, og den virtuelle sample-klokka har komprimert tidsaksen tilsvarende.
# Default 0.0 = enhver forekomst forkaster segmentet, siden tapt data ikke kan
# repareres. None = av. Eldre økter uten kolonnen får maske 0 og påvirkes ikke.
OVF_REJECT = 0.0

# --- De to Kalman-kolonnene ---------------------------------------------------
# Samme filter, to parametersett:
#   Kalman (kalman.py) - referansetuningen, sveipet mot Skjærhalden-øktenes
#       støygulv. Se sveipene i kalman.py.
#   MEKF (mekf.py)     - samme klasse med konstantene fra firmwarens
#       wave_config.h, altså det drifteren selv ville regnet ut.
# Er de to like, er kolonnene identiske - da er avvik et varsel om at firmware og
# offline har drevet fra hverandre. Konstantene bor i hver sin modul, ikke her.
# For en bølgebøye MÅ accel-tilten trustes sterkt (liten R) - da pinnes den
# lavfrekvente orienteringen til gravitasjonsretningen. Stor R gir lavfrekvent
# orienteringsdrift som ω⁻⁴ forsterker til urimelig Hs; den adaptive R-en gjør
# unntaket fra dette der accelen faktisk ikke ER gravitasjon (slag, rask rulling).
# --- Sti for F5-kjøring i Spyder ---------------------------------------------
# Spyder (F5) kjører fila uten kommandolinje-argumenter, så sett øktstien her når
# du vil kjøre interaktivt. Kan være øktkatalogen (f.eks. ".../20260714_084020"
# eller ..._tmp) ELLER direkte til <stamp>_imu.csv. Overstyres alltid hvis du gir
# sti som argument på kommandolinjen (python3 postprocess.py <sti>). Tom => krev arg.
#EFAULT_PATH = "/home/pif/master/Målinger/m-linger/Skjærhallen/20260731_110314"


DEFAULT_PATH = "/home/pif/master/Målinger/m-linger/Skjærhallen/20260731_131522" # Høyest bølger, 480 Hz
DEFAULT_PATH = "/home/pif/master/Målinger/m-linger/Skjærhallen/20260731_122652" # Nest høyest bølger, 480 Hz
DEFAULT_PATH = "//home/pif/master/Målinger/m-linger/Skjærhallen/20260731_110314" # Svanekilen, lavest bølger, 480 Hz


#DEFAULT_PATH = "/home/pif/master/Målinger/m-linger/Torrevieja/20260714_084020_tmp" # Nærmere land, bølgebryting
#DEFAULT_PATH = "/home/pif/master/Målinger/m-linger/Torrevieja/20260714_075013_tmp"

# Plott spekteret ved kjøring (praktisk i Spyder). Overstyres med --plot/--no-plot.
PLOT = True
# Fast x-område for plottene (log-akse). Fast, ikke datastyrt, slik at spektre
# fra ulike økter kan legges ved siden av hverandre og sammenlignes direkte.
PLOT_XLIM = (TAPER_F1, 10.0)     # Hz

# Skriv sammenligningen av alle tre lavfrekvens-avskjæringene (utapert / fast
# taper / GPS-utledet f_c) for alle metodene. Overstyres --compare/--no-compare.
COMPARE = True
def vertical_accel(q, ax, ay, az):
    """Vertikal lineær accel (m/s²): roter til verdens-Z, trekk fra g.

    Selve rotasjonen ligger i rotation.world_z; det som gjør denne til et
    DOMENE-uttrykk og ikke ren geometri er fratrekket av g."""
    return world_z(q, ax, ay, az) - GRAVITY
def fill_bucket_gaps(bidx, series, masks, max_fill=GAP_MAX_FILL):
    """Gjør 10 Hz-serien TIDSUNIFORM: sett inn de manglende bøttene mellom
    bidx[0] og bidx[-1] og fyll dem med lineær interpolasjon.

    Uten dette blir et hull på N bøtter til ÉN bøtteovergang, og Welch antar
    likevel eksakt 10 Hz - tidsaksen er da komprimert nettopp der data mangler.

      bidx   = bøtte-indekser (strengt økende, kan ha hull)
      series = dict {navn: liste} med verdier (interpoleres lineært)
      masks  = dict {navn: liste} med 0/1-flagg (fylles konservativt: en fylt
               bøtte arver 1 hvis en av naboene har 1)

    ALLE hull fylles - ellers ville tidsaksen fortsatt vært komprimert. max_fill
    brukes kun til å rapportere hvor mange hull som er lengre enn det man med
    rimelighet kan interpolere over. Returnerer
    (full_idx, series_ut, masks_ut, gap10, stats), der gap10 er 1 for hver
    oppdiktet bøtte slik at welch_psd kan forkaste segmenter som inneholder dem."""
    b = np.asarray(bidx, dtype=np.int64)
    stats = dict(n_gaps=0, n_filled=0, longest=0, n_long=0, span=len(b))
    if len(b) < 2:
        return (b,
                {k: np.asarray(v, dtype=np.float64) for k, v in series.items()},
                {k: np.asarray(v, dtype=np.float64) for k, v in masks.items()},
                np.zeros(len(b), dtype=np.float64), stats)

    d = np.diff(b)
    holes = d[d > 1] - 1
    stats["n_gaps"] = int(len(holes))
    stats["n_filled"] = int(holes.sum()) if len(holes) else 0
    stats["longest"] = int(holes.max()) if len(holes) else 0
    stats["n_long"] = int(np.count_nonzero(holes > max_fill))
    if stats["n_gaps"] == 0:
        return (b,
                {k: np.asarray(v, dtype=np.float64) for k, v in series.items()},
                {k: np.asarray(v, dtype=np.float64) for k, v in masks.items()},
                np.zeros(len(b), dtype=np.float64), stats)

    full = np.arange(b[0], b[-1] + 1, dtype=np.int64)
    xf, xb = full.astype(np.float64), b.astype(np.float64)
    out = {k: np.interp(xf, xb, np.asarray(v, dtype=np.float64))
           for k, v in series.items()}
    # Flagg-serier: interpolér og terskle på > 0, så en fylt bøtte mellom en
    # flagget og en uflagget nabo blir flagget (konservativt - vi vil heller
    # forkaste ett segment for mye enn å ta med forurenset data).
    out_m = {k: (np.interp(xf, xb, np.asarray(v, dtype=np.float64)) > 0.0
                 ).astype(np.float64) for k, v in masks.items()}
    gap = np.ones(len(full), dtype=np.float64)
    gap[b - b[0]] = 0.0
    stats["span"] = int(len(full))
    return full, out, out_m, gap, stats


# --- FIR-lavpass + desimering (samme kjede som sfy4-buoy/src/fir.rs) ----------
def low_freq_taper(f, f1, f2):
    """Kohout/Tucker-Pitt halv-cosinus-taper (amplituderespons)."""
    if f <= f1:
        return 0.0
    if f >= f2:
        return 1.0
    return 0.5 * (1.0 - math.cos(math.pi * (f - f1) / (f2 - f1)))


def find_cutoff(psd_acc, seglen, fmin, distance, prominence, fmax):
    """Auto-detektert cut-off: første spektrale minimum over fmin i det UTAPEREDE
    elevasjonsspekteret S_eta = psd_acc/ω⁴, funnet med SciPy find_peaks på -S
    (minsteavstand i bins + prominens). Prosessen følger Rabault et al. (2022,
    fig. 7), Tucker (1958) og Waseda et al. (2018): minimumet skiller den ω⁻⁴-
    forsterkede lavfrekvente støyhalen fra bølgetoppen.

    Returnerer f_c [Hz], eller None hvis intet minimum ble funnet (da MÅ kalleren
    falle tilbake på fast taper - ellers blir amplitudene ufysisk store)."""
    try:
        from scipy.signal import find_peaks
    except ImportError:
        print("  (scipy ikke installert - kan ikke auto-detektere cut-off; "
              "'pip install scipy')")
        return None
    df = FS / seglen
    k = np.arange(1, seglen // 2 + 1)
    f = k * df
    s_eta = np.asarray(psd_acc)[k] / (2.0 * np.pi * f) ** 4
    sel = (f >= fmin) & (f <= fmax)
    if not np.any(sel):
        return None
    idx, _ = find_peaks(-s_eta[sel], distance=max(1, int(distance)),
                        prominence=prominence)
    if len(idx) == 0:
        return None
    return float(f[sel][idx[0]])


def running_median(x, m):
    """Glidende median over m punkter, klippet mot arrayets ender. Median og ikke
    middel fordi enkeltbins i et Welch-estimat med få segmenter har kraftig
    kjikvadrat-spredning - middelet drar med seg utliggerne, medianen ikke."""
    if m <= 1:
        return np.asarray(x, dtype=np.float64)
    n = len(x)
    h = m // 2
    out = np.empty(n, dtype=np.float64)
    for i in range(n):
        out[i] = np.median(x[max(0, i - h):min(n, i + h + 1)])
    return out


def find_cutoff_gps(psd_acc, psd_vup, seglen, fmin, fmax,
                    rel_factor, smooth_hz, span_hz,
                    ref_quantile=GPS_REF_QUANTILE):
    """GPS-utledet cut-off: sammenlign IMU-ens elevasjonsspekter (psd_acc/ω⁴) med
    GPS-ens (psd_vup/ω²) og finn KNEKKPUNKTET i forholdskurven.

    Under f_c er IMU-en dominert av orienteringsfeil som ω⁻⁴ blåser opp; GPS-en
    ser ikke den feilen og gir derfor et uavhengig gulv. Over ~0.3 Hz stiger
    forholdet igjen fordi GPS-en under-leser chop - kurven har altså et minimum
    i midten, og det er lavfrekvenssiden av det minimumet vi er ute etter.

    Kriteriet er RELATIVT og oppgitt i Hz:
      1. glidende median over `smooth_hz` (demper kjikvadrat-spredningen),
      2. terskel = `rel_factor` x `ref_quantile`-persentilen av kurven i
         [fmin, fmax] (lav kvantil, ikke minimum - se konstantene),
      3. f_c = laveste frekvens der den glattede kurven holder seg under
         terskelen sammenhengende over `span_hz`.
    En ABSOLUTT terskel duger ikke: kurvens minimum ligger rundt 2.0-2.4, så en
    grense på 2 avgjøres av bin-støy og f_c vandrer med seglen (se kommentaren
    ved GPS_REL_FACTOR).

    BEGGE spektrene er UTAPEREDE her, og det er et krav: en taper ville nullet
    ut nettopp båndet vi leter i, og forholdet ble 0/0 under f1. Taperen påføres
    først etterpå, når momentene integreres. f_c er altså utledet av rådataene,
    ikke av en allerede avskåret serie.

    Returnerer (fc, glattet ratio-array, f-array, kurvens minimum). fc=None hvis
    kravet aldri oppfylles i [fmin, fmax]."""
    df = FS / seglen
    k = np.arange(1, seglen // 2 + 1)
    f = k * df
    w = 2.0 * np.pi * f
    eta_imu = np.asarray(psd_acc)[k] / w ** 4
    eta_gps = np.asarray(psd_vup)[k] / w ** 2
    with np.errstate(divide="ignore", invalid="ignore"):
        rat = np.where(eta_gps > 0.0, eta_imu / eta_gps, np.inf)

    # 1) Glatting over et fast FREKVENSbånd, ikke et fast antall bins - ellers
    #    betyr kriteriet noe helt annet ved seglen 512 enn ved 8192.
    m = max(1, int(round(smooth_hz / df)))
    if m % 2 == 0:
        m += 1
    rat_s = running_median(rat, m)

    band = (f >= fmin) & (f <= fmax)
    vals = rat_s[band]
    vals = vals[np.isfinite(vals)]
    if len(vals) == 0:
        return None, rat_s, f, None

    # 2) Terskel relativt til en lav KVANTIL av kurven - ikke minimum, som synker
    #    med segmentantallet og dermed lar terskelen følge støyen (se konstantene).
    rmin = float(np.percentile(vals, ref_quantile))
    thr = rel_factor * rmin

    # 3) Sammenhengende under terskelen over span_hz.
    nspan = max(1, int(round(span_hz / df)))
    ok = rat_s <= thr
    for j in np.where(band)[0]:
        if j + nspan <= len(ok) and ok[j:j + nspan].all():
            return float(f[j]), rat_s, f, rmin
    return None, rat_s, f, rmin


def gps_snr_at(psd_vup, seglen, f, noise):
    """GPS-ens signal/støy-forhold i bin nærmest f: S_vUp(f) / eget støygulv.
    Brukes til å sjekke at GPS-referansen fortsatt er gyldig der f_c havnet -
    er den ikke det, sammenligner IMU/GPS-kriteriet støy mot støy."""
    if f is None or noise is None or noise <= 0.0:
        return None
    k = int(round(f / (FS / seglen)))
    if k < 1 or k > seglen // 2:
        return None
    return float(np.asarray(psd_vup)[k] / noise)


def gps_noise_floor(psd_vup, seglen, band=GPS_NOISE_BAND):
    """GPS-ens eget hvitstøy-gulv: median-PSD av vUp i et bånd over bølgene.
    Brukes kun til rapportering - sier hvor langt ned GPS-fasiten er troverdig."""
    df = FS / seglen
    f = np.arange(1, seglen // 2 + 1) * df
    sel = (f >= band[0]) & (f <= band[1])
    if not np.any(sel):
        return 0.0
    return float(np.median(np.asarray(psd_vup)[1:seglen // 2 + 1][sel]))


def peak_frequency(psd_acc, seglen, fmin, fmax):
    """Frekvensen der det UTAPEREDE elevasjonsspekteret er størst i [fmin,fmax].
    Brukes kun som sanity-sjekk: et gyldig f_c skal ligge UNDER spektraltoppen."""
    df = FS / seglen
    k = np.arange(1, seglen // 2 + 1)
    f = k * df
    s_eta = np.asarray(psd_acc)[k] / (2.0 * np.pi * f) ** 4
    sel = (f >= fmin) & (f <= fmax)
    if not np.any(sel):
        return None
    return float(f[sel][int(np.argmax(s_eta[sel]))])


def make_taper(seglen, f1, f2, fc=None):
    """Amplituderespons T[k] for k = 0..N/2.
    fc=None  -> Kohout/Tucker-Pitt halv-cosinus mellom f1 og f2.
    fc satt  -> hard avskjæring (T=0 under fc, 1 over), som i auto-metoden."""
    df = FS / seglen
    f = np.arange(seglen // 2 + 1) * df
    if fc is not None:
        return (f >= fc).astype(np.float64)
    return np.array([low_freq_taper(x, f1, f2) for x in f], dtype=np.float64)


def wave_params(psd, seglen, fmax, taper):
    """Elevasjon-PSD = psd_acc/ω⁴ · T², momenter opp til fmax, Hs/Tz/Tc.
    taper = amplituderespons T[k] fra make_taper.
    Returnerer dict med hs/tz/tc/m0/m2/m4 (-1 der udefinert)."""
    n = seglen
    df = FS / n
    m0 = m2 = m4 = 0.0
    for k in range(1, n // 2 + 1):
        f = k * df
        if f > fmax:
            break
        t = taper[k]
        if t <= 0.0:
            continue
        w = 2.0 * math.pi * f
        psd_eta = psd[k] / (w ** 4) * (t * t)
        m0 += psd_eta * df
        m2 += psd_eta * f * f * df
        m4 += psd_eta * f * f * f * f * df
    hs = 4.0 * math.sqrt(m0) if m0 > 0 else -1.0
    tz = math.sqrt(m0 / m2) if (m0 > 0 and m2 > 0) else -1.0
    tc = math.sqrt(m2 / m4) if (m2 > 0 and m4 > 0) else -1.0
    return dict(hs=hs, tz=tz, tc=tc, m0=m0, m2=m2, m4=m4)


def ahrs_fra_cfg(orientation_name):
    """cfg.csv orientation_name -> vår metodenøkkel.

    Firmware skriver bare det WaveAhrs::kName gir: "Madgwick", "Kalman" eller
    "SFLP". nxp og mekf kan derfor aldri komme herfra - de er alternativer man
    ber om. "Kalman" er kalman.py og IKKE mekf.py: de to er samme algoritme, og
    forskjellen mellom dem er tuningen i wave_config.h, som er nettopp det
    MEKF-kolonnen finnes for å vise.

    Delt med raw_to_csv.Params.ahrs_navn - regelen skal stå ett sted, ellers kan
    rekonstruksjonen og analysen ende opp med å mene ulike ting om samme fangst."""
    n = (orientation_name or "").strip().lower()
    return "kalman" if n.startswith("kalman") else "madgwick"


# --- Innlesing (med samme "_tmp"-hale-vern som postProcess) -------------------
def resolve_paths(path):
    """Finn imu.csv + stamp + øktkatalog fra en katalog- eller filsti."""
    if os.path.isdir(path):
        cand = sorted(glob(os.path.join(path, "*_imu.csv")))
        if not cand:
            sys.exit(f"Fant ingen *_imu.csv i {path}")
        imu = cand[0]
    elif os.path.isfile(path):
        imu = path
    else:
        sys.exit(f"Fant ikke {path}")
    directory = os.path.dirname(os.path.abspath(imu))
    stamp = os.path.basename(imu)[:-len("_imu.csv")]
    return imu, stamp, directory


def imu_col_names(idx):
    """Kolonnenavnene i imu.csv ble lagt om ved build_seq 3: SFLP-kanalene fikk
    _sflp-suffiks, og det USUFFIKSERTE quaternion-settet er nå det VALGTE filteret
    (het mqw..mqz før). Rekkefølgen på kolonnene er den samme.

    'mqw' finnes bare i gamle filer og er derfor diskriminanten - vi kan ikke bare
    slå opp 'qw', for det navnet finnes i begge og betyr to ulike ting."""
    legacy = "mqw" in idx
    return dict(
        azn="az_ned" if legacy else "az_ned_sflp",
        q_sflp=(("qw", "qx", "qy", "qz") if legacy
                else ("qw_sflp", "qx_sflp", "qy_sflp", "qz_sflp")),
        vacc="vacc_madgwick" if legacy else "vacc",
    )


# Hvor mye av en strømmefil som var fangstens egen. grunn er None når fila ble
# lest helt ut - da er de tre tallene bare bokføring.
CsvKutt = namedtuple("CsvKutt", "rader lest total grunn")


def _kutt_melding(path, k):
    return (f"  {os.path.basename(path)}: {k.rader} hele rader, "
            f"{k.total - k.lest} B hale forkastet ({k.grunn})")


def read_csv_stream(path, time_col):
    """Les en strømmet CSV (imu.csv / gps.csv) -> (idx, rader, CsvKutt).

    DEN ENE PLASSEN hale-regelen står. Alle leserne under - og rawplot.py -
    bruker denne, for regelen må ikke kunne gli fra hverandre mellom dem: to
    verktøy som er uenige om hvor dataene slutter, gir to ulike Hs av samme økt.

    En avbrutt fangst rekker aldri truncate(), så fila står igjen på hele den
    preallokerte lengden, og alt etter siste skrevne rad er GAMMELT KORTINNHOLD -
    binærsøppel fra filene som lå i de klyngene før. Derfor leses fila binært:
    i tekstmodus dør lesingen på UnicodeDecodeError i den halen, altså FØR
    hale-regelen under i det hele tatt får se en rad.

    Tre ting avslutter dataene, og rekkefølgen er som den er fordi den billigste
    sjekken tar de fleste tilfellene:
      - feil antall felt   (raden ble kuttet midt i, eller det er ikke CSV)
      - tid som ikke øker  (halen er en ELDRE kopi av samme fil - se rad 7483 i
                            20260814_053720_gps.csv, der siste SD-blokk lå igjen
                            i en utdatert utgave og hoppet 800 ms bakover)
      - felt som ikke er tall

    Radene kommer som lister av strenger; kallerne konverterer selv de
    kolonnene de trenger."""
    with open(path, "rb") as f:
        data = f.read()
    linjer = data.split(b"\n")
    header = [c.decode("ascii", "replace") for c in linjer[0].rstrip(b"\r").split(b",")]
    idx = {nm: i for i, nm in enumerate(header)}
    if time_col not in idx:
        return idx, [], CsvKutt(0, len(linjer[0]) + 1, len(data),
                                f"mangler kolonnen '{time_col}'")

    rader, brukt, forrige, grunn = [], len(linjer[0]) + 1, None, None
    for ln in linjer[1:]:
        if not ln.strip():
            continue                       # tom linje (siste \n) - ikke en hale
        fld = ln.rstrip(b"\r").split(b",")
        if len(fld) != len(header):
            grunn = "ufullstendig rad"
            break
        try:
            t = float(fld[idx[time_col]])
            rad = [v.decode("ascii") for v in fld]
        except (ValueError, UnicodeDecodeError):
            grunn = "ikke-numerisk felt"
            break
        if forrige is not None and t <= forrige:
            grunn = f"tid gikk ikke fram ({t:.0f} etter {forrige:.0f})"
            break
        forrige = t
        rader.append(rad)
        brukt += len(ln) + 1
    return idx, rader, CsvKutt(len(rader), brukt, len(data), grunn)


def read_imu_rows(imu_path, meta=None):
    """Les imu.csv -> liste av dict per rad. Hale-regelen ligger i
    read_csv_stream; her er bare kolonnene.

    meta: valgfri dict som fylles med opplysninger om FILA, ikke om radene -
    foreløpig hvilken kolonne vacc_raw ble tatt fra. Den må ut, for navnet
    varierer med filversjonen, og en melding om «vacc_fir» ville vært direkte
    feil på en økt der fallbacken ble brukt."""
    idx, rader, kutt = read_csv_stream(imu_path, "win_start_ms")
    cols = imu_col_names(idx)
    need = ["win_start_ms", "ax_mg", "ay_mg", "az_mg", cols["azn"],
            "gx_mdps", "gy_mdps", "gz_mdps", "braking"]
    for nm in need:
        if nm not in idx:
            sys.exit(f"Mangler kolonne '{nm}' i {imu_path}")
    has_q = all(c in idx for c in cols["q_sflp"])
    # vacc_fir er den FIR-desimerte serien og førstevalget; den beholdt navnet
    # sitt gjennom omdøpingen ved build_seq 3. Fallbacken er den ufiltrerte
    # senter-tapp-kolonnen, som heter ulikt før/etter - derav oppslaget.
    raw_col = idx.get("vacc_fir", idx.get(cols["vacc"]))
    if meta is not None:
        meta["vacc_col"] = ("vacc_fir" if "vacc_fir" in idx
                            else cols["vacc"] if raw_col is not None else None)
    rows = []
    if kutt.grunn:
        print(_kutt_melding(imu_path, kutt))
    for fields in rader:
        rows.append(dict(
            t=int(fields[idx["win_start_ms"]]),
            ax=float(fields[idx["ax_mg"]]),
            ay=float(fields[idx["ay_mg"]]),
            az=float(fields[idx["az_mg"]]),
            azn=float(fields[idx[cols["azn"]]]),
            gx=float(fields[idx["gx_mdps"]]),
            gy=float(fields[idx["gy_mdps"]]),
            gz=float(fields[idx["gz_mdps"]]),
            braking=1 if fields[idx["braking"]].strip() == "1" else 0,
            # fifo_ovf kom i build 55. Eldre økter mangler kolonnen -> 0,
            # så gamle filer fortsatt leses uendret.
            ovf=(1 if "fifo_ovf" in idx
                 and fields[idx["fifo_ovf"]].strip() == "1" else 0),
            # SFLP-quaternionen (on-chip). Brukes KUN til tilt-diagnostikken -
            # vertikal-accelen tas fortsatt fra az_ned_sflp, som før. None hvis
            # kolonnene mangler (eldre økter).
            q=(tuple(float(fields[idx[c]]) for c in cols["q_sflp"])
               if has_q else None),
            # Vertikal-accelen som ALT er regnet ut, av AHRS-en på råstrømmen -
            # om bord ved kAhrsRateHz, eller av raw_to_csv fra raw.bin. Kolonnen
            # er FIR-desimert til radrutenettet, altså samme trinn 1 som ax_mg og
            # resten av radene, og er dermed direkte sammenlignbar med det
            # replayet under regner ut. --vacc-source velger hvilken som brukes;
            # None her betyr at valget ikke er mulig for denne fila.
            vacc_raw=(float(fields[raw_col]) if raw_col is not None else None),
            # Antall RÅSAMPLES bak raden. Den eneste kilden i imu.csv til hva
            # sensoren faktisk leverte - resten av kolonnene er desimert, og
            # sier derfor bare radraten. Se faktisk_odr().
            n=(int(fields[idx["n"]]) if "n" in idx else None),
        ))
    return rows


RAW_TO_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "raw_to_csv.py")


def finn_raalogg(directory, stamp, override=None):
    """<stamp>_raw.bin, eller None. "off" slår av oppslaget.

    Ett nivå opp er med fordi rawplot legger imu.csv i <økt>/<stamp>_raw/ mens
    raw.bin blir liggende i øktkatalogen - kopiere den ville vært 19 MB for å
    slippe et katalogoppslag."""
    if override == "off":
        return None
    if override:
        if not os.path.isfile(override):
            sys.exit(f"--raw {override}: fant ikke fila")
        return override
    for kandidat in (os.path.join(directory, f"{stamp}_raw.bin"),
                     os.path.join(os.path.dirname(directory), f"{stamp}_raw.bin")):
        if os.path.isfile(kandidat):
            return kandidat
    return None


def lag_raw_csv(raw_bin, ut_fil, metode):
    """Kjør raw_to_csv for ett filter på råstrømmen -> <stamp>_imu_raw_<f>.csv.

    Delprosess og ikke import: raw_to_csv importerer denne modulen, så veien går
    bare én vei. Filene er dessuten mellomlagre - de skal kunne lages, leses og
    slettes uavhengig av hvem som kjører."""
    print(f"  raw:   kjører {metode} på råstrømmen -> "
          f"{os.path.basename(ut_fil)} ...", flush=True)
    t0 = time.time()
    res = subprocess.run([sys.executable, RAW_TO_CSV, raw_bin, "--mode", "imu",
                          "--ahrs", metode, "-o", ut_fil],
                         capture_output=True, text=True)
    if res.returncode != 0:
        hale = "\n".join((res.stdout + res.stderr).strip().splitlines()[-15:])
        sys.exit(f"raw_to_csv.py feilet for --ahrs {metode} "
                 f"(exit {res.returncode}):\n{hale}")
    print(f"         ferdig på {time.time() - t0:.0f} s")


def les_raw_vacc(path, rows):
    """vacc_fir fra en søskenfil, stilt opp mot radene -> (array, n_mangler).

    Oppstillingen går på win_start_ms og ikke på posisjon: filene er laget av
    samme rålogg med samme rutenett, men kantene kan avvike med en rad, og en
    forskyvning her ville vært usynlig i tallene og likevel gale. Rader uten
    treff blir NaN, og kalleren lar replayet stå der."""
    idx, rader, kutt = read_csv_stream(path, "win_start_ms")
    if "vacc_fir" not in idx:
        sys.exit(f"{os.path.basename(path)} mangler vacc_fir - er den laget av "
                 f"en eldre raw_to_csv?")
    if kutt.grunn:
        print(_kutt_melding(path, kutt))
    tabell = {int(r[idx["win_start_ms"]]): float(r[idx["vacc_fir"]]) for r in rader}
    ut = np.array([tabell.get(r["t"], np.nan) for r in rows], dtype=np.float64)
    return ut, int(np.isnan(ut).sum())


def bygg_raa_serier(directory, stamp, rows, cfg_ahrs, raw_bin, vacc_source,
                    regen=False, har_kolonne=False):
    """Vertikal-accel fra råstrømmen for hver AHRS-metode -> (serier, kilder).

    serier: {intern nøkkel: array på radaksen} for de metodene som fikk en.
    kilder: {metodenavn: "csv" | "raw:<fil>"} for ALLE metodene - også de som
    endte på replay, for det er den opplysningen som gjør tabellen lesbar.

    Metoden fangsten selv kjørte står ikke her: den ligger allerede som vacc_fir
    i imu.csv, og run() tar den derfra. Å lage en søskenfil for den ville vært en
    kopi av den største fila i mappa."""
    # SFLP står oppført selv om den aldri kjøres her: den kommer ferdig fra
    # brikken, og "on-chip" er et tredje svar - ikke et replay vi valgte bort.
    serier, kilder = {}, {"sflp": "onchip"}
    for navn, key in AHRS_METHODS:
        if navn == cfg_ahrs and har_kolonne and vacc_source != "csv":
            kilder[navn] = "raw:imu.csv"          # run() henter den fra radene
            continue
        sti = os.path.join(directory, f"{stamp}_imu_raw_{navn}.csv")
        if vacc_source == "csv":
            kilder[navn] = "csv"
            continue
        if regen or not os.path.isfile(sti):
            if raw_bin is None:
                if vacc_source == "raw":
                    sys.exit(f"--vacc-source raw: fant ingen {stamp}_raw.bin, og "
                             f"{os.path.basename(sti)} finnes ikke. Uten råloggen "
                             f"kan {navn} bare replayes - bruk auto eller csv.")
                kilder[navn] = "csv"
                continue
            lag_raw_csv(raw_bin, sti, navn)
        arr, mangler = les_raw_vacc(sti, rows)
        if mangler:
            print(f"  raw:   {mangler} av {len(rows)} rader uten treff i "
                  f"{os.path.basename(sti)} - de beholder replay-verdien")
        serier[key] = arr
        kilder[navn] = f"raw:{os.path.basename(sti)}"
    return serier, kilder


def faktisk_odr(rows):
    """Sensorens FAKTISKE samplerate -> Hz, eller None.

    Summen av n-kolonnen er antall råsamples fangsten virkelig fikk; delt på
    tida de dekker gir det raten sensoren leverte, ikke den som ble bedt om.
    De to er ikke like: 20260813 ble satt til 480 Hz og målte 464, fordi
    FIFO-dreneringen ikke alltid rekker rundt. Det avviket er usynlig i alle de
    andre kolonnene i imu.csv - de er desimert til radrutenettet uansett hva
    sensoren gjorde.

    Tidsspennet er (siste - første) PLUSS én radperiode: n-ene teller samples i
    hele den siste raden også, og uten det siste leddet ville en kort fangst
    fått en for høy rate."""
    n = [r["n"] for r in rows if r.get("n") is not None]
    if len(n) < 2 or len(n) != len(rows):
        return None
    t = np.array([r["t"] for r in rows], dtype=np.float64)
    dt = np.diff(t)
    dt = dt[dt > 0]
    if not len(dt):
        return None
    span_ms = (t[-1] - t[0]) + float(np.median(dt))
    return float(sum(n)) / (span_ms / 1000.0) if span_ms > 0 else None


def metode_rater(cfg, rows, kilder):
    """Raten hver metode FAKTISK kjørte på -> {navn: Hz}.

    Dette er ikke pynt til figuren: en Hs fra et filter på 480 Hz og en fra det
    samme filteret replayet på 100 Hz er to ulike tall, og uten raten ved siden
    av ser tabellen ut som en sammenligning av filtre når den delvis er en
    sammenligning av rater."""
    def tall(nokkel):
        try:
            v = float(cfg.get(nokkel, ""))
            return v if v > 0 else None
        except ValueError:
            return None

    imu_odr = tall("imu_odr_hz")
    sflp_odr = tall("sflp_odr_hz")
    # Radraten: cfg først, ellers det radene selv sier (eldre økter uten window_ms).
    rad_hz = tall("output_rate_hz")
    if rad_hz is None and len(rows) > 1:
        d = np.diff(np.array([r["t"] for r in rows], dtype=np.float64))
        d = d[d > 0]
        rad_hz = 1000.0 / float(np.median(d)) if len(d) else None

    rater = {}
    for navn, _ in METHODS:
        if navn == "sflp":
            rater[navn] = sflp_odr        # on-chip-fusjonen, ikke råraten
        elif str(kilder.get(navn, "csv")).startswith("raw"):
            rater[navn] = imu_odr         # filteret kjørte per råsample
        else:
            rater[navn] = rad_hz
    return rater


def read_gps_vup(gps_path):
    """Les gps.csv -> (t_s, vUp) som numpy-arrays. rel_ms ligger på samme
    millis-akse som win_start_ms, så tidene er direkte sammenlignbare.
    Returnerer None hvis fila eller kolonnene mangler."""
    if not os.path.isfile(gps_path):
        return None
    idx, rader, kutt = read_csv_stream(gps_path, "rel_ms")
    if "vUp" not in idx:
        return None
    if kutt.grunn:
        print(_kutt_melding(gps_path, kutt))
    ts = np.array([float(r[idx["rel_ms"]]) / 1000.0 for r in rader])
    vs = np.array([float(r[idx["vUp"]]) for r in rader])
    if len(ts) < 2:
        return None
    return ts, vs


def read_kv(path):
    """Les en enkel key,value-fil (ses.csv / cfg.csv) til dict."""
    out = {}
    if not os.path.isfile(path):
        return out
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split(",", 1)
            if len(parts) == 2:
                out[parts[0]] = parts[1]
    return out


# --- Pipeline (speiler StreamAnalyzer::ingest + finalize) --------------------
def run(rows, seglen, overlap_div, window_kind, fmax, f1, f2, beta, brake_reject=0.0,
        ovf_reject=OVF_REJECT,
        cutoff_mode="taper", cut_fmin=CUT_FMIN, cut_distance=CUT_DISTANCE,
        cut_prominence=CUT_PROMINENCE, gps=None, gps_rel=GPS_REL_FACTOR,
        gps_smooth_hz=GPS_SMOOTH_HZ, gps_span_hz=GPS_SPAN_HZ,
        gps_own_prom=GPS_OWN_PROMINENCE, gps_ramp=GPS_RAMP,
        gps_max_f1=GPS_MAX_F1, gps_fmax=GPS_SEARCH_FMAX,
        detrend=DETREND, gap_reject=GAP_REJECT,
        noise_band=NOISE_BAND,
        decimate_mode=DECIMATE_MODE, fir_ntap=FIR_NTAP, fir_cutoff=FIR_CUTOFF,
        fir_compensate=FIR_COMPENSATE_DELAY, skip_start_s=0.0,
        vacc_source=VACC_SOURCE, raw_ahrs="madgwick", raw_col="vacc_fir",
        raw_series=None):
    # AHRS-ene har identisk grensesnitt (init_from_accel/update/.q), så løkka
    # under behandler dem likt - forskjellen ligger kun inne i modulene.
    madg = Madgwick(beta=beta)
    kal = Kalman()
    # Bøyas eget filter, med wave_config.h-verdiene som mekf.py holder. Ingen
    # parameter herfra: skal den sammenlignes med drifterens egne tall, må den
    # kjøre drifterens egen tuning.
    mek = Mekf()
    # NXP-filteret er unntaket: det tar IKKE dt per steg, men baker inn 1/fs ved
    # konstruksjon (se FAST RATE i kalman_nxp.py). Raten må derfor bestemmes her.
    # RAW_DT_MS er logge-perioden fra cfg.csv; mangler den (eldre økter), brukes
    # medianavstanden mellom radene - som er nettopp den dt de andre filtrene får
    # servert. Median og ikke middel, så et enkelt hull ikke drar raten.
    dts = np.diff(np.asarray([r["t"] for r in rows], dtype=np.float64))
    dts = dts[dts > 0.0]
    if not len(dts):
        raise ValueError("kan ikke utlede rad-raten til NXP-filteret: "
                         "for få rader med økende tid")
    obs_dt_ms = float(np.median(dts))
    nxp_dt_ms = RAW_DT_MS or obs_dt_ms
    # Feil rate er IKKE en mild feil her: den skalerer gyro-integrasjonen, og
    # Hs_nxp målt på 110314 går fra 0.098 m ved riktig rate til 26.8 m ved fjerde-
    # delen av den (se tabellen i kalman_nxp.py). Et avvik mellom cfg.csv og den
    # faktiske radavstanden må derfor ikke passere stille - da er ett av de to
    # tallene galt, og vi kan ikke vite hvilket.
    if RAW_DT_MS and abs(RAW_DT_MS - obs_dt_ms) > 0.05 * obs_dt_ms:
        print(f"  ADVARSEL: cfg.csv oppgir radperiode {RAW_DT_MS:g} ms, men "
              f"radene ligger {obs_dt_ms:g} ms fra hverandre - NXP-filteret har "
              f"fast tidssteg og blir feil hvis cfg.csv ikke stemmer")
    nxp = KalmanNxp(fs=1000.0 / nxp_dt_ms)

    # Kildevalget for vertikal-accelen (se VACC_SOURCE). Det avgjøres HER, én
    # gang, og resultatet følger med ut i res: en Hs kan ikke leses uten å vite
    # hvilken av de to kjedene den kom fra.
    har_raw = bool(rows) and rows[0].get("vacc_raw") is not None
    if vacc_source == "raw" and not har_raw:
        sys.exit("--vacc-source raw: imu.csv har verken vacc_fir eller vacc. "
                 "Den kolonnen finnes bare i økter logget med rålogg eller "
                 "WaveLogMode::Both - bruk --vacc-source csv (eller auto).")
    bruk_raw = har_raw and vacc_source in ("auto", "raw")
    # Hvilken metode kolonnen ER. Den ble laget av filteret fangsten kjørte, så
    # den hører hjemme i den radens plass - ikke i Madgwick-raden uansett.
    raw_slot = {"madgwick": "m", "kalman": "k"}.get(str(raw_ahrs).lower())
    if bruk_raw and raw_slot is None:
        print(f"  ADVARSEL: cfg.csv oppgir AHRS '{raw_ahrs}', som ikke har en "
              f"egen rad her - vacc_fir legges i Madgwick-raden.")
        raw_slot = "m"
    if not bruk_raw:
        raw_slot = None

    have_q = False
    prev_t = 0

    # 10 Hz-bøtting (én akkumulator per metode: M=Madgwick, S=SFLP, K=Kalman,
    # N=NXP, E=MEKF).
    cur_bucket = -1
    bsum_m = bsum_s = bsum_k = bsum_n = bsum_e = 0.0
    bn = 0
    bbrake = 0                       # brems-rader i inneværende bøtte
    bovf = 0                         # fifo-overflow-rader i inneværende bøtte
    vm10, vs10, vk10, vn10, ve10 = [], [], [], [], []
    brake10 = []                     # 1 per bøtte hvis noen rad i den var brems
    ovf10 = []                       # 1 per bøtte hvis noen rad i den hadde overflow
    bidx10 = []                      # bøtte-indeks -> tidsakse for GPS-samstilling
    n_brake = 0
    n_ovf = 0
    have_sflp_q = any(r.get("q") is not None for r in rows[:1])
    # FIR-modus trenger seriene på RÅ rate (bøtte-midlet er nettopp det filteret vi
    # vil bytte ut), så de samles opp her og desimeres etter løkka. Én tuple per rad
    # i stedet for 17 append-kall, og kun når modusen faktisk er på.
    collect_rows = decimate_mode == "fir"
    ROW_KEYS = ("m", "s", "k", "n", "e")
    ANGLE_KEYS = ()
    rowvals = []

    def flush_bucket():
        nonlocal bsum_m, bsum_s, bsum_k, bsum_n, bsum_e, bn, bbrake, bovf
        if bn > 0:
            vm10.append(bsum_m / bn)
            vs10.append(bsum_s / bn)
            vk10.append(bsum_k / bn)
            vn10.append(bsum_n / bn)
            ve10.append(bsum_e / bn)
            brake10.append(1 if bbrake > 0 else 0)
            ovf10.append(1 if bovf > 0 else 0)
            bidx10.append(cur_bucket)   # bøtta som nå lukkes
        bsum_m = bsum_s = bsum_k = bsum_n = bsum_e = 0.0
        bn = 0
        bbrake = 0
        bovf = 0

    # Orientering per rad (tredje rad av R) - trengs til "after"-korreksjonen og
    # til tilpasningen. Lagres som quaternioner og vektoriseres etter løkka.

    for i, r in enumerate(rows):
        if r["braking"]:
            n_brake += 1
        if r["ovf"]:
            n_ovf += 1
        t = r["t"]
        # AHRS-ene mates i m/s², ikke mg: Kalman-ESKF-ens adaptive R sammenligner
        # |a| mot g og trenger derfor ekte enheter. Madgwick normaliserer og er
        # skala-invariant (verifisert til 1.7e-16), så den ser ingen forskjell.
        axm = r["ax"] * MG2MS2
        aym = r["ay"] * MG2MS2
        azm = r["az"] * MG2MS2
        gx = r["gx"] * MDPS2RADS
        gy = r["gy"] * MDPS2RADS
        gz = r["gz"] * MDPS2RADS
        dt = (t - prev_t) / 1000.0 if have_q else 0.0

        if not have_q:
            madg.init_from_accel(axm, aym, azm)
            kal.init_from_accel(axm, aym, azm)
            nxp.init_from_accel(axm, aym, azm)
            mek.init_from_accel(axm, aym, azm)
            have_q = True
        elif dt > 0.0:
            madg.update(gx, gy, gz, axm, aym, azm, dt)
            kal.update(gx, gy, gz, axm, aym, azm, dt)
            nxp.update(gx, gy, gz, axm, aym, azm, dt)
            mek.update(gx, gy, gz, axm, aym, azm, dt)
        prev_t = t

        vm = vertical_accel(madg.q, axm, aym, azm)
        vs = r["azn"] * MG2MS2
        vk = vertical_accel(kal.q, axm, aym, azm)
        # MEKF: om bord mates den med RÅ mg (wave_analysis.cpp sender r.ax
        # uskalert), her med m/s². Den forskjellen er null - accelvektoren
        # normaliseres inne i filteret, så bare retningen brukes.
        ve = vertical_accel(mek.q, axm, aym, azm)
        # Samme uttrykk for NXP: quaternionen er body -> verden i samme
        # konvensjon (verifisert, se kalman_nxp.py), så ingen konjugering. Dette
        # er nøyaktig det buf.rs gjør med q.rotate(axl) og fratrekk av g.
        vn = vertical_accel(nxp.q, axm, aym, azm)
        # Kildebyttet står her, etter at replayet er regnet ut: AHRS-ene skal
        # kjøre uansett - konvergensen deres er den samme uansett hvilken serie
        # som går videre - det er kun hvilken verdi som brukes som endres.
        #
        # To kilder, i denne rekkefølgen: fangstens EGET filter ligger som
        # vacc_fir i imu.csv (raw_slot), mens de øvrige filtrene kommer fra hver
        # sin søskenfil (raw_series). De overlapper ikke, men søskenfila vinner
        # om de skulle gjøre det: den er eksplisitt bedt om.
        if raw_slot == "m":
            vm = r["vacc_raw"]
        elif raw_slot == "k":
            vk = r["vacc_raw"]
        if raw_series:
            # NaN = ingen rad med denne tida i søskenfila; da står replayet.
            v = raw_series.get("m")
            if v is not None and math.isfinite(v[i]):
                vm = v[i]
            v = raw_series.get("k")
            if v is not None and math.isfinite(v[i]):
                vk = v[i]
            v = raw_series.get("n")
            if v is not None and math.isfinite(v[i]):
                vn = v[i]
            v = raw_series.get("e")
            if v is not None and math.isfinite(v[i]):
                ve = v[i]

        if not math.isfinite(vm):
            vm = 0.0
        if not math.isfinite(vs):
            vs = 0.0
        if not math.isfinite(vk):
            vk = 0.0
        if not math.isfinite(vn):
            vn = 0.0
        if not math.isfinite(ve):
            ve = 0.0

        bucket = t // BUCKET_MS
        if bucket != cur_bucket:
            flush_bucket()
            cur_bucket = bucket
        bsum_m += vm
        bsum_s += vs
        bsum_k += vk
        bsum_n += vn
        bsum_e += ve
        if collect_rows:                       # rekkefølge = ROW_KEYS
            rowvals.append((vm, vs, vk, vn, ve))
        if r["braking"]:
            bbrake += 1
        if r["ovf"]:
            bovf += 1
        bn += 1
    flush_bucket()  # siste delvise bøtte

    # Gjør serien tidsuniform FØR Welch: uten dette blir et hull på N bøtter til
    # én bøtteovergang, mens df = fs/N fortsatt antar eksakt 10 Hz.
    full_idx, ser, msk, gap10, gapstats = fill_bucket_gaps(
        bidx10,
        dict({"m": vm10, "s": vs10, "k": vk10, "n": vn10, "e": ve10}),
        {"brake": brake10, "ovf": ovf10})

    # FIR-modus: kast bøtte-midlet og bygg de samme seriene med lavpass +
    # desimering i stedet, levert på NØYAKTIG samme bøtteakse (full_idx). Alt
    # nedenfor - Welch, GPS-samstilling, brems/overflow-masker - ser
    # derfor ingen forskjell utover at antialias-filteret er et annet.
    firstats = None
    if decimate_mode == "fir":
        rowmat = np.asarray(rowvals, dtype=np.float64)
        rser = {k: rowmat[:, i] for i, k in enumerate(ROW_KEYS)}
        fser, fgap, firstats = fir.fir_decimate_series(
            np.array([r["t"] for r in rows], dtype=np.float64), rser, full_idx,
            BUCKET_MS, FS, ntap=fir_ntap, cutoff=fir_cutoff, unwrap=ANGLE_KEYS,
            compensate=fir_compensate, raw_dt_ms=RAW_DT_MS)
        ser.update(fser)
        # Filterets kanter og hull-smøring legges til hull-masken, ikke over den:
        # bøtter som ALLEREDE var oppdiktet skal fortsatt være flagget.
        gap10 = np.maximum(gap10, fgap)

    vm10, vs10, vk10, vn10, ve10 = (ser["m"], ser["s"], ser["k"], ser["n"],
                                    ser["e"])
    brake10, ovf10 = msk["brake"], msk["ovf"]
    win = window_weights(seglen, window_kind)

    # Oppstart: flagg bøttene før skip_start_s som "start", med terskel 0.0, så et
    # segment forkastes hvis det så mye som berører dem.
    #
    # Dette gjøres bevisst som et FORKASTINGSKRITERIUM og ikke ved å kutte radene:
    # AHRS-ene (Madgwick/Kalman) har allerede kjørt over hele økta på dette
    # punktet, så de er ferdig konvergert når det første godkjente segmentet
    # starter. Kuttet man radene i stedet, ville filtrene startet kaldt ved
    # skip_start_s - altså fått konvergenstransienten rett inn i analysebåndet,
    # som er det motsatte av hensikten. Bøtteaksen (full_idx) er absolutt fra
    # øktstart, så GPS-interpolasjonen treffer fortsatt samme tidspunkter.
    start10 = None
    if skip_start_s > 0.0:
        start10 = (np.asarray(full_idx, dtype=np.float64) * BUCKET_MS / 1000.0
                   < skip_start_s).astype(np.float64)

    # Samme forkastingskriterier på ALLE seriene, ellers ville IMU og GPS
    # vært midlet over ulike deler av økta og bin-for-bin-forholdet ugyldig.
    rejects = [("start", start10, 0.0 if start10 is not None else None),
               ("brems", brake10, brake_reject if brake_reject > 0.0 else None),
               ("fifo_ovf", ovf10, ovf_reject),
               ("hull", gap10, gap_reject if gap_reject is not None else None)]
    psd_m, nseg, nrej = welch_psd(vm10, seglen, overlap_div, win, FS, rejects, detrend)
    psd_s, _, _ = welch_psd(vs10, seglen, overlap_div, win, FS, rejects, detrend)
    psd_k, _, _ = welch_psd(vk10, seglen, overlap_div, win, FS, rejects, detrend)
    psd_n, _, _ = welch_psd(vn10, seglen, overlap_div, win, FS, rejects, detrend)
    psd_e, _, _ = welch_psd(ve10, seglen, overlap_div, win, FS, rejects, detrend)

    # GPS-referanse: vUp interpolert på NØYAKTIG samme 10 Hz-bøttesenter som
    # IMU-serien, så Welch-segmenteringen og frekvens-binsene blir identiske.
    # NB: bruker full_idx (etter hull-fylling), ikke bidx10 - ellers ville GPS
    # ligget på den ekte tidsaksen mens IMU-serien var komprimert.
    psd_v, gps_noise, gps_ratios, gps_f = None, None, None, None
    psd_v_eq = None                  # psd_v·ω²: mates til wave_params (som deler på ω⁴)
    vg10 = None                      # GPS-vUp på bøtteaksen - se v10 i returverdien
    if gps is not None and len(full_idx):
        tg, vup = gps
        bt = (np.asarray(full_idx, dtype=np.float64) + 0.5) * BUCKET_MS / 1000.0
        vg10 = np.interp(bt, tg, vup - float(np.mean(vup)))
        psd_v, _, _ = welch_psd(vg10, seglen, overlap_div, win, FS, rejects, detrend)
        gps_noise = gps_noise_floor(psd_v, seglen)
        # GPS-elevasjon = S_vUp/ω², men wave_params deler på ω⁴ - vi pre-
        # multipliserer med ω² så samme funksjon kan brukes for alle metodene.
        w2 = (2.0 * np.pi * np.arange(seglen // 2 + 1) * FS / seglen) ** 2
        psd_v_eq = psd_v * w2

    # GPS-utledet f_c beregnes ALLTID når GPS finnes - den er billig og trengs
    # til sammenligningstabellen, uavhengig av hvilken cutoff_mode som er valgt.
    gps_fcs, gps_own_fc, gps_snr, gps_rmin = {}, None, None, None
    if psd_v is not None and nseg > 0:
        for key, psd in (("m", psd_m), ("s", psd_s), ("k", psd_k), ("n", psd_n),
                         ("e", psd_e)):
            # Øvre søkegrense er gps_fmax, IKKE fmax: se GPS_SEARCH_FMAX.
            gps_fcs[key], gps_ratios, gps_f, gps_rmin = find_cutoff_gps(
                psd, psd_v, seglen, cut_fmin, gps_fmax,
                gps_rel, gps_smooth_hz, gps_span_hz)
        # GPS' EGEN cut-off, helt uavhengig av IMU-en: første spektrale minimum i
        # S_vUp/ω². psd_v_eq = psd_v·ω², så find_cutoff sin /ω⁴ gir nettopp det.
        gps_own_fc = find_cutoff(psd_v_eq, seglen, cut_fmin, cut_distance,
                                 gps_own_prom, fmax)
        # Er GPS fortsatt over eget støygulv der IMU-ens f_c havnet? Hvis ikke,
        # er IMU/GPS-kriteriet en sammenligning av støy mot støy.
        gps_snr = gps_snr_at(psd_v, seglen, gps_fcs.get("m"), gps_noise)

    # Lavfrekvens-avskjæring: fast taper, auto-detektert f_c, eller GPS-utledet.
    # f_c finnes per metode (Madgwick/SFLP/Kalman/NXP/MEKF har ulik lavfrekvent
    # støy).
    tapers, fcs, fpeaks = {}, {}, {}
    gps_clamped, gps_bands = {}, {}     # f_c klemt mot maks, og påført (f1,f2)
    for key, psd in (("m", psd_m), ("s", psd_s), ("k", psd_k), ("n", psd_n),
                     ("e", psd_e)):
        fc = None
        fpeaks[key] = None
        if nseg > 0 and cutoff_mode == "auto":
            fc = find_cutoff(psd, seglen, cut_fmin, cut_distance,
                             cut_prominence, fmax)
            fpeaks[key] = peak_frequency(psd, seglen, cut_fmin, fmax)
            fcs[key] = fc
            tapers[key] = make_taper(seglen, f1, f2, fc)
            continue
        if cutoff_mode == "gps":
            fc = gps_fcs.get(key)
        if cutoff_mode == "gps" and fc is not None:
            # Halv-cosinus-rampe f_c -> gps_ramp*f_c i stedet for hardt kutt, med
            # nedre kant klemt mot maksgrensen (øvre kant følger av rampen).
            g1 = min(fc, gps_max_f1) if gps_max_f1 > 0.0 else fc
            if g1 != fc:
                gps_clamped[key] = fc      # rå f_c, før klemming
            fcs[key] = fc
            tapers[key] = make_taper(seglen, g1, gps_ramp * g1, None)
            gps_bands[key] = (g1, gps_ramp * g1)
            continue
        fcs[key] = fc                       # None => fast taper (eller mislyktes)
        tapers[key] = make_taper(seglen, f1, f2, fc)
    wp_m = wave_params(psd_m, seglen, fmax, tapers["m"])
    wp_s = wave_params(psd_s, seglen, fmax, tapers["s"])
    wp_k = wave_params(psd_k, seglen, fmax, tapers["k"])
    wp_n = wave_params(psd_n, seglen, fmax, tapers["n"])
    wp_e = wave_params(psd_e, seglen, fmax, tapers["e"])

    # GPS som fjerde metode. Den får SAMME taper som Madgwick, ellers ville Hs
    # blitt summert over et annet frekvensbånd enn de andre og kolonnene ikke
    # vært sammenlignbare. (I gps-modus er f_c uansett utledet fra IMU/GPS-
    # forholdet, så det finnes ingen egen GPS-f_c.)
    #
    # wp_gps_raw er GPS HELT uten lavfrekvensbegrensning: T=1 fra df og opp, altså
    # fmin=0. Den er referansen som viser hva taperen faktisk holder unna - GPS er
    # den eneste metoden som kan svare på det, siden den bare trenger ω⁻² mot
    # IMU-ens ω⁻⁴. Merk at ω⁻² likevel forsterker GPS-ens eget hastighetsstøygulv
    # kraftig i de nederste binsene, så differansen mot wp_gps er dels ekte
    # lavfrekvensenergi og dels støy - den skal leses som en øvre grense.
    # NB: her sto make_taper(..., None) før, som er NØYAKTIG samme faste taper
    # som tapers["m"] i taper-modus - "utapert" var da en ren duplikat av wp_gps.
    wp_gps = wp_gps_raw = None
    if psd_v_eq is not None:
        tapers["g"] = tapers["m"]
        fcs["g"] = fcs["m"]
        fpeaks["g"] = None
        wp_gps = wave_params(psd_v_eq, seglen, fmax, tapers["g"])
        wp_gps_raw = wave_params(psd_v_eq, seglen, fmax,
                                 make_taper(seglen, f1, f2, fc=0.0))

    return dict(psd_m=psd_m, psd_s=psd_s, psd_k=psd_k, psd_n=psd_n, psd_e=psd_e,
                nseg=nseg, nrej=nrej,
                wp_m=wp_m, wp_s=wp_s, wp_k=wp_k, wp_n=wp_n, wp_e=wp_e,
                tapers=tapers, fcs=fcs, fpeaks=fpeaks, cutoff_mode=cutoff_mode,
                psd_v=psd_v, psd_v_eq=psd_v_eq, wp_gps=wp_gps,
                wp_gps_raw=wp_gps_raw, gps_fcs=gps_fcs,
                gps_own_fc=gps_own_fc, gps_snr=gps_snr, gps_rmin=gps_rmin,
                gps_clamped=gps_clamped, gps_bands=gps_bands, gps_ramp=gps_ramp,
                gps_noise=gps_noise, gps_ratios=gps_ratios, gps_f=gps_f,
                n_data=len(rows), n_brake=n_brake, n10=len(vm10),
                n_brake_buckets=int(sum(brake10)), brake_reject=brake_reject,
                n_ovf=n_ovf, n_ovf_buckets=int(sum(ovf10)), ovf_reject=ovf_reject,
                # Seriene på analyseraten, klare til kryssspektral analyse i
                # selfnoise.py. "g" er GPS-vUp [m/s]; de øvrige er vertikal
                # akselerasjon [m/s²]. rejects/bidx følger med fordi en ekstern
                # modul MÅ kunne bruke nøyaktig samme segmenter - to kanaler midlet
                # over ulike deler av økta gir et meningsløst kryssspekter.
                v10=dict(m=vm10, s=vs10, k=vk10, n=vn10, e=ve10, g=vg10),
                # Raten NXP-filteret faktisk ble konstruert med. Tas med fordi
                # den er en egenskap ved RESULTATET her (fast tidssteg), ikke
                # bare ved inngangsdataene - se kalman_nxp.py.
                nxp_fs=1000.0 / nxp_dt_ms,
                rejects=rejects, bidx=np.asarray(full_idx),
                gapstats=gapstats, gap_reject=gap_reject, noise_band=noise_band,
                detrend=detrend, beta=beta,
                # Hvilken kjede tallene faktisk kom fra: "raw" = vacc_fir fra
                # fila (AHRS på råstrømmen), "csv" = replay på radraten. Og
                # hvilken rad som byttet kilde - resten er replay som før.
                vacc_source=("raw" if bruk_raw else "csv"),
                vacc_source_method={"m": "madgwick", "k": "kalman"}.get(raw_slot),
                vacc_source_col=(raw_col if bruk_raw else None),
                decimate_mode=decimate_mode, firstats=firstats)


# --- Utskrift (samme filformat som firmware) ---------------------------------
def spectrum_bins(psd_m, psd_s, psd_k, psd_n, psd_e, seglen, fmax, tapers,
                  psd_v=None):
    """Bygg per-frekvens-tabellen (samme utregning som spec.csv/waveParams):
    accel-PSD som den er, og elevasjon = acc/ω⁴·T². Bins tas med så lenge MINST
    én metode har taper > 0 (metodene kan ha ulik f_c i auto-modus); der en
    metodes taper er 0 blir dens elevasjon 0.
    Returnerer dict av lister -> delt av write_spec/print_spectrum/plot_spectrum."""
    df = FS / seglen
    tm, ts, tk, tn, te = (tapers["m"], tapers["s"], tapers["k"], tapers["n"],
                          tapers["e"])
    keys = ["f", "T", "acc_m", "acc_s", "acc_k", "acc_n", "acc_e",
            "eta_m", "eta_s", "eta_k", "eta_n", "eta_e"]
    if psd_v is not None:
        keys += ["eta_gps", "eta_gps_raw"]
    out = {k: [] for k in keys}
    for k in range(1, seglen // 2 + 1):
        f = k * df
        if f > fmax:
            break
        if (tm[k] <= 0.0 and ts[k] <= 0.0 and tk[k] <= 0.0 and tn[k] <= 0.0
                and te[k] <= 0.0):
            continue
        w4 = (2.0 * math.pi * f) ** 4
        out["f"].append(f)
        out["T"].append(1.0 / f)
        out["acc_m"].append(psd_m[k])
        out["acc_s"].append(psd_s[k])
        out["acc_k"].append(psd_k[k])
        out["acc_n"].append(psd_n[k])
        out["acc_e"].append(psd_e[k])
        out["eta_m"].append(psd_m[k] / w4 * tm[k] * tm[k])
        out["eta_s"].append(psd_s[k] / w4 * ts[k] * ts[k])
        out["eta_k"].append(psd_k[k] / w4 * tk[k] * tk[k])
        out["eta_n"].append(psd_n[k] / w4 * tn[k] * tn[k])
        out["eta_e"].append(psd_e[k] / w4 * te[k] * te[k])
        if psd_v is not None:
            # GPS: elevasjon fra vertikalhastighet krever kun ω² (ikke ω⁴).
            # Samme taper som de andre, så kurvene er sammenlignbare bin for bin.
            tg = tapers["g"]
            eta_g = psd_v[k] / (2.0 * math.pi * f) ** 2
            out["eta_gps"].append(eta_g * tg[k] * tg[k])
            # Utapert GPS i tillegg: med kun ω⁻² (mot IMU-ens ω⁻⁴) er GPS langt
            # mindre utsatt for lavfrekvent støyforsterkning, så den utaperte
            # kurven viser om taperen kutter ekte energi eller bare støy.
            out["eta_gps_raw"].append(eta_g)
    return out


def write_spec(path, bins):
    gps = "eta_gps" in bins
    with open(path, "w") as sf:
        sf.write("f_hz,psd_acc_madgwick,psd_eta_madgwick,psd_acc_sflp,psd_eta_sflp,"
                 "psd_acc_kalman,psd_eta_kalman,psd_acc_nxp,psd_eta_nxp,"
                 "psd_acc_mekf,psd_eta_mekf"
                 + (",psd_eta_gps,psd_eta_gps_raw\n" if gps else "\n"))
        for i in range(len(bins["f"])):
            sf.write(f"{bins['f'][i]:.5f},"
                     f"{bins['acc_m'][i]:.6f},{bins['eta_m'][i]:.6f},"
                     f"{bins['acc_s'][i]:.6f},{bins['eta_s'][i]:.6f},"
                     f"{bins['acc_k'][i]:.6f},{bins['eta_k'][i]:.6f},"
                     f"{bins['acc_n'][i]:.6f},{bins['eta_n'][i]:.6f},"
                     f"{bins['acc_e'][i]:.6f},{bins['eta_e'][i]:.6f}"
                     + (f",{bins['eta_gps'][i]:.6f},"
                        f"{bins['eta_gps_raw'][i]:.6f}\n" if gps else "\n"))


def print_spectrum(bins):
    """Skriv elevasjonsspekteret S_eta(f) til konsollen + topp-frekvens per metode."""
    f = bins["f"]
    if not f:
        print("  (ingen frekvens-bins i båndet - for kort serie?)")
        return
    gps = "eta_gps" in bins
    print("  --- elevasjonsspekter S_eta(f) [m^2/Hz] ---")
    print("      f[Hz]   T[s]   Madgwick       SFLP     Kalman        NXP"
          "       MEKF" + ("        GPS" if gps else ""))
    for i in range(len(f)):
        print(f"   {f[i]:8.4f} {bins['T'][i]:6.1f}  {bins['eta_m'][i]:10.5f} "
              f"{bins['eta_s'][i]:10.5f} {bins['eta_k'][i]:10.5f} "
              f"{bins['eta_n'][i]:10.5f} {bins['eta_e'][i]:10.5f}"
              + (f" {bins['eta_gps'][i]:10.5f}" if gps else ""))
    # Topp-frekvens (der elevasjonsenergien er størst) per metode.
    methods = [("Madgwick", "eta_m"), ("SFLP", "eta_s"), ("Kalman", "eta_k"),
               ("NXP", "eta_n"), ("MEKF", "eta_e")]
    if gps:
        methods.append(("GPS", "eta_gps"))
    for name, key in methods:
        j = max(range(len(f)), key=lambda i: bins[key][i])
        print(f"   topp {name:8s}: f={f[j]:.4f} Hz (T={bins['T'][j]:.1f} s)")


def log_hz_axis(ax, xlim=None):
    """Logaritmisk frekvensakse med lesbare Hz-tall i stedet for 10⁻¹-notasjon.

    Skilt ut som egen funksjon fordi selfnoise.py tegner figurer som havner ved
    siden av disse i oppgaven: to sett tick-regler som driver fra hverandre er
    verre enn litt indirekte kode.

    Log x-akse fordi bølgebåndet spenner over en dekade (0.03-0.5 Hz) og
    lavfrekvensenden er der hele støygulv-diskusjonen ligger - lineær x klemmer
    den sammen til ingenting. Med log-log blir dessuten et f⁻⁴-gulv en RETT
    LINJE, som er selve diagnosen.

    matplotlib.ticker importeres lazy, som ellers i denne fila."""
    import matplotlib.ticker as mticker
    ax.set_xscale("log")
    if xlim:
        ax.set_xlim(*xlim)
    ax.grid(True, which="both", alpha=0.3)
    # Major og minor MÅ ha hver sine ticks, ellers merkes samme punkt to ganger
    # og etikettene legger seg oppå hverandre: dekadene som major, kun 2 og 5
    # som merkede minor.
    ax.xaxis.set_major_locator(mticker.LogLocator(base=10.0, subs=(1.0,)))
    ax.xaxis.set_minor_locator(mticker.LogLocator(base=10.0, subs=(2.0, 5.0)))

    # ScalarFormatter velger ETT desimalantall for hele serien og skriver da
    # "0.00" for 0.01 og "10.00" for 10. Her trengs desimaler per verdi.
    def _hz(v, _pos):
        if v <= 0.0:
            return ""
        return f"{v:.{max(0, int(math.ceil(-math.log10(v))))}f}"

    for which in ("major", "minor"):
        getattr(ax.xaxis, f"set_{which}_formatter")(mticker.FuncFormatter(_hz))
    ax.tick_params(axis="x", which="minor", labelsize=7)


def plot_spectrum(bins, stamp, seglen, f1, f2, fmax, save_path=None, fcs=None):
    """2-panels log-plot: accel-PSD og elevasjons-PSD for alle metodene.
    fcs=None (taper-modus): taper-båndet [f1,f2] skyggelagt.
    fcs satt (auto-modus): auto-detektert f_c tegnes som stiplet linje per metode
    (i metodens egen farge); metoder uten deteksjon bruker fast taper.
    matplotlib importeres lazy."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (matplotlib ikke installert - hopper over plot; 'pip install matplotlib')")
        return
    f = bins["f"]
    if not f:
        print("  (ingen frekvens-bins å plotte)")
        return

    def masked(key):
        # 0 (utenfor taper) gir -inf i log-akse -> nan tegnes ikke.
        return [v if v > 0.0 else float("nan") for v in bins[key]]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    colors = {}
    meths = [("Madgwick", "m"), ("SFLP", "s"), ("Kalman", "k"), ("NXP", "n"),
             ("MEKF", "e")]
    for name, key in meths:
        line, = ax1.semilogy(f, bins["acc_" + key], label=name)
        colors[key] = line.get_color()
        ax2.semilogy(f, masked("eta_" + key), color=colors[key], label=name)
    if "eta_gps" in bins:
        # Uavhengig fasit: GPS vUp -> elevasjon via ω⁻² (ikke ω⁻⁴).
        ax2.semilogy(f, masked("eta_gps"), color="k", ls=":", lw=1.6,
                     label="GPS vUp (ω⁻²)")
    ax1.set_ylabel("PSD accel  [(m/s²)²/Hz]")
    ax1.set_title(f"{stamp}   seglen={seglen}")
    ax2.set_ylabel("PSD elevasjon  [m²/Hz]")
    ax2.set_xlabel("frekvens [Hz]")
    for ax in (ax1, ax2):
        if fcs is None:
            ax.axvspan(f1, f2, color="orange", alpha=0.15)  # taper-rampe
        else:
            for key, fc in fcs.items():
                if fc is not None:
                    ax.axvline(fc, color=colors[key], ls="--", lw=1.0, alpha=0.8)
        log_hz_axis(ax, PLOT_XLIM)
        ax.legend()
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=110)
        print(f"  skrev {save_path}")
    # INGEN plt.show(): på en GUI-backend blokkerer den til vinduet lukkes, og
    # da henger enhver som kjører dette som subprosess (mapplot.py gjør det) -
    # målt 6m22s vegg-tid mot 18s CPU. Figuren er allerede skrevet til
    # save_path; åpne den derfra.
    plt.close(fig)


def variant_wps(res, seglen, fmax, f1, f2, fcs=None, with_gps=True, ramp=None):
    """wave_params per metode under ÉN taper-variant, regnet ut fra de allerede
    beregnede PSD-ene (ingen ny gjennomgang av rådata).
      fcs = None   -> fast Kohout/Tucker-Pitt-taper f1..f2
      fcs = tall   -> hard avskjæring ved den frekvensen for alle metodene
      fcs = dict   -> hard avskjæring ved egen f_c per metode
    ramp satt -> halv-cosinus fra f_c til ramp*f_c i stedet for hardt kutt.
    with_gps=False utelater GPS helt: GPS-elevasjon går som ω⁻², ikke ω⁻⁴, og
    trenger derfor ingen lavfrekvens-taper - å påføre den ville bare kastet bort
    ekte GPS-signal. GPS hører hjemme i den utaperede varianten."""
    def tap(fc):
        if ramp is not None and fc is not None:
            return make_taper(seglen, fc, ramp * fc, None)
        return make_taper(seglen, f1, f2, fc)

    out = {}
    for key, psd in (("m", res["psd_m"]), ("s", res["psd_s"]),
                     ("k", res["psd_k"]), ("n", res["psd_n"]),
                     ("e", res["psd_e"])):
        fc = fcs.get(key) if isinstance(fcs, dict) else fcs
        out[key] = wave_params(psd, seglen, fmax, tap(fc))
    if with_gps:
        if res["psd_v_eq"] is not None:
            fc = fcs.get("m") if isinstance(fcs, dict) else fcs
            out["g"] = wave_params(res["psd_v_eq"], seglen, fmax, tap(fc))
        else:
            out["g"] = None         # gps.csv mangler -> kolonnen vises som '-'
    return out


def print_wave_table(wps, label=None, marker=" "):
    """Skriv Hs/Tz/Tc-tabellen. GPS-kolonnen tas kun med når 'g' finnes i wps,
    og vises da som '-' hvis gps.csv manglet."""
    if label:
        print()                     # luft mellom variant-tabellene
        print(f" {marker} {label}")
    gps = "g" in wps

    def cell(key, metric, fmt):
        wp = wps.get(key)
        return f"{wp[metric]:{fmt}}" if wp is not None else f"{'-':>7}"

    print("             Madgwick     SFLP      Kalman      NXP      MEKF"
          + ("      GPS" if gps else ""))
    for metric, name, fmt in (("hs", "Hs [m]  ", "7.3f"),
                              ("tz", "Tz [s]  ", "7.2f"),
                              ("tc", "Tc [s]  ", "7.2f")):
        print(f"   {name}:  {cell('m', metric, fmt)}  {cell('s', metric, fmt)}  "
              f"{cell('k', metric, fmt)}  {cell('n', metric, fmt)}  "
              f"{cell('e', metric, fmt)}"
              + (f"  {cell('g', metric, fmt)}" if gps else ""))
def print_variants(res, seglen, fmax, f1, f2, active_mode):
    """Tre tabeller side om side i tid: utapert, fast taper, GPS-utledet f_c.
    Viser hvor mye av Hs som er et valg av lavfrekvensgrense og hvor mye som er
    metodeforskjell. Pilen markerer varianten som skrives til fil."""
    gps_fcs = res["gps_fcs"]
    print("\n  === lavfrekvens-behandling: samme PSD, tre avskjæringer ===")

    # GPS er kun med i [1]: uten taper er GPS-tallet det ærligste vi har (ω⁻²
    # trenger ingen avskjæring), og det er den eneste raden der GPS-kolonnen
    # tilfører noe de andre ikke allerede sier.
    print_wave_table(
        variant_wps(res, seglen, fmax, f1, f2, f1),
        f"[1] utapert - hardt kutt ved f1={f1:.3f} Hz, ingen rampe",
        "->" if active_mode == "none" else " ")

    print_wave_table(
        variant_wps(res, seglen, fmax, f1, f2, None, with_gps=False),
        f"[2] fast taper - Kohout/Tucker-Pitt {f1:.3f}-{f2:.3f} Hz",
        "->" if active_mode == "taper" else " ")

    if gps_fcs and any(v is not None for v in gps_fcs.values()):
        ramp = res["gps_ramp"]
        # Rampen er PER METODE - metodene har ulik lavfrekvent støy og dermed
        # ulik f_c. Skrives derfor som ett bånd per metode, på samme form som
        # taper-båndet i [2] og klart til å limes rett inn i TAPER_F1/TAPER_F2.
        # Ett felles spenn (min f_c .. ramp x maks f_c) ville blandet metodene:
        # nedre kant fra én metode, øvre fra en annen, og en enkelt runaway-
        # deteksjon ville sett ut som en urimelig bred, men gyldig rampe.
        # Metoder uten treff faller tilbake på den faste taperen fra [2] - se
        # tap() i variant_wps - og merkes 'fast'.
        bands = "  ".join(
            f"{nm} {v:.3f}-{ramp * v:.3f}" if v is not None else f"{nm} fast"
            for nm, v in zip(("M", "S", "K", "N", "E"),
                             (gps_fcs.get(k) for k in ("m", "s", "k", "n", "e"))))
        print_wave_table(
            variant_wps(res, seglen, fmax, f1, f2, gps_fcs, with_gps=False,
                        ramp=ramp),
            f"[3] GPS-utledet f_c - rampe x{ramp:g}, egen f_c per metode [Hz]\n"
            f"      {bands}",
            "->" if active_mode == "gps" else " ")
    else:
        print("\n    [3] GPS-utledet f_c - utilgjengelig "
              + ("(ingen gps.csv)" if not gps_fcs
                 else "(IMU/GPS-kravet ble aldri oppfylt)"))
    if active_mode == "auto":
        print("    (--cutoff auto er aktiv - se egen tabell over)")


def write_ana(path, res, seglen, args):
    wp_m, wp_s, wp_k = res["wp_m"], res["wp_s"], res["wp_k"]
    with open(path, "w") as out:
        out.write("key,value\n")
        out.write(f"imu_rows,{res['n_data']}\n")
        out.write(f"brake_windows,{res['n_brake']}\n")
        out.write(f"vacc10hz_samples,{res['n10']}\n")
        out.write(f"welch_segments,{res['nseg']}\n")
        out.write(f"welch_rejected_brake,{res['nrej'].get('brems', 0)}\n")
        out.write(f"welch_rejected_ovf,{res['nrej'].get('fifo_ovf', 0)}\n")
        out.write(f"fifo_ovf_windows,{res['n_ovf']}\n")
        # Hvor mye av øktstarten som ble utelatt. Skrives alltid (også 0), ellers
        # kan man ikke se på en ana-fil om Hs gjelder hele økta eller bare halen.
        out.write(f"skip_start_s,{args.skip_start:.0f}\n")
        out.write(f"welch_rejected_start,{res['nrej'].get('start', 0)}\n")

        # Analysekonfigurasjonen, samme nøkler som firmware skriver i sin ana.csv.
        # Den STO i cfg.csv, men cfg beskriver hvordan imu.csv/gps.csv ble til -
        # og denne kjøringen bruker som regel andre parametre enn firmware gjorde
        # (taper og fmax overstyres fra kommandolinja). Skrives derfor her, slik at
        # hver ana-fil kan leses uten å gjette hvilke premisser tallene hviler på.
        out.write(f"vacc_bucket_ms,{BUCKET_MS}\n")
        out.write(f"vacc_fs_hz,{FS:.3f}\n")
        out.write(f"welch_seglen,{seglen}\n")
        out.write(f"welch_overlap_div,{args.overlap_div}\n")
        out.write(f"welch_step,{seglen // args.overlap_div}\n")
        # Stor forbokstav: firmware skriver "Hann"/"Hamming", og nøkkelen skal kunne
        # sammenlignes direkte mellom de to ana-filene.
        out.write(f"welch_window,{args.window.capitalize()}\n")
        out.write(f"psd_df_hz,{FS / seglen:.6f}\n")
        out.write(f"wave_fmax_hz,{args.fmax:.3f}\n")
        out.write(f"taper_f1_hz,{args.taper_f1:.3f}\n")
        out.write(f"taper_f2_hz,{args.taper_f2:.3f}\n")

        out.write(f"cutoff_mode,{res['cutoff_mode']}\n")
        out.write(f"decimate_mode,{res['decimate_mode']}\n")
        # Uten disse to kan en Hs i denne fila ikke leses: samme nøkkel
        # (Hs_madgwick) betyr AHRS på råstrømmen i den ene kjøringen og replay på
        # radraten i den neste, og på 20260813 skiller de +7.5 %.
        out.write(f"vacc_source,{res['vacc_source']}\n")
        if res["vacc_source_method"]:
            out.write(f"vacc_source_method,{res['vacc_source_method']}\n")
            out.write(f"vacc_source_col,{res['vacc_source_col']}\n")
        # Per metode: hvor serien kom fra, og hvilken rate filteret kjørte på.
        # Uten disse er tabellen uleselig - Hs_kalman fra 480 Hz og Hs_kalman fra
        # et 100 Hz-replay står under samme nøkkel og er ikke samme størrelse.
        for navn, _ in METHODS:
            if navn in res.get("srcs", {}):
                out.write(f"src_{navn},{res['srcs'][navn]}\n")
            hz = res.get("rates", {}).get(navn)
            if hz:
                out.write(f"rate_{navn}_hz,{hz:g}\n")
        # Hva sensoren FAKTISK leverte, mot det cfg.csv ba om. Se faktisk_odr().
        if res.get("imu_odr_actual"):
            out.write(f"imu_odr_actual_hz,{res['imu_odr_actual']:.1f}\n")
        if res["firstats"] is not None:
            out.write(f"fir_ntap,{res['firstats']['ntap']}\n")
            out.write(f"fir_cutoff_hz,{res['firstats']['cutoff']:.3f}\n")
            out.write(f"fir_decimation,{res['firstats']['dec']}\n")
            out.write(f"fir_delay_ms,{res['firstats']['delay_s'] * 1000:.1f}\n")
        if res["wp_gps"] is not None:
            out.write(f"Hs_gps,{res['wp_gps']['hs']:.3f}\n")
            out.write(f"Tz_gps,{res['wp_gps']['tz']:.2f}\n")
            out.write(f"Tc_gps,{res['wp_gps']['tc']:.2f}\n")
            out.write(f"Hs_gps_utapert,{res['wp_gps_raw']['hs']:.3f}\n")
            out.write(f"Tz_gps_utapert,{res['wp_gps_raw']['tz']:.2f}\n")
            # Tc manglet her mens Hs/Tz ble skrevet - da kunne den utaperte
            # GPS-varianten ikke settes opp mot de andre i en full Hs/Tz/Tc-rad.
            out.write(f"Tc_gps_utapert,{res['wp_gps_raw']['tc']:.2f}\n")
            out.write(f"gps_vup_noise_psd,{res['gps_noise']:.3e}\n")
        for name, key in (("madgwick", "m"), ("sflp", "s"), ("kalman", "k"),
                          ("nxp", "n"), ("mekf", "e")):
            fc = res["fcs"][key]
            out.write(f"fc_{name},{fc:.4f}\n" if fc is not None else f"fc_{name},-1\n")
        out.write(f"Hs_madgwick,{wp_m['hs']:.3f}\n")
        out.write(f"Tz_madgwick,{wp_m['tz']:.2f}\n")
        out.write(f"Tc_madgwick,{wp_m['tc']:.2f}\n")
        out.write(f"Hs_sflp,{wp_s['hs']:.3f}\n")
        out.write(f"Tz_sflp,{wp_s['tz']:.2f}\n")
        out.write(f"Tc_sflp,{wp_s['tc']:.2f}\n")
        out.write(f"Hs_kalman,{wp_k['hs']:.3f}\n")
        out.write(f"Tz_kalman,{wp_k['tz']:.2f}\n")
        out.write(f"Tc_kalman,{wp_k['tc']:.2f}\n")
        out.write(f"Hs_nxp,{res['wp_n']['hs']:.3f}\n")
        out.write(f"Tz_nxp,{res['wp_n']['tz']:.2f}\n")
        out.write(f"Tc_nxp,{res['wp_n']['tc']:.2f}\n")
        # MEKF = det bøya selv ville regnet ut med KalmanAhrs valgt i
        # wave_config.h. Sammenlign mot firmwares egen <stamp>_ana.csv, ikke mot
        # kalman-kolonnen over.
        out.write(f"Hs_mekf,{res['wp_e']['hs']:.3f}\n")
        out.write(f"Tz_mekf,{res['wp_e']['tz']:.2f}\n")
        out.write(f"Tc_mekf,{res['wp_e']['tc']:.2f}\n")
        # Raten NXP-filteret kjørte på hører med i sammendraget: den er ikke
        # utledbar av de andre feltene, og resultatet avhenger av den.
        out.write(f"nxp_fs_hz,{res['nxp_fs']:.4g}\n")


def main():
    # Må stå før første bruk av FS i funksjonen (argparse-defaultene leser den).
    global BUCKET_MS, FS
    ap = argparse.ArgumentParser(description="Offline ORB-etterprosessering (speiler DEBUG_POSTPROCESS).")
    ap.add_argument("path", nargs="?", default=DEFAULT_PATH,
                    help="Øktkatalog eller sti til <stamp>_imu.csv "
                         "(default: DEFAULT_PATH i toppen av fila, for F5 i Spyder)")
    ap.add_argument("--decimate-hz", type=float, default=FS,
                    help=f"Raten IMU-serien desimeres til før Welch (default {FS:g}). "
                         "1000/raten må gi et helt antall ms - bøtteindeksene er "
                         "heltallsaritmetikk. Seglengde og båndgrense følger med.")
    ap.add_argument("--seglen", type=int, default=None,
                    help="Welch-seglengde (potens av 2). Utelatt => utledet av "
                         "--decimate-hz: 1024 @ <=10 Hz, 2048 @ <26, 4096 @ <=52, "
                         "ellers 8192 - se default_seglen()")
    ap.add_argument("--skip-start", type=float, default=0.0, metavar="SEK",
                    help="Se bort fra de første SEK sekundene (f.eks. 300 = 5 min "
                         "sjøsetting/håndtering). Segmenter som berører perioden "
                         "forkastes; AHRS-ene kjører fortsatt over hele økta, så de "
                         "er ferdig konvergert når analysen starter")
    ap.add_argument("--overlap-div", type=int, default=OVERLAP_DIV, help="Steg = seglen/div (4 => 75%%)")
    ap.add_argument("--window", choices=["hann", "hamming"], default=WINDOW)
    ap.add_argument("--fmax", type=float, default=None,
                    help=f"Øvre båndgrense (Hz). Utelatt => {WAVE_FMAX:g}, klemt til "
                         "Nyquist når --decimate-hz er lavere enn det dobbelte")
    ap.add_argument("--taper-f1", type=float, default=TAPER_F1, help="Lavfrekvens-taper start (T=0 under)")
    ap.add_argument("--taper-f2", type=float, default=TAPER_F2, help="Lavfrekvens-taper slutt (T=1 over)")
    ap.add_argument("--beta", type=float, default=MADGWICK_BETA, help="Madgwick beta")
    ap.add_argument("--cutoff", choices=["taper", "auto", "gps"], default=CUTOFF_MODE,
                    help="Lavfrekvens-avskjæring: 'taper' = fast f1/f2, "
                         "'auto' = første spektrale minimum (find_peaks), "
                         "'gps' = der IMU slutter å løpe fra GPS vUp")
    ap.add_argument("--gps-rel", type=float, default=GPS_REL_FACTOR,
                    help="gps: terskel = denne x forholdskurvens eget minimum")
    ap.add_argument("--gps-smooth-hz", type=float, default=GPS_SMOOTH_HZ,
                    help="gps: glidende median over dette båndet (Hz)")
    ap.add_argument("--gps-span-hz", type=float, default=GPS_SPAN_HZ,
                    help="gps: forholdet må ligge under terskelen over dette spennet (Hz)")
    ap.add_argument("--gps-ramp", type=float, default=GPS_RAMP,
                    help="gps: halv-cosinus-rampe f_c -> denne x f_c "
                         "(1.0 = hardt kutt)")
    ap.add_argument("--gps-max-f1", type=float, default=GPS_MAX_F1,
                    help="gps: maks for rampens nedre kant (Hz). Øvre kant "
                         "følger av rampen. 0 = av")
    ap.add_argument("--gps-fmax", type=float, default=GPS_SEARCH_FMAX,
                    help="gps: øvre grense for SØKET etter f_c (Hz). Egen fra "
                         "--fmax; et treff over bølgebåndet er ingen "
                         "lavfrekvens-cut-off")
    ap.add_argument("--gps-own-prominence", type=float, default=GPS_OWN_PROMINENCE,
                    help="Prominens for GPS' EGEN cut-off (gyldighetsgrense for "
                         "referansen, brukes ikke som taper)")
    ap.add_argument("--compare", dest="compare", action="store_true", default=COMPARE,
                    help="Skriv alle tre avskjæringene (utapert / fast / GPS) "
                         "for alle metodene")
    ap.add_argument("--no-compare", dest="compare", action="store_false",
                    help="Kun tabellen for valgt --cutoff")
    ap.add_argument("--cut-fmin", type=float, default=CUT_FMIN,
                    help="auto: nedre søkegrense for f_c (Hz)")
    ap.add_argument("--cut-distance", type=int, default=CUT_DISTANCE,
                    help="auto: minste avstand mellom minima [frekvens-bins]")
    ap.add_argument("--cut-prominence", type=float, default=CUT_PROMINENCE,
                    help="auto: prominens-krav på minimumet")
    ap.add_argument("--brake-reject", type=float, default=BRAKE_REJECT,
                    help="Forkast Welch-segment hvis brems-andel > denne (0 = av)")
    ap.add_argument("--ovf-reject", type=float, default=OVF_REJECT,
                    help="Forkast Welch-segment hvis fifo_ovf-andel > denne "
                         "(0 = enhver forekomst forkaster)")
    ap.add_argument("--no-ovf-reject", dest="ovf_reject", action="store_const",
                    const=None, help="Ikke forkast på fifo_ovf")
    ap.add_argument("--vacc-source", choices=["auto", "raw", "csv"],
                    default=VACC_SOURCE,
                    help="hvor vertikal-accelen hentes fra: raw = kolonnen "
                         "vacc_fir, der AHRS-en alt har kjørt på råstrømmen "
                         "(om bord, eller offline fra raw.bin). csv = replay av "
                         "filteret på radraten, som før. auto (default) = raw "
                         "når kolonnen finnes, og HVERT alternativfilter kjøres "
                         "da på råstrømmen via raw_to_csv (mellomlagres som "
                         "<stamp>_imu_raw_<filter>.csv ved siden av imu.csv). "
                         "csv gjør ingen av delene")
    ap.add_argument("--raw", default=None, metavar="STI",
                    help="<stamp>_raw.bin å kjøre alternativfiltrene på. Uten "
                         "den letes det ved siden av imu.csv og ett nivå opp. "
                         "'off' slår av oppslaget")
    ap.add_argument("--regen-raw", action="store_true",
                    help="lag <stamp>_imu_raw_<filter>.csv på nytt selv om de "
                         "finnes (etter en endring i raw_to_csv/filtrene)")
    ap.add_argument("--detrend", choices=["none", "mean", "linear"], default=DETREND,
                    help="Detrending per Welch-segment. 'none' = firmware-tro; "
                         f"default '{DETREND}'")
    ap.add_argument("--decimate", choices=["mean", "fir"], default=DECIMATE_MODE,
                    help="Hvordan de loggede radene blir til den desimerte serien: "
                         "'bucket' = middel over bøtta (firmware-tro), 'fir' = "
                         f"FIR-lavpass + desimering som sfy-bøya. Default '{DECIMATE_MODE}'")
    ap.add_argument("--fir-ntap", type=int, default=FIR_NTAP,
                    help=f"fir: antall tap (oddetall), default {FIR_NTAP}")
    ap.add_argument("--fir-cutoff", type=float, default=FIR_CUTOFF,
                    help="fir: cut-off i Hz. Utelatt = halve analyseraten, som fir.rs. "
                         "Lavere gir reell margin mot bretting")
    ap.add_argument("--fir-causal", dest="fir_compensate", action="store_false",
                    default=FIR_COMPENSATE_DELAY,
                    help="fir: ikke kompenser gruppeforsinkelsen - firmware-tro, "
                         "men serien ligger da (ntap-1)/2 samples etter GPS")
    ap.add_argument("--gap-reject", type=float, default=GAP_REJECT,
                    help="Forkast Welch-segment hvis > denne andelen er utfylt "
                         "hull i 10 Hz-serien (0 = enhver forekomst forkaster)")
    ap.add_argument("--no-gap-reject", dest="gap_reject", action="store_const",
                    const=None, help="Ikke forkast på hull (men fyll dem likevel)")
    ap.add_argument("--noise-band-lo", type=float, default=NOISE_BAND[0],
                    help="Nedre grense for støygulv-diagnostikken (Hz)")
    ap.add_argument("--noise-band-hi", type=float, default=NOISE_BAND[1],
                    help="Øvre grense for støygulv-diagnostikken (Hz)")
    ap.add_argument("--plot", dest="plot", action="store_true", default=PLOT,
                    help="Plott spekteret (default styres av PLOT i toppen av fila)")
    ap.add_argument("--no-plot", dest="plot", action="store_false", help="Ikke plott")
    args = ap.parse_args()

    if not args.path:
        sys.exit("Ingen sti oppgitt. Sett DEFAULT_PATH i toppen av fila (for F5 i "
                 "Spyder), eller gi sti som argument: python3 postprocess.py <sti>")

    # Analyseraten settes FØRST: resten av fila leser BUCKET_MS/FS som globaler ved
    # kall, så alt som skjer etter dette punktet ser den nye raten.
    if args.decimate_hz <= 0.0:
        sys.exit(f"--decimate-hz maa vaere positiv (fikk {args.decimate_hz:g})")
    bucket = 1000.0 / args.decimate_hz
    if abs(bucket - round(bucket)) > 1e-6:
        # Ikke rund stille: bøtteindeksene er heltallsdivisjon (t // BUCKET_MS), og
        # en rate som ikke deler 1000 ms jevnt ville gitt bøtter av ulik lengde.
        near = [1000.0 / m for m in (int(bucket), int(bucket) + 1) if m > 0]
        sys.exit(f"--decimate-hz {args.decimate_hz:g} gir bøtte {bucket:.3f} ms - "
                 f"maa vaere et helt antall ms. Naermeste gyldige: "
                 + ", ".join(f"{h:.4g} Hz" for h in sorted(set(near))))
    BUCKET_MS = int(round(bucket))
    # FS regnes tilbake fra det avrundede BUCKET_MS, ikke fra argumentet: da er
    # raten alltid nøyaktig konsistent med bøttingen, uten avrundingsdrift.
    FS = 1000.0 / BUCKET_MS

    if args.seglen is None:
        args.seglen = default_seglen(FS)
    elif args.seglen & (args.seglen - 1):
        sys.exit(f"--seglen maa vaere potens av 2 (fikk {args.seglen})")

    # fmax følger med ned når raten er lav, men lar seg ikke dra opp av en høy rate:
    # bølger finnes ikke over WAVE_FMAX, og å ta med flere bins i momentene ville
    # endret Tz/Tc med raten og gjort kjøringene usammenlignbare.
    if args.fmax is None:
        args.fmax = min(WAVE_FMAX, 0.5 * FS)
    if args.fmax > 0.5 * FS:
        print(f"  ADVARSEL: --fmax {args.fmax:g} Hz er over Nyquist ({0.5 * FS:g} Hz) "
              "- momentene stopper uansett ved Nyquist")

    # GPS-støygulvet måles i et fast bånd over bølgene. Det ligger trygt under
    # Nyquist ved alle vanlige rater, men forsvinner helt under ~4 Hz.
    if 0.5 * FS <= GPS_NOISE_BAND[0]:
        print(f"  ADVARSEL: GPS-støybåndet {GPS_NOISE_BAND[0]:g}-{GPS_NOISE_BAND[1]:g} Hz "
              f"ligger over Nyquist ({0.5 * FS:g} Hz) - gps_vup_noise_psd blir 0")

    imu, stamp, directory = resolve_paths(args.path)
    print(f"=== ETTERPROSESSERING {stamp} ===")
    print(f"  imu:   {imu}")

    ses = read_kv(os.path.join(directory, f"{stamp}_ses.csv"))
    if ses:
        # Vis kun nøkler som faktisk finnes (en avbrutt _tmp-økt mangler
        # imu_rows/duration_ms, som skrives først ved stopSession).
        shown = [f"{k}={ses[k]}" for k in
                 ("start_utc_iso", "imu_rows", "gps_rows", "duration_ms")
                 if k in ses]
        if shown:
            print("  ses:   " + "  ".join(shown))

    # cfg.csv holder build-konstantene (ses.csv holder kun per-kjøring-verdiene).
    # Eldre økter har build_seq i ses.csv i stedet, så vi faller tilbake dit.
    # NB: kun LOGGE-parametrene brukes herfra - se RAW_DT_MS i toppen av fila.
    global RAW_DT_MS
    cfg = read_kv(os.path.join(directory, f"{stamp}_cfg.csv"))
    build_seq = cfg.get("build_seq") if cfg else ses.get("build_seq") if ses else None
    if build_seq is not None:
        print(f"  cfg:   build_seq={build_seq}")
    if cfg:
        shown = [f"{k}={cfg[k]}" for k in
                 ("imu_odr_hz", "output_rate_hz", "window_ms", "gps_rate_hz",
                  "log_duration_sec")
                 if k in cfg]
        if shown:
            print("         " + "  ".join(shown))
        try:
            w = float(cfg["window_ms"])
            RAW_DT_MS = w if w > 0.0 else None
        except (KeyError, ValueError):
            RAW_DT_MS = None
        # Fang «be om høyere rate enn loggen» HER, mens vi kan si det pent. Uten
        # dette havner det først i fir_decimate, som bare kan kaste ValueError midt
        # i analysen. Mangler cfg.csv står den grenen fortsatt igjen som nett.
        if RAW_DT_MS and BUCKET_MS < RAW_DT_MS:
            sys.exit(f"--decimate-hz {FS:g} ({BUCKET_MS} ms) er høyere enn loggeraten "
                     f"{1000.0 / RAW_DT_MS:g} Hz ({RAW_DT_MS:g} ms) - "
                     "det finnes ingen data å desimere fra")

    imu_meta = {}
    rows = read_imu_rows(imu, imu_meta)
    print(f"  leste {len(rows)} imu-rader")

    # Alternativfiltrene kjøres på RÅSTRØMMEN når råloggen finnes. Uten den er
    # radene alt vi har, og da replayes de - som før. Dette skjer før run(), så
    # analysen selv slipper å vite noe om filer og delprosesser.
    cfg_ahrs = ahrs_fra_cfg(cfg.get("orientation_name"))
    raw_bin = finn_raalogg(directory, stamp, args.raw)
    if raw_bin and args.vacc_source != "csv":
        print(f"  raw:   {os.path.basename(raw_bin)} "
              f"({os.path.getsize(raw_bin) / 1e6:.1f} MB), fangstens filter er "
              f"{cfg_ahrs}")
    raw_series, kilder = bygg_raa_serier(
        directory, stamp, rows, cfg_ahrs, raw_bin, args.vacc_source,
        regen=args.regen_raw,
        har_kolonne=bool(rows) and rows[0].get("vacc_raw") is not None)
    rater = metode_rater(cfg, rows, kilder)

    # GPS leses alltid når den finnes - vUp gir en uavhengig Hs-kontroll selv
    # når cut-offen ikke utledes fra den.
    gps = read_gps_vup(os.path.join(directory, f"{stamp}_gps.csv"))
    if gps is None:
        print("  gps:   ingen brukbar *_gps.csv (vUp) - GPS-kontroll utelatt")
        if args.cutoff == "gps":
            sys.exit("--cutoff gps krever <stamp>_gps.csv med rel_ms + vUp")
    else:
        print(f"  gps:   {len(gps[0])} fixes, {len(gps[0]) / max(gps[0][-1] - gps[0][0], 1e-9):.2f} Hz")

    def go():
        return run(rows, args.seglen, args.overlap_div, args.window,
                   args.fmax, args.taper_f1, args.taper_f2, args.beta,
                   args.brake_reject, args.ovf_reject,
                   args.cutoff, args.cut_fmin, args.cut_distance,
                   args.cut_prominence,
                   gps, args.gps_rel, args.gps_smooth_hz, args.gps_span_hz,
                   args.gps_own_prominence, args.gps_ramp, args.gps_max_f1,
                   args.gps_fmax, args.detrend, args.gap_reject,
                   (args.noise_band_lo, args.noise_band_hi),
                   decimate_mode=args.decimate, fir_ntap=args.fir_ntap,
                   fir_cutoff=args.fir_cutoff, fir_compensate=args.fir_compensate,
                   skip_start_s=args.skip_start,
                   vacc_source=args.vacc_source,
                   # Hvilket filter kolonnen er laget av. Står i cfg.csv, for det
                   # er en egenskap ved OPPTAKET - ikke ved denne kjøringen.
                   raw_ahrs=cfg.get("orientation_name", "madgwick"),
                   raw_col=imu_meta.get("vacc_col"),
                   raw_series=raw_series)

    res = go()
    # Kilder og rater hører til RESULTATET og ikke til kjøringen: de forteller
    # hvordan hvert tall i tabellen ble til, og skal derfor følge det ut i
    # ana-fila og videre til figuren.
    res["srcs"], res["rates"] = kilder, rater
    res["imu_odr_actual"] = faktisk_odr(rows)
    if res["imu_odr_actual"]:
        satt = cfg.get("imu_odr_hz", "?")
        print(f"  odr:   satt {satt} Hz, faktisk "
              f"{res['imu_odr_actual']:.1f} Hz (av n-kolonnen)")
    ord_ = {"onchip": "on-chip", "csv": "replay"}
    print("  vacc:  " + "  ".join(
        f"{navn}="
        f"{ord_.get(kilder.get(navn, 'csv'), 'råstrøm')}"
        + (f"@{rater[navn]:g}Hz" if rater.get(navn) else "")
        for navn, _ in METHODS))

    bins = spectrum_bins(res["psd_m"], res["psd_s"], res["psd_k"], res["psd_n"],
                         res["psd_e"], args.seglen, args.fmax, res["tapers"],
                         res["psd_v"])

    # _python-suffiks: firmware skriver SELV <stamp>_spec.csv og <stamp>_ana.csv i
    # samme katalog ved øktslutt. Uten eget navn ville denne kjøringen trunkert dem,
    # og de kan ikke regenereres (firmware regnet dem strømmende). Utdata herfra er
    # dessuten bevisst IKKE firmware-tro - se "Avvik fra firmware" øverst.
    spec_path = os.path.join(directory, f"{stamp}_spec_python.csv")
    ana_path = os.path.join(directory, f"{stamp}_ana_python.csv")
    write_spec(spec_path, bins)
    write_ana(ana_path, res, args.seglen, args)

    wp_m, wp_s, wp_k = res["wp_m"], res["wp_s"], res["wp_k"]
    print(f"  brems-vinduer: {res['n_brake']} / {res['n_data']}")
    print(f"  vacc-samples ({FS:g} Hz): {res['n10']}")
    # Analyseraten avgjør hva seglengden faktisk BETYR, så de hører sammen på én
    # linje: uten sekundene sier «seglen 4096» ingenting om oppløsningen.
    print(f"  analyserate: {FS:g} Hz (bøtte {BUCKET_MS} ms), seglen={args.seglen} "
          f"= {args.seglen / FS:.1f} s, df={FS / args.seglen:.4f} Hz, "
          f"fmax={args.fmax:g} Hz")
    print(f"  Welch segm={res['nseg']} (vindu={args.window}, "
          f"detrend={args.detrend}, beta={args.beta:g})")
    if args.detrend == "none":
        print("     detrend=none er firmware-tro; en udetrendet drift lekker som f⁻²"
              " rett inn i lavfrekvensbåndet")
    fst = res["firstats"]
    if fst is None:
        print(f"  desimering: gjennomsnitt over {BUCKET_MS} ms")
    else:
        print(f"  desimering: FIR {fst['ntap']} tap, fc={fst['cutoff']:g} Hz, "
              f"{fst['fs_raw']:.1f} -> {FS:g} Hz (D={fst['dec']}), "
              f"forsinkelse {fst['delay_s'] * 1000:.0f} ms "
              + ("kompensert" if fst["compensate"] else "IKKE kompensert (firmware-tro)"))
        print(f"     {fst['n_flagged']} av {fst['n_out']} bøtter flagget "
              f"({fst['n_edge']} kant, {fst['n_raw_filled']} rå-samples utfylt) - "
              "filterstøtten treffer der ikke ekte data")
    gs = res["gapstats"]
    if gs["n_gaps"] > 0:
        print(f"  hull i den desimerte serien: {gs['n_gaps']} hull, {gs['n_filled']} bøtter "
              f"fylt av {gs['span']} ({100.0 * gs['n_filled'] / max(gs['span'], 1):.2f} %), "
              f"lengste {gs['longest']} bøtter ({gs['longest'] * BUCKET_MS / 1000.0:.1f} s)")
        if gs["n_long"] > 0:
            print(f"     {gs['n_long']} hull er lengre enn {GAP_MAX_FILL} bøtter - "
                  "interpolasjonen der er ren utfylling, ikke data")
        print("     (uten fylling ville tidsaksen vært komprimert nettopp her, "
              "og GPS-referansen ligget på en annen tidsakse enn IMU-en)")
    else:
        print("  hull i den desimerte serien: ingen - tidsaksen er uniform")
    nrej = res["nrej"]
    if args.skip_start > 0.0:
        # Kostnaden er alltid litt større enn selve perioden: siste segment som
        # berører den forkastes i sin helhet, så analysen starter først ved
        # skip_start + inntil én seglengde.
        print(f"  hopper over start: {args.skip_start:g} s -> "
              f"{nrej.get('start', 0)} segm forkastet, analysen starter tidligst "
              f"{args.skip_start + args.seglen / FS:.0f} s inn i økta")
        if res["nseg"] == 0:
            print("     ADVARSEL: ingen segmenter igjen - --skip-start er for stor "
                  "for øktlengden")
    if args.brake_reject > 0.0:
        print(f"  brems-forkasting: terskel={args.brake_reject:.2f} -> "
              f"{nrej.get('brems', 0)} segm forkastet "
              f"({res['n_brake_buckets']} brems-bøtter)")
    if args.ovf_reject is not None:
        print(f"  fifo-overflow-forkasting: terskel={args.ovf_reject:.3f} -> "
              f"{nrej.get('fifo_ovf', 0)} segm forkastet "
              f"({res['n_ovf']} rader / {res['n_ovf_buckets']} bøtter flagget)")
        if res["n_ovf"] == 0:
            print("     ingen overflow logget - enten skjedde det ingen, eller så "
                  "er økta fra firmware < build 55 (uten fifo_ovf-kolonnen)")
        elif res["nseg"] == 0:
            print("     ADVARSEL: alle segmenter forkastet -> ingen spekter. "
                  "Hev --ovf-reject eller bruk --no-ovf-reject")
    if args.cutoff == "auto":
        print(f"  auto cut-off: fmin={args.cut_fmin:.3f} Hz, "
              f"dist={args.cut_distance} bins ({args.cut_distance * FS / args.seglen:.4f} Hz), "
              f"prom={args.cut_prominence:g}")
        for name, key in (("Madgwick", "m"), ("SFLP", "s"), ("Kalman", "k"),
                          ("NXP", "n"), ("MEKF", "e")):
            fc, fp = res["fcs"][key], res["fpeaks"][key]
            if fc is None:
                print(f"   fc {name:8s}: IKKE FUNNET -> faller tilbake på fast "
                      f"taper ({args.taper_f1:.3f}-{args.taper_f2:.3f} Hz)")
                continue
            note = ""
            if fc <= 1.5 * args.cut_fmin:
                # For lav f_c => ω⁻⁴-halen slipper gjennom => Hs overestimeres.
                note = "  ADVARSEL: f_c nær søkegrensen, Hs kan bli overestimert"
            elif fp is not None and fc >= fp:
                # For høy f_c => selve bølgeenergien kuttes bort => Hs underestimeres.
                note = (f"  ADVARSEL: f_c over spektraltoppen ({fp:.4f} Hz), "
                        "bølgeenergi kuttes bort")
            print(f"   fc {name:8s}: {fc:.4f} Hz (T={1.0 / fc:.1f} s){note}")
    elif args.cutoff == "gps":
        rmin = res["gps_rmin"]
        print(f"  GPS-utledet cut-off: glattet over {args.gps_smooth_hz:.3f} Hz, "
              f"terskel {args.gps_rel:g}x kurvens q{GPS_REF_QUANTILE:g}"
              + (f" ({rmin:.2f}) = {args.gps_rel * rmin:.2f}" if rmin else "")
              + f", holdt over {args.gps_span_hz:.3f} Hz, søk i "
                f"{args.cut_fmin:.3f}-{args.gps_fmax:.3f} Hz")
        rat, gf = res["gps_ratios"], res["gps_f"]
        if rat is not None:
            print("      f[Hz]   T[s]  IMU/GPS (glattet)")
            for ff in (0.03, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30, 0.50):
                if ff > args.fmax:
                    break
                i = int(round(ff / (FS / args.seglen))) - 1
                if 0 <= i < len(gf):
                    print(f"   {gf[i]:8.3f} {1.0 / gf[i]:6.1f} {rat[i]:8.1f}")
        for name, key in (("Madgwick", "m"), ("SFLP", "s"), ("Kalman", "k"),
                          ("NXP", "n"), ("MEKF", "e")):
            fc = res["fcs"][key]
            if fc is None:
                print(f"   fc {name:8s}: IKKE FUNNET -> fast taper "
                      f"({args.taper_f1:.3f}-{args.taper_f2:.3f} Hz)")
                continue
            b = res["gps_bands"].get(key)
            band = f" -> taper {b[0]:.3f}-{b[1]:.3f} Hz" if b else ""
            note = (f"  KLEMT (rå f_c {res['gps_clamped'][key]:.3f} Hz > maks "
                    f"{args.gps_max_f1:.3f})" if key in res["gps_clamped"] else "")
            print(f"   fc {name:8s}: {fc:.4f} Hz (T={1.0 / fc:.1f} s)"
                  f"{band}{note}")
        # Gyldighetssjekk av selve referansen: GPS' egen nedre grense må ligge
        # UNDER f_c, ellers måler kriteriet støy mot støy.
        own, snr = res["gps_own_fc"], res["gps_snr"]
        fc_m = res["fcs"]["m"] if args.cutoff == "gps" else res["gps_fcs"].get("m")
        print("   GPS' egen gyldighetsgrense (uavhengig av IMU):")
        print(f"     f_c(GPS) = "
              + (f"{own:.4f} Hz (T={1.0 / own:.1f} s), prom={args.gps_own_prominence:g}"
                 if own is not None else
                 f"ikke funnet med prom={args.gps_own_prominence:g}"))
        if snr is not None:
            print(f"     GPS-SNR ved f_c: {snr:.1f}x over eget støygulv")
        if own is not None and fc_m is not None:
            if own < fc_m and (snr is None or snr >= GPS_MIN_SNR):
                print(f"     OK: GPS er gyldig godt under f_c ({own:.3f} < {fc_m:.3f} Hz)"
                      " - referansen holder der IMU-en kuttes")
            else:
                print(f"     ADVARSEL: GPS' egen grense ({own:.3f} Hz) ligger ikke "
                      f"trygt under f_c ({fc_m:.3f} Hz) - IMU/GPS-kriteriet kan "
                      "sammenligne støy mot støy")
        elif snr is not None and snr < GPS_MIN_SNR:
            print(f"     ADVARSEL: GPS-SNR ved f_c er kun {snr:.1f}x - "
                  "referansen er svak der")
    else:
        print(f"  taper: {args.taper_f1:.3f}-{args.taper_f2:.3f} Hz (fast)")
    wp_g = res["wp_gps"]
    print_wave_table({"m": wp_m, "s": wp_s, "k": wp_k, "n": res["wp_n"],
                      "e": res["wp_e"], "g": wp_g})
    if wp_g is not None:
        # GPS er uavhengig av orienteringsestimatet, men er selv støybegrenset i
        # lavfrekvensenden (ω⁻² på vUp) og under-leser chop over ~0.3 Hz (5 Hz
        # fix-rate + mottakerens tracking-loop). Les kolonnen som kontroll, ikke fasit.
        print(f"   GPS-kolonnen: samme taper som Madgwick. Utapert: "
              f"Hs={res['wp_gps_raw']['hs']:.3f} m, Tz={res['wp_gps_raw']['tz']:.2f} s. "
              f"vUp-støygulv {res['gps_noise']:.2e} (m/s)²/Hz")

    if args.compare:
        print_variants(res, args.seglen, args.fmax, args.taper_f1, args.taper_f2,
                       args.cutoff)
        print(f"   -> pilen markerer varianten som er skrevet til "
              f"{stamp}_ana_python.csv / _spec_python.csv")

    if args.plot:
        png_path = os.path.join(directory, f"{stamp}_spec.png")
        plot_spectrum(bins, stamp, args.seglen, args.taper_f1, args.taper_f2,
                      args.fmax, save_path=png_path,
                      fcs=res["fcs"] if args.cutoff == "auto" else None)
    print(f"  skrev {spec_path}")
    print(f"  skrev {ana_path}")
    print("=== ferdig ===")


if __name__ == "__main__":
    main()

