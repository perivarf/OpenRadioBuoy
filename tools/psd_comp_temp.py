"""Enkel, UTAPERT PSD-sammenligning: Madgwick og Kalman mot GPS.

Utforskingsskript - ikke en del av analysekjeden.

Hvorfor egen fil og ikke bare lese _spec_python.csv: den fila er tapert. Alle
psd_eta-kolonnene er multiplisert med taper² og er derfor EKSAKT NULL under f1,
og fila starter først ved f1. Det er nettopp båndet under taperen dette skriptet
skal vise, så tallene må regnes på nytt fra tidsseriene.

Heldigvis er de allerede utaperte i postprocess.run(): psd_m, psd_k og psd_v er
rå Welch-estimater, og taperen påføres først i spectrum_bins/wave_params. Her
gjenbrukes de som de er.

Begge kanalene refereres til elevasjon [m], som er det eneste stedet de kan
sammenlignes bin for bin - IMU-en måler akselerasjon, GPS-en hastighet:

    IMU:  S_ηη = S_aa / ω⁴          GPS:  S_ηη = S_vv / ω²

Merk at forholdstallet IKKE er et mål på enighet - to kanaler kan ha identisk
PSD og være helt ukorrelerte. Til enighet trengs koherens; se selfnoise.py
--compare og docs/koherens.md avsnitt 9. Det forholdstallet HER viser, er
forsterkningsforskjellen |G|²: hvor mye GPS-mottakerens interne glatting demper
bølgebåndet.

Bruk:
    python3 tools/psd_comp_temp.py <øktkatalog> [<øktkatalog> ...]
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import postprocess as pp                         # noqa: E402

BANDS = ((0.05, 0.10), (0.10, 0.15), (0.15, 0.20), (0.20, 0.25), (0.25, 0.30),
         (0.30, 0.40), (0.40, 0.50), (0.50, 0.70), (0.70, 1.00), (1.00, 1.50))

COLORS = {"Madgwick": "C0", "Kalman": "C2", "GPS": "k"}
XLIM = (0.05, 2.0)


def elevation_psds(res, seglen, fs):
    """Utaperte elevasjons-PSD-er [m²/Hz] for Madgwick, Kalman og GPS.

    Bin 0 hoppes over: ω = 0 gir divisjon på null, og et konstantledd har uansett
    ingen elevasjonstolkning - detrendingen har allerede fjernet det."""
    f = np.arange(1, seglen // 2 + 1) * fs / seglen
    w = 2.0 * np.pi * f
    out = {"Madgwick": res["psd_m"][1:] / w ** 4,
           "Kalman": res["psd_k"][1:] / w ** 4}
    if res["psd_v"] is not None:
        out["GPS"] = res["psd_v"][1:] / w ** 2
    return f, out


def band_table(f, psds, stamp):
    """Båndmiddel av de tre PSD-ene og av forholdstallene.

    Amplitudeforholdet √(S_imu/S_gps) tas med fordi det er tallet som er direkte
    sammenlignbart med |G| og med Hs-forholdet - Hs = 4√m0 er lineær i amplitude,
    ikke i effekt, så en faktor 16 i PSD er en faktor 4 i Hs."""
    print(f"\n=== {stamp}   utapert elevasjons-PSD [m²/Hz]")
    print(f"{'bånd [Hz]':>12} {'Madgwick':>11} {'Kalman':>11} {'GPS':>11} "
          f"{'M/GPS':>9} {'K/GPS':>9} {'√(M/GPS)':>9}")
    for lo, hi in BANDS:
        b = (f >= lo) & (f < hi)
        if not np.any(b):
            continue
        m, k = psds["Madgwick"][b].mean(), psds["Kalman"][b].mean()
        if "GPS" not in psds:
            print(f"{lo:5.2f}-{hi:5.2f} {m:11.3e} {k:11.3e}")
            continue
        g = psds["GPS"][b].mean()
        print(f"{lo:5.2f}-{hi:5.2f} {m:11.3e} {k:11.3e} {g:11.3e} "
              f"{m / g:9.2f} {k / g:9.2f} {np.sqrt(m / g):9.2f}")


def plot_psd(f, psds, stamp, path):
    """Figur 1: de tre utaperte PSD-ene."""
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 5))
    for name in ("Madgwick", "Kalman", "GPS"):
        if name not in psds:
            continue
        # GPS prikket og svart: den er en annen fysisk størrelse referert inn,
        # og skal ikke kunne forveksles med de to AHRS-kurvene.
        ax.loglog(f, psds[name], color=COLORS[name], lw=1.4,
                  ls=":" if name == "GPS" else "-",
                  label=name + (" vUp (ω⁻²)" if name == "GPS" else " (ω⁻⁴)"))
    top = max(float(np.max(p[(f >= XLIM[0]) & (f <= XLIM[1])])) for p in psds.values())
    ax.set_ylim(top / 1e6, top * 3.0)
    ax.set_ylabel("PSD elevasjon  [m²/Hz]")
    ax.set_xlabel("frekvens [Hz]")
    ax.set_title(f"{stamp}   utapert elevasjons-PSD")
    pp.log_hz_axis(ax, XLIM)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"  skrev {path}")
    plt.close(fig)


def plot_ratio(f, psds, stamp, path, smooth=7, lag_t=0.95):
    """Figur 2: forholdstallene IMU/GPS.

    Log y og en referanselinje på 1.0, fordi det interessante er hvor mange
    ganger fra hverandre de ligger, ikke differansen. Rå kurve svakt bak den
    glattede: forholdet mellom to støyete estimater spriker kraftig per bin, og
    uten rådataen synlig ser den glattede kurven mer bestemt ut enn den er.

    lag_t legger inn en førsteordens lavpass 1/|G|² = 1+(ωT)² som referanse.
    Målt over alle seks Skjærhalden-øktene faller forholdstallet på den kurven
    med T = 0.84-1.04 s og 10-18 % rms-avvik, så den er ikke en illustrasjon -
    den er tilpasningen. Sett lag_t = 0 for å utelate den."""
    import matplotlib.pyplot as plt
    if "GPS" not in psds:
        print("  (ingen GPS i økta - hopper over forholdsplottet)")
        return
    k = np.ones(smooth) / smooth
    fig, ax = plt.subplots(figsize=(9, 5))
    for name in ("Madgwick", "Kalman"):
        r = psds[name] / psds["GPS"]
        ax.loglog(f, r, color=COLORS[name], lw=0.6, alpha=0.20)
        ax.loglog(f, np.convolve(r, k, "same") / np.convolve(np.ones_like(r), k, "same"),
                  color=COLORS[name], lw=1.6, label=f"{name} / GPS")
    if lag_t > 0.0:
        ax.loglog(f, 1.0 + (2.0 * np.pi * f * lag_t) ** 2, color="C3", ls="-.", lw=1.4,
                  label=f"1.ordens lavpass, T={lag_t:g} s  (knekk {1/(2*np.pi*lag_t):.2f} Hz)")
    ax.axhline(1.0, color="0.3", ls="--", lw=1.0, label="likt nivå")
    ax.set_ylabel("PSD-forhold  IMU / GPS   [-]")
    ax.set_xlabel("frekvens [Hz]")
    ax.set_title(f"{stamp}   forholdstall (√forhold = amplitudeforhold)")
    pp.log_hz_axis(ax, XLIM)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"  skrev {path}")
    plt.close(fig)


def fit_lag(f, psds, band=(0.4, 1.5)):
    """Tilpass GPS-ens forsterkning |G| = √(S_gps/S_imu) i båndet.

    To tilpasninger, fordi de svarer på hvert sitt spørsmål:
      n  fra |G| ~ f^n            - ER det en systematisk frekvensavhengighet?
      T  fra |G| = 1/√(1+(ωT)²)   - passer den på en førsteordens lavpass?

    Begge gjøres i log-log. En rett minstekvadrat på lineære verdier ville vært
    dominert av de store tallene i toppen av båndet; i log teller en avvik på
    20 % like mye uansett hvor på kurven den ligger, som er det man mener.

    Returnerer (n, T, rms_log). rms_log er avviket til T-modellen, i log-enheter,
    så 0.12 betyr ~12 % typisk avvik."""
    b = (f >= band[0]) & (f <= band[1])
    G = np.sqrt(psds["GPS"][b] / psds["Madgwick"][b])
    fb = f[b]
    n = float(np.polyfit(np.log(fb), np.log(G), 1)[0])
    # Skann T i stedet for å linearisere: modellen er ikke lineær i T, og et
    # grovt skann på 1 ms er både enklere og mer robust enn en optimizer her.
    ts = np.arange(0.05, 5.0, 0.001)
    model = 1.0 / np.sqrt(1.0 + (2.0 * np.pi * np.outer(fb, ts)) ** 2)
    err = np.sqrt(np.mean((np.log(model) - np.log(G)[:, None]) ** 2, axis=0))
    i = int(np.argmin(err))
    return n, float(ts[i]), float(err[i])


def plot_report(sessions, path, lag_t=0.95, smooth=9):
    """Samlefigur over alle øktene - resultatet av kjøringen på én side.

    sessions = liste av dict med stamp, f, psds, fit, hs.

    Fire paneler, ett per påstand: (1) GPS-dempningen er den samme kurven i alle
    øktene, (2) Madgwick og Kalman skiller lag først under ~0.25 Hz, (3) Kalman
    mot GPS direkte, (4) tallene bak. Uten panel 2 ville figuren sett ut som om
    de to filtrene er utskiftbare, som de er over 0.3 Hz og ikke er under.

    Panel 2 og 3 er begge PSD-forhold og deler y-skala, så de kan leses mot
    hverandre; panel 1 er amplitude (√ av et PSD-forhold) fordi det er den
    størrelsen som er sammenlignbar med |G| og med Hs."""
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(11, 17))
    gs = fig.add_gridspec(4, 1, height_ratios=[1.0, 1.0, 1.0, 0.62], hspace=0.42)
    ax1, ax2 = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])
    ax3, ax4 = fig.add_subplot(gs[2]), fig.add_subplot(gs[3])
    k = np.ones(smooth) / smooth

    def sm(y):
        return np.convolve(y, k, "same") / np.convolve(np.ones_like(y), k, "same")

    for i, s in enumerate(sessions):
        f, p = s["f"], s["psds"]
        c = f"C{i}"
        ax1.loglog(f, sm(np.sqrt(p["GPS"] / p["Madgwick"])), color=c, lw=1.4,
                   label=f"{s['stamp'][9:]}  T={s['fit'][1]:.2f} s")
        ax2.loglog(f, sm(p["Kalman"] / p["Madgwick"]), color=c, lw=1.4,
                   label=s["stamp"][9:])
        ax3.loglog(f, sm(p["Kalman"] / p["GPS"]), color=c, lw=1.4,
                   label=s["stamp"][9:])
    fr = np.logspace(np.log10(XLIM[0]), np.log10(XLIM[1]), 400)
    ax1.loglog(fr, 1.0 / np.sqrt(1.0 + (2.0 * np.pi * fr * lag_t) ** 2), color="k",
               ls="--", lw=2.0, label=f"1.ordens, T={lag_t:g} s")
    ax1.axhline(1.0, color="0.5", ls=":", lw=1.0)
    ax1.set_ylim(1e-2, 3.0)
    ax1.set_ylabel("|G| = amplitude GPS / IMU   [-]")
    # Tittelen sier "over 0.5 Hz" fordi kurvene faktisk spriker under den -
    # å skrive "samme kurve i alle seks øktene" ville vært motsagt av figuren.
    ax1.set_title("GPS-mottakerens dempning - samme kurve over 0.5 Hz, "
                  "spriker under (lav SNR)")

    ax2.axhline(1.0, color="0.5", ls=":", lw=1.0)
    ax2.set_ylim(0.25, 5.0)
    ax2.set_ylabel("PSD-forhold  Kalman / Madgwick   [-]")
    ax2.set_title("Filtrene er identiske over ~0.3 Hz og skiller lag under den")

    # Referansekurven er 1/|G|² = 1+(ωT)², altså den samme tilpasningen som i
    # panel 1, snudd. Tas med her fordi hele stigningen i Kalman/GPS over
    # 0.3 Hz ER den kurven - uten den ser panelet ut som et resultat om Kalman.
    ax3.loglog(fr, 1.0 + (2.0 * np.pi * fr * lag_t) ** 2, color="k", ls="--", lw=2.0,
               label=f"1.ordens, T={lag_t:g} s")
    ax3.axhline(1.0, color="0.5", ls=":", lw=1.0)
    ax3.set_ylim(0.1, 300.0)
    ax3.set_ylabel("PSD-forhold  Kalman / GPS   [-]")
    ax3.set_title("Kalman mot GPS - stigningen er GPS-dempningen, ikke Kalman")
    for ax in (ax1, ax2, ax3):
        pp.log_hz_axis(ax, XLIM)
        ax.set_xlabel("frekvens [Hz]")
        ax.legend(fontsize=7.5, ncol=2)

    ax4.axis("off")
    rows = [[s["stamp"][9:], f"{s['fit'][0]:+.2f}", f"{s['fit'][1]:.2f}",
             f"{100 * s['fit'][2]:.0f} %",
             f"{s['hs'][0]:.3f}", f"{s['hs'][1]:.3f}", f"{s['hs'][2]:.3f}",
             f"{s['hs'][2] / s['hs'][0]:.3f}" if s["hs"][0] else "-"]
            for s in sessions]
    tb = ax4.table(cellText=rows,
                   colLabels=["økt", "n i |G|~f^n", "T [s]", "rms",
                              "Hs Madgwick", "Hs Kalman", "Hs GPS", "Hs GPS/M"],
                   loc="upper center", cellLoc="center")
    tb.auto_set_font_size(False)
    tb.set_fontsize(9)
    tb.scale(1.0, 1.6)
    ax4.set_title("Tilpasning over 0.4-1.5 Hz, og Hs som taperen gir "
                  "(taper 0.1-0.2 Hz)", pad=18)
    # Fotnoten hører til figuren, ikke til et panel: den gjelder alle tre.
    fig.text(0.5, 0.012,
             "Forholdstall måler forsterkningsforskjell, IKKE enighet - to kanaler kan ha "
             "identisk PSD og være ukorrelerte.\nTil enighet: koherens, se "
             "selfnoise.py --compare og docs/koherens.md avsnitt 9.",
             ha="center", fontsize=8, color="0.35")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    print(f"\n  skrev rapport {path}")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("directories", nargs="+", help="øktkatalog(er)")
    ap.add_argument("--decimate-hz", type=float, default=None)
    ap.add_argument("--seglen", type=int, default=None)
    ap.add_argument("--out-dir", default=None,
                    help="hvor figurene skrives (default: øktkatalogen)")
    ap.add_argument("--lag-t", type=float, default=0.95,
                    help="tidskonstant [s] i referansekurven i forholdsplottet "
                         "(0 = utelat; default 0.95, tilpasset på Skjærhalden-øktene)")
    ap.add_argument("--report", default=None,
                    help="samlefigur over ALLE øktene, skrevet til denne stien")
    ap.add_argument("--only-report", action="store_true",
                    help="hopp over figurene per økt og lag bare samlefiguren")
    args = ap.parse_args()

    if args.decimate_hz:
        # Samme rekkefølge som postprocess.main(): BUCKET_MS settes FØR noe leser
        # FS, og FS regnes tilbake fra det avrundede ms-tallet.
        ms = int(round(1000.0 / args.decimate_hz))
        pp.BUCKET_MS, pp.FS = ms, 1000.0 / ms

    sessions = []
    for d in args.directories:
        imu, stamp, directory = pp.resolve_paths(d)
        try:
            cfg = pp.read_kv(os.path.join(directory, f"{stamp}_cfg.csv"))
            pp.RAW_DT_MS = float(cfg["window_ms"]) or None
        except (OSError, KeyError, ValueError):
            pp.RAW_DT_MS = None
        seglen = args.seglen or pp.default_seglen(pp.FS)
        rows = pp.read_imu_rows(imu)
        gps = pp.read_gps_vup(os.path.join(directory, f"{stamp}_gps.csv"))
        # Taper-argumentene sendes med fordi run() krever dem, men de påvirker
        # ingenting her: psd_m/psd_k/psd_v er rå, og taperen brukes bare av
        # spectrum_bins og wave_params, som vi ikke kaller.
        res = pp.run(rows, seglen, 4, "hann", min(pp.WAVE_FMAX, 0.5 * pp.FS),
                     0.1, 0.2, 0.2, 0.05, 0.0, "taper",
                     gps=gps, decimate_mode="fir")
        f, psds = elevation_psds(res, seglen, pp.FS)
        band_table(f, psds, stamp)
        if "GPS" in psds:
            sessions.append(dict(
                stamp=stamp, f=f, psds=psds, fit=fit_lag(f, psds),
                hs=(res["wp_m"]["hs"], res["wp_k"]["hs"],
                    res["wp_gps"]["hs"] if res.get("wp_gps") else 0.0)))
        if args.only_report:
            continue
        out = args.out_dir or directory
        plot_psd(f, psds, stamp, os.path.join(out, f"{stamp}_psd_utapert.png"))
        plot_ratio(f, psds, stamp, os.path.join(out, f"{stamp}_psd_forhold.png"),
                   lag_t=args.lag_t)

    if args.report and sessions:
        plot_report(sessions, args.report, lag_t=args.lag_t)


if __name__ == "__main__":
    main()
