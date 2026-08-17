#!/usr/bin/env python3
"""Kartfigur for én ORB-loggeøkt: driftruta på OpenStreetMap + faktatabell.

Tegner hvor målingen ble gjort og hvordan bøya drev under opptaket, som én
statisk PNG/PDF klar til å limes inn i oppgaven:

    overskrift + beskrivelse   (fra <stamp>_notat.txt)
    kart                       (ekte OSM-fliser, rute, 30 s-punkter, målestokk)
    tabell                     (øktdata fra ses.csv, logge-parametre fra cfg.csv,
                                Hs/Tz/Tc fra ana-fila)

POSISJONEN LIGGER I <stamp>_gps.csv, ikke i _imu.csv - sistnevnte har bare
accel/gyro/quaternioner. Skriptet leser derfor gps-fila (kolonnene rel_ms, lat,
lon, gspeed, hAccuracy, fix) med samme hale-avkutting som postprocess.py, så en
avbrutt _tmp-økt håndteres likt begge steder.

BØLGEPARAMETRE: postprocess.py kjøres FØRST på hver økt (default-parametre, kun
stien settes), og tallene tas så utelukkende fra <stamp>_ana_python.csv. Da er
Hs/Tz/Tc alltid ferske og laget av samme kjede for alle målingene. Firmwarens
egen <stamp>_ana.csv brukes ikke. Hopp over kjøringen med --skip-postprocess
når _ana_python.csv allerede er oppdatert.

OVERSKRIFT OG BESKRIVELSE leses KUN fra <stamp>_notat.txt i øktkatalogen:
    linje 1      -> overskrift, f.eks. "Skjærhalden - Måling 1 - 2026-07-31"
    resten       -> beskrivende tekst, f.eks. "Rolig sjø i vik, lett bris fra SV"
Mangler fila lages overskriften av katalognavn + dato. Lag en stubb med
--init-notat. Oversiktskartet leser tilsvarende oversikt_notat.txt fra
felleskatalogen til øktene.

FLERE MÅLINGER: oppgi n økter på kommandolinja. Da lages først ETT oversiktskart
med alle rutene, hver merket med et nummer, og deretter én figur per måling der
det samme nummeret vises i overskriften og i oversiktsruta. Nummeret er
rekkefølgen på kommandolinja med mindre --nr sier noe annet.

Bruk:
    python3 mapplot.py <sti>                 # <sti> = øktkatalog eller *_gps.csv
    python3 mapplot.py <sti1> <sti2> <sti3>  # oversiktskart + én figur per måling
    python3 mapplot.py <sti1> <sti2> --nr 3,7
    python3 mapplot.py <sti...> --only-overview
    python3 mapplot.py <sti> --init-notat    # skriv notat-stubb og avslutt
    python3 mapplot.py <sti> --format pdf --dpi 300
    python3 mapplot.py <sti> --no-map        # uten flisnedlasting (offline)
    python3 mapplot.py <sti> --tick-sec 60 --label-sec 600
    python3 mapplot.py <sti> --inset-km 5    # bredere oversiktsrute i enkeltkartet
    python3 mapplot.py <sti...> --overview-inset-km 20,200
    python3 mapplot.py <sti...> --skip-postprocess   # ikke regn ut på nytt

INNFELTE RUTER: hvert enkeltkart har én rute på --inset-km (default 2 km) med
hovedkartets utsnitt tegnet inn som ramme. Oversiktskartet har en KASKADE av
ruter, --overview-inset-km (default 10 og 100 km), der hver rute viser hvor
utsnittet under ligger - fra måleområdet og ut til hvor i landet det er.

Flisene hentes fra tile.openstreetmap.org og caches på disk (~/.cache/orb_mapplot),
så gjentatte kjøringer av samme økt ikke laster ned noe. Attribusjonen "©
OpenStreetMap contributors" tegnes inn i figuren - den skal stå der.

Krever: numpy, matplotlib, Pillow. (Ingenting utover det som allerede brukes.)
"""

import argparse
import math
import os
import socket
import subprocess
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone
from glob import glob

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from postprocess import read_kv, read_csv_stream  # noqa: E402  (cfg/ses/ana, og
# hale-regelen for strømmefilene - den skal stå ETT sted, se read_csv_stream)

# --- Standard-økt (test/default hvis ingen sti oppgis) -----------------------
DEFAULT_SESSION = "/home/pif/master/Målinger/m-linger/Skjærhalden/20260731_131527"

# --- Fliser ------------------------------------------------------------------
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
TILE_PX = 256
# OSMs flisepolicy krever en identifiserbar User-Agent. Uten den blir vi blokkert.
USER_AGENT = "ORB-mapplot/1.0 (per.ivar.faust@gmail.com)"
TILE_CACHE = os.path.expanduser("~/.cache/orb_mapplot")
TILE_TIMEOUT = 20            # s per flis
MAX_TILES = 36               # tak på antall fliser -> velger zoom under dette
MAX_ZOOM = 19                # OSM har ikke standardfliser over dette

R_EARTH = 6378137.0          # Web Mercator-sfære (EPSG:3857)

# --- Kartutsnitt -------------------------------------------------------------
PAD_FRAC = 0.12              # marg rundt sporets bbox, andel av største side
MIN_SPAN_M = 200.0           # minste kartutstrekning: en stillestående bøye skal
                             # ikke zoomes inn til meterskala
TRACK_DECIM_HZ = 1.0         # ruta tegnes desimert hit (visuelt likt, mindre fil)
PATH_DECIM_S = 10.0          # tilbakelagt distanse regnes på 10 s-steg: ved 10 Hz
                             # og 0.35 m posisjonsstøy ville rå steg blåst opp
                             # banelengden med titalls prosent

# --- Farger ------------------------------------------------------------------
C_TRACK = "#1f4e9c"
C_TICK = "#1f4e9c"
C_START = "#1a9850"
C_STOP = "#d73027"
C_BADGE = "#b03000"          # nummer-badge: samme farge på oversikt og enkeltplott
C_LABEL = "#12305e"
C_MUTED = "#555555"

METHOD_ROWS = [              # (nøkkel-suffiks i ana-fila, visningsnavn)
    ("madgwick", "Madgwick"),
    ("sflp", "SFLP"),
    ("kalman", "Kalman"),
    ("gps", "GPS"),
    # Utapert GPS er den ene raden som IKKE er båndbegrenset: GPS trenger bare
    # ω⁻² mot IMU-ens ω⁻⁴, så den tåler lavfrekvensenden. Differansen mot
    # GPS-raden over sier hvor mye taperen faktisk fjerner.
    ("gps_utapert", "GPS (untapered)"),
]

# Kurvene i spekterplottet: (kolonne-suffiks i _spec_python.csv, navn, stil, farge)
PSD_CURVES = [
    ("madgwick", "Madgwick", "-", None),
    ("sflp", "SFLP", "-", None),
    ("kalman", "Kalman", "-", None),
    ("gps", "GPS", ":", "#d73027"),
    ("gps_raw", "GPS (untapered)", "--", "#7a1f1f"),
]

DASH = "–"              # tankestrek for manglende verdi


# --- Filoppslag --------------------------------------------------------------
def resolve_session(path):
    """Finn gps.csv + stamp + øktkatalog fra en katalog- eller filsti.
    Speiler resolve_paths() i postprocess.py, men vi trenger gps-fila, ikke imu."""
    if os.path.isdir(path):
        cand = sorted(glob(os.path.join(path, "*_gps.csv")))
        if not cand:
            sys.exit(f"Fant ingen *_gps.csv i {path}")
        gps = cand[0]
    elif os.path.isfile(path):
        base = os.path.basename(path)
        if base.endswith("_imu.csv"):          # praktisk: godta imu-stien òg
            gps = path[: -len("_imu.csv")] + "_gps.csv"
            if not os.path.isfile(gps):
                sys.exit(f"Fant ikke {gps}")
        else:
            gps = path
    else:
        sys.exit(f"Fant ikke {path}")
    directory = os.path.dirname(os.path.abspath(gps))
    stamp = os.path.basename(gps)[: -len("_gps.csv")]
    return gps, stamp, directory


def read_gps_track(gps_path):
    """Les gps.csv -> dict med numpy-arrays. rel_ms ligger på samme millis-akse
    som win_start_ms i imu.csv. Hale-regelen ligger i postprocess.read_csv_stream
    - den samme som read_imu_rows og read_gps_vup bruker - og hva den kuttet blir
    med ut i cols["kutt"], så load_session kan si det i øktblokka si."""
    cols = {}
    idx, rader, kutt = read_csv_stream(gps_path, "rel_ms")
    for nm in ("rel_ms", "lat", "lon"):
        if nm not in idx:
            sys.exit(f"Mangler kolonne '{nm}' i {gps_path}")
    want = [c for c in ("rel_ms", "lat", "lon", "gspeed", "hAccuracy",
                        "fix", "sats", "alt_msl") if c in idx]
    acc = {c: [float(r[idx[c]]) for r in rader] for c in want}
    n_raw = len(acc["rel_ms"])
    if n_raw < 2:
        sys.exit(f"For få GPS-rader i {gps_path}")
    for c in want:
        cols[c] = np.asarray(acc[c], dtype=float)
    cols["t_s"] = cols.pop("rel_ms") / 1000.0

    # Kun 3D-fix tegnes; punkter uten fix ville hoppet vilt rundt i kartet.
    if "fix" in cols:
        ok = cols["fix"] >= 3
    else:
        ok = np.ones(n_raw, dtype=bool)
    n_drop = int(n_raw - ok.sum())
    if ok.sum() < 2:
        sys.exit(f"For få rader med 3D-fix i {gps_path}")
    for c in list(cols):
        cols[c] = cols[c][ok]
    cols["n_raw"] = n_raw
    cols["n_dropped"] = n_drop
    cols["kutt"] = kutt
    return cols


def method_rate(ana, key):
    """" (480 Hz)" til metodenavnet, eller "" når raten ikke er kjent.

    Raten er ikke pynt. Madgwick kjørt på råstrømmen og Madgwick replayet på
    radraten er to ulike tall under samme navn, og SFLP er en tredje rate igjen
    (brikkas egen fusjon). Uten dette leses tabellen som en sammenligning av
    filtre der den delvis er en sammenligning av rater. Tom streng på eldre
    ana-filer, som ikke har nøkkelen - da står navnet som før."""
    try:
        hz = float(ana.get(f"rate_{key}_hz", ""))
    except (TypeError, ValueError):
        return ""
    return f" ({hz:g} Hz)" if hz > 0 else ""


def read_note(directory, stamp, ses):
    """<stamp>_notat.txt -> (overskrift, beskrivelse). Linje 1 er overskrift,
    resten er brødtekst. Mangler fila lages en overskrift av katalognavn + dato."""
    path = note_path(directory, stamp)
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            lines = f.read().splitlines()
        # Hopp over ledende blanke linjer og #-kommentarer fra stubben.
        while lines and (not lines[0].strip() or lines[0].lstrip().startswith("#")):
            lines.pop(0)
        if lines:
            title = lines[0].strip()
            body = "\n".join(lines[1:]).strip()
            return title, body
    parent = os.path.basename(os.path.dirname(os.path.abspath(directory)))
    iso = ses.get("start_utc_iso", "")
    date = iso[:10] if len(iso) >= 10 else stamp[:8]
    return f"{parent} {DASH} {date}", ""


def note_path(directory, stamp):
    return os.path.join(directory, f"{stamp}_notat.txt")


def write_note_stub(directory, stamp):
    path = note_path(directory, stamp)
    if os.path.isfile(path):
        print(f"  {path} finnes allerede - rører den ikke.")
        return path
    parent = os.path.basename(os.path.dirname(os.path.abspath(directory)))
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"{parent} {DASH} Measurement 1 {DASH} {stamp[:4]}-{stamp[4:6]}-{stamp[6:8]}\n")
        f.write("Short description of the conditions, e.g. calm sea in bay, light SW breeze.\n")
    print(f"  skrev {path} - rediger linje 1 (overskrift) og resten (beskrivelse).")
    return path


POSTPROCESS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "postprocess.py")


def run_postprocess(directory, stamp, args, quiet=True):
    """Kjør postprocess.py på økta, så _ana_python.csv og _spec_python.csv
    alltid er ferske og laget av samme kjede med SAMME taper for alle
    målingene - ellers er ikke Hs/Tz/Tc sammenlignbare på tvers av øktene.
    Alt annet står på postprocess sine egne defaults.

    MPLBACKEND=Agg settes i tillegg til at postprocess selv er fikset: en
    GUI-backend som popper opp et vindu er unødvendig når vi kaller den som
    subprosess."""
    print(f"  kjører postprocess.py {stamp} "
          f"(taper {args.taper_f1:g}-{args.taper_f2:g} Hz) ...", flush=True)
    env = dict(os.environ, MPLBACKEND="Agg")
    res = subprocess.run([sys.executable, POSTPROCESS, directory,
                          "--taper-f1", str(args.taper_f1),
                          "--taper-f2", str(args.taper_f2),
                          "--vacc-source", args.vacc_source]
                         + (["--raw", args.raw] if args.raw else []),
                         env=env, capture_output=True, text=True)
    if res.returncode != 0:
        tail = "\n".join((res.stdout + res.stderr).strip().splitlines()[-15:])
        sys.exit(f"postprocess.py feilet for {stamp} "
                 f"(exit {res.returncode}):\n{tail}")
    if not quiet:
        print(res.stdout)


def read_spec(directory, stamp):
    """Les <stamp>_spec_python.csv -> dict {kolonnenavn: numpy-array}.
    Skrevet av postprocess.py i samme kjøring som _ana_python.csv, så spekteret
    og Hs/Tz/Tc i tabellen hører garantert sammen."""
    path = os.path.join(directory, f"{stamp}_spec_python.csv")
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        header = f.readline().rstrip("\n").split(",")
        acc = {nm: [] for nm in header}
        for line in f:
            fld = line.rstrip("\n").split(",")
            if len(fld) < len(header):
                break
            try:
                vals = [float(v) for v in fld[:len(header)]]
            except ValueError:
                break
            for nm, v in zip(header, vals):
                acc[nm].append(v)
    if len(acc.get("f_hz", [])) < 2:
        return None
    return {nm: np.asarray(v, dtype=float) for nm, v in acc.items()}


def pick_ana(directory, stamp):
    """Bølgeparametrene tas KUN fra _ana_python.csv - den er skrevet av
    postprocess.py rett før, med analysevalgene som gjelder her. Firmwarens
    egen _ana.csv brukes ikke."""
    py = os.path.join(directory, f"{stamp}_ana_python.csv")
    if os.path.isfile(py):
        return read_kv(py), os.path.basename(py)
    return {}, None


# --- Web Mercator ------------------------------------------------------------
def lonlat_to_merc(lon, lat):
    x = R_EARTH * np.radians(lon)
    y = R_EARTH * np.log(np.tan(np.pi / 4.0 + np.radians(lat) / 2.0))
    return x, y


def merc_to_lonlat(x, y):
    lon = np.degrees(x / R_EARTH)
    lat = np.degrees(2.0 * np.arctan(np.exp(y / R_EARTH)) - np.pi / 2.0)
    return lon, lat


def merc_per_metre(lat_deg):
    """Web Mercator strekker avstander med 1/cos(lat). Én meter på bakken er
    altså 1/cos(lat) Mercator-enheter - brukes til målestokk og marger."""
    return 1.0 / math.cos(math.radians(lat_deg))


def merc_world(zoom):
    """Verdenens bredde i Mercator-enheter per flis-piksel ved gitt zoom."""
    return 2.0 * math.pi * R_EARTH / (TILE_PX * (2 ** zoom))


def merc_to_pixel(x, y, zoom):
    half = math.pi * R_EARTH
    scale = TILE_PX * (2 ** zoom) / (2.0 * half)
    return (x + half) * scale, (half - y) * scale


def pick_zoom(bbox, max_tiles=MAX_TILES, zmax=MAX_ZOOM):
    """Største zoom der bboxen dekkes av <= max_tiles fliser."""
    x0, y0, x1, y1 = bbox
    for z in range(zmax, 0, -1):
        px0, py1 = merc_to_pixel(x0, y0, z)     # y snus
        px1, py0 = merc_to_pixel(x1, y1, z)
        nx = int(px1 // TILE_PX) - int(px0 // TILE_PX) + 1
        ny = int(py1 // TILE_PX) - int(py0 // TILE_PX) + 1
        if nx * ny <= max_tiles:
            return z
    return 1


def prefer_ipv4():
    """Sorter getaddrinfo slik at IPv4 prøves først.

    tile.openstreetmap.org svarer på både A og AAAA. urllib gjør IKKE Happy
    Eyeballs slik curl gjør: den tar adressene i den rekkefølgen getaddrinfo
    gir dem, og på et nett uten fungerende IPv6-rute betyr det 20 s timeout
    FØR hver eneste flis faller tilbake til IPv4 (målt: 20.6 s vs 0.53 s).
    Vi fjerner ikke IPv6 - bare flytter det bakerst, så det fortsatt brukes
    hvis IPv4 skulle feile."""
    orig = socket.getaddrinfo

    def sorted_getaddrinfo(*args, **kwargs):
        res = orig(*args, **kwargs)
        return sorted(res, key=lambda r: 0 if r[0] == socket.AF_INET else 1)

    socket.getaddrinfo = sorted_getaddrinfo


def fetch_tile(z, x, y, url_tmpl, cache_dir):
    """Én flis, disk-cachet. Returnerer PIL-bilde eller None ved feil."""
    from PIL import Image
    n = 2 ** z
    if not (0 <= x < n and 0 <= y < n):
        return None
    path = os.path.join(cache_dir, str(z), str(x), f"{y}.png")
    if not os.path.isfile(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        req = urllib.request.Request(
            url_tmpl.format(z=z, x=x, y=y), headers={"User-Agent": USER_AGENT})
        with urllib.request.urlopen(req, timeout=TILE_TIMEOUT) as resp:
            data = resp.read()
        tmp = path + ".part"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, path)                  # aldri en halv flis i cachen
    return Image.open(path).convert("RGB")


def build_basemap(bbox, zoom, url_tmpl=TILE_URL, cache_dir=TILE_CACHE, quiet=False):
    """Sy sammen flisene som dekker bbox -> (numpy-bilde, extent i Mercator).
    Returnerer (None, None) hvis flisene ikke kan hentes."""
    from PIL import Image
    x0, y0, x1, y1 = bbox
    px0, py1 = merc_to_pixel(x0, y0, zoom)
    px1, py0 = merc_to_pixel(x1, y1, zoom)
    tx0, tx1 = int(px0 // TILE_PX), int(px1 // TILE_PX)
    ty0, ty1 = int(py0 // TILE_PX), int(py1 // TILE_PX)
    nx, ny = tx1 - tx0 + 1, ty1 - ty0 + 1
    canvas = Image.new("RGB", (nx * TILE_PX, ny * TILE_PX), (238, 238, 233))
    n_net = 0
    try:
        for i, tx in enumerate(range(tx0, tx1 + 1)):
            for j, ty in enumerate(range(ty0, ty1 + 1)):
                cached = os.path.isfile(
                    os.path.join(cache_dir, str(zoom), str(tx), f"{ty}.png"))
                img = fetch_tile(zoom, tx, ty, url_tmpl, cache_dir)
                if img is not None:
                    canvas.paste(img, (i * TILE_PX, j * TILE_PX))
                    n_net += 0 if cached else 1
    except (urllib.error.URLError, urllib.error.HTTPError, OSError) as exc:
        print(f"  ADVARSEL: klarte ikke hente kartfliser ({exc}) - tegner uten bakgrunnskart.")
        return None, None
    if not quiet:
        print(f"  fliser: {nx}x{ny} @ z{zoom} ({n_net} lastet ned, "
              f"{nx * ny - n_net} fra cache)")
    # Extent = flisrutenettets ytterkanter i Mercator-enheter.
    step = merc_world(zoom) * TILE_PX
    half = math.pi * R_EARTH
    ex0 = -half + tx0 * step
    ex1 = -half + (tx1 + 1) * step
    ey1 = half - ty0 * step
    ey0 = half - (ty1 + 1) * step
    return np.asarray(canvas), (ex0, ex1, ey0, ey1)


# --- Statistikk --------------------------------------------------------------
def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dp = p2 - p1
    dl = np.radians(lon2 - lon1)
    a = np.sin(dp / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dl / 2) ** 2
    return 2.0 * R_EARTH * np.arcsin(np.sqrt(np.clip(a, 0.0, 1.0)))


def track_stats(gps):
    """Banelengde, netto drift, middelposisjon og fart."""
    t, lat, lon = gps["t_s"], gps["lat"], gps["lon"]
    # Banelengden på 10 s-steg: rå 10 Hz-steg er dominert av posisjonsstøyen
    # (~0.35 m) og ville gitt en kraftig overdrevet distanse.
    grid = np.arange(t[0], t[-1] + PATH_DECIM_S, PATH_DECIM_S)
    k = np.searchsorted(t, grid)
    k = np.unique(np.clip(k, 0, len(t) - 1))
    dist = float(np.sum(haversine(lat[k[:-1]], lon[k[:-1]], lat[k[1:]], lon[k[1:]])))
    net = float(haversine(lat[0], lon[0], lat[-1], lon[-1]))
    brg = math.degrees(math.atan2(
        math.sin(math.radians(lon[-1] - lon[0])) * math.cos(math.radians(lat[-1])),
        math.cos(math.radians(lat[0])) * math.sin(math.radians(lat[-1]))
        - math.sin(math.radians(lat[0])) * math.cos(math.radians(lat[-1]))
        * math.cos(math.radians(lon[-1] - lon[0])))) % 360.0
    out = dict(dist=dist, net=net, bearing=brg,
               lat_mid=float(np.mean(lat)), lon_mid=float(np.mean(lon)))
    if "gspeed" in gps:
        out["v_mean"] = float(np.mean(gps["gspeed"]))
        out["v_max"] = float(np.max(gps["gspeed"]))
    if "hAccuracy" in gps:
        out["hacc"] = float(np.median(gps["hAccuracy"]))
    if "sats" in gps:
        out["sats"] = float(np.median(gps["sats"]))
    return out


def fit_bbox(bbox, ratio):
    """Blås opp bboxen til nøyaktig høyde/bredde = ratio, om senteret.
    Kartaksen har den formen, og siden aspect="equal" da allerede er oppfylt
    slipper vi at matplotlib krymper aksen og etterlater hvit luft."""
    x0, y0, x1, y1 = bbox
    w, h = x1 - x0, y1 - y0
    if h / w > ratio:
        w = h / ratio
    else:
        h = w * ratio
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    return cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2


def map_bbox(lat, lon, pad_frac=PAD_FRAC, min_span_m=MIN_SPAN_M):
    """Kvadratisk-ish Mercator-bbox rundt sporet, med marg og minsteutstrekning."""
    x, y = lonlat_to_merc(lon, lat)
    x0, x1, y0, y1 = x.min(), x.max(), y.min(), y.max()
    lat_mid = float(np.mean(lat))
    min_span = min_span_m * merc_per_metre(lat_mid)
    dx, dy = max(x1 - x0, min_span), max(y1 - y0, min_span)
    pad = pad_frac * max(dx, dy)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    return (cx - dx / 2 - pad, cy - dy / 2 - pad,
            cx + dx / 2 + pad, cy + dy / 2 + pad)


# --- Formattering ------------------------------------------------------------
def fmt_val(d, key, unit="", scale=None, dec=None):
    """Hent key fra en cfg/ses/ana-dict og formatér, eller returner tankestrek."""
    raw = d.get(key)
    if raw is None or raw == "":
        return DASH
    try:
        v = float(raw)
    except ValueError:
        return str(raw)
    if scale:
        v *= scale
    if dec is None:
        txt = f"{v:g}"
    else:
        txt = f"{v:.{dec}f}"
    return f"{txt}{unit}"


def joined(parts, template):
    """Sett sammen en verdi av flere cfg-felt, men gi én enkel tankestrek hvis
    ingen av dem finnes - "± – g / ± – °/s" er bare støy."""
    if all(p == DASH for p in parts):
        return DASH
    return template.format(*parts)


def fmt_segment(ana, cfg):
    """Welch-segmentet som «1024 (102.4 s), 75% overlap».

    Antall bins alene sier ingenting om hvor lang tid segmentet dekker - det
    avhenger av samplingsraten etter desimering (vacc_fs_hz, evt. utledet av
    vacc_bucket_ms), og overlappen av welch_overlap_div: div=4 => 75 %.

    ana-fila går ALLTID foran cfg.csv: analyseparametrene lå i cfg fram til
    build 66, men den beskriver hvordan imu.csv ble til, ikke hvordan den ble
    tolket. Kjører postprocess.py med en annen taper eller seglengde enn
    firmware, er det ana-fila som har fasit. cfg beholdes som fallback for de
    øktene som alt er logget."""
    def lookup(key, conv=float):
        for src in (ana, cfg):
            if src.get(key):
                try:
                    v = conv(float(src[key]))
                except (ValueError, ZeroDivisionError):
                    continue
                if v:
                    return v
        return None

    n = lookup("welch_seglen", int)
    if not n:
        return None
    fs = lookup("vacc_fs_hz") or lookup("vacc_bucket_ms", lambda v: 1000.0 / v)
    txt = f"Segment {n}"
    if fs:
        secs = f"{n / fs:.1f}".removesuffix(".0")
        txt += f" ({secs} s)"
    div = lookup("welch_overlap_div")
    if div and div > 0:
        txt += f", {(1.0 - 1.0 / div) * 100:.0f}% overlap"
    return txt


def fmt_dur(ms):
    try:
        s = int(round(float(ms) / 1000.0))
    except (TypeError, ValueError):
        return DASH
    return f"{s // 3600:d}:{(s % 3600) // 60:02d}:{s % 60:02d}"


def fmt_time(iso):
    """'2026-07-31T13:15:28Z' -> ('2026-07-31 13:15:28 UTC', '15:15:28 lokal')."""
    if not iso:
        return DASH, None
    try:
        dt = datetime.strptime(iso, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except ValueError:
        return iso, None
    utc = dt.strftime("%Y-%m-%d %H:%M:%S")
    try:
        from zoneinfo import ZoneInfo
        loc = dt.astimezone(ZoneInfo("Europe/Oslo")).strftime("%H:%M:%S")
    except Exception:
        loc = None
    return utc, loc


def fmt_latlon(lat, lon):
    ns, ew = ("N" if lat >= 0 else "S"), ("E" if lon >= 0 else "W")
    return f"{abs(lat):.5f}°{ns}  {abs(lon):.5f}°{ew}"


def nice_scale(span_m):
    """Pen målestokklengde (1/2/5-serie) på ca. 25 % av kartbredden."""
    target = span_m * 0.25
    exp = math.floor(math.log10(target))
    for mult in (1, 2, 5, 10):
        val = mult * 10 ** exp
        if val >= target:
            return val
    return 10 ** (exp + 1)


# --- Tabellinnhold -----------------------------------------------------------
def build_blocks(cfg, ses, ana, stats):
    """Radene i «Økt»- og «Logging»-blokkene + bølgetabellen."""
    t_start, l_start = fmt_time(ses.get("start_utc_iso", ""))
    # Sluttidspunktet er redundant når både start og varighet står der, og
    # kolonnen er smal - så det er droppet.
    okt = [
        ("Date", t_start.split()[0] if t_start != DASH else DASH),
        ("Start (UTC)", t_start.split()[-1] if t_start != DASH else DASH),
    ]
    if l_start:
        okt.append(("Start (local)", l_start))
    okt += [
        ("Duration", fmt_dur(ses.get("duration_ms"))),
        ("Position (mean)", fmt_latlon(stats["lat_mid"], stats["lon_mid"])),
        ("Path length (10 s)", f"{stats['dist']:.0f} m"),
        ("Net drift", f"{stats['net']:.0f} m at {stats['bearing']:.0f}°"),
        ("Speed mean/max",
         f"{stats['v_mean']:.2f} / {stats['v_max']:.2f} m/s"
         if "v_mean" in stats else DASH),
        ("GPS quality",
         f"hAcc {stats['hacc']:.2f} m · {stats.get('sats', 0):.0f} sats"
         if "hacc" in stats else DASH),
        # imu_rows hører til analysen og ligger i _ana.csv (og i _ana_python.csv,
        # som er den ana-dicten her) fra og med raw_write-firmwaren. Eldre økter
        # har den i ses.csv, så den leses derfra når den finnes.
        ("Rows IMU / GPS",
         joined([fmt_val(ses if "imu_rows" in ses else ana, "imu_rows"),
                 fmt_val(ses, "gps_rows")], "{} / {}")),
    ]

    build = fmt_val(cfg, "build_seq")
    if build != DASH and cfg.get("build_date"):
        build += f"  ({cfg['build_date']})"
    logg = [
        # Satt mot faktisk ODR, ikke acc mot gyro: de to sensorene kjører på
        # samme ODR, så den kolonnen sa det samme to ganger. Den FAKTISKE raten
        # er derimot ikke utledbar av noe annet i tabellen - den regnes av antall
        # råsamples per rad (n) i postprocess, og på 20260813 er den 464 Hz mot
        # 480 satt. Avviket er FIFO-en som ikke rekker rundt, og det er en av de
        # tingene man vil se på en figur uten å måtte kjøre rawlog.
        ("ODR set / actual",
         joined([fmt_val(cfg, "imu_odr_hz"),
                 fmt_val(ana, "imu_odr_actual_hz", dec=1)], "{} / {} Hz")),
        ("Output rate / window",
         joined([fmt_val(cfg, "output_rate_hz"), fmt_val(cfg, "window_ms")],
                "{} Hz / {} ms")),
        ("Full-scale",
         joined([fmt_val(cfg, "accel_fs_g"), fmt_val(cfg, "gyro_fs_dps")],
                "±{} g / ±{} °/s")),
        ("LPF acc / gyro",
         joined([fmt_val(cfg, "acc_lpf_hz", dec=0),
                 fmt_val(cfg, "gyro_lpf_hz", dec=0)], "{} / {} Hz")),
        ("SFLP ODR", fmt_val(cfg, "sflp_odr_hz", " Hz", dec=0)),
        ("GPS rate", fmt_val(cfg, "gps_rate_hz", " Hz")),
        ("Log duration (set)", fmt_val(cfg, "log_duration_sec", " s")),
        ("Build", build),
    ]

    bolge = []
    for key, name in METHOD_ROWS:
        row = [name + method_rate(ana, key)]
        for p in ("Hs", "Tz", "Tc"):
            row.append(fmt_val(ana, f"{p}_{key}", dec=(3 if p == "Hs" else 2)))
        bolge.append(row)
    return okt, logg, bolge


# --- Tegning -----------------------------------------------------------------
def corner_load(x, y, bbox):
    """Hvor mange sporpunkter ligger i hvert av kartets fire hjørner?
    Brukes til å legge oversiktskart, nordpil og målestokk der de ikke dekker
    ruta - hvilket hjørne som er ledig varierer fra økt til økt."""
    fx = (x - bbox[0]) / (bbox[2] - bbox[0])
    fy = (y - bbox[1]) / (bbox[3] - bbox[1])
    rects = {"tl": (0.00, 0.54, 0.34, 1.00), "tr": (0.66, 0.54, 1.00, 1.00),
             "bl": (0.00, 0.00, 0.34, 0.46), "br": (0.66, 0.00, 1.00, 0.46)}
    return {k: int(np.sum((fx >= r[0]) & (fx <= r[2])
                          & (fy >= r[1]) & (fy <= r[3])))
            for k, r in rects.items()}


def place_decorations(load):
    """Fordel oversiktskart / nordpil / målestokk på de minst opptatte hjørnene."""
    free = sorted(load, key=lambda k: (load[k], k))
    inset = free[0]
    top = [k for k in free if k.startswith("t") and k != inset] or ["tl"]
    bot = [k for k in free if k.startswith("b") and k != inset] or ["bl"]
    return inset, top[0], bot[0]


def draw_map(ax, gps, stats, args, bbox, quiet=False):
    import matplotlib.patheffects as pe

    lat, lon, t = gps["lat"], gps["lon"], gps["t_s"]
    zoom = args.zoom if args.zoom else pick_zoom(bbox)

    if not args.no_map:
        img, extent = build_basemap(bbox, zoom, args.tile_url, args.tile_cache, quiet)
        if img is not None:
            ax.imshow(img, extent=extent, origin="upper", interpolation="bilinear",
                      zorder=0)
        else:
            args.no_map = True
    if args.no_map:
        ax.set_facecolor("#eef2f5")   # nøytral bakgrunn; målestokken gir skalaen

    ax.set_xlim(bbox[0], bbox[2])
    ax.set_ylim(bbox[1], bbox[3])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_edgecolor("#8a8a8a")
        s.set_linewidth(0.8)

    halo = [pe.Stroke(linewidth=2.6, foreground="white"), pe.Normal()]
    x, y = lonlat_to_merc(lon, lat)

    # Ruta desimeres til ~1 Hz: ved noen hundre meters utstrekning er det
    # visuelt identisk med 10 Hz, men gir en langt lettere PDF.
    dt_med = float(np.median(np.diff(t))) if len(t) > 2 else 0.1
    step = max(1, int(round(1.0 / (TRACK_DECIM_HZ * max(dt_med, 1e-3)))))
    ax.plot(x[::step], y[::step], "-", color=C_TRACK, lw=1.6, zorder=3,
            solid_capstyle="round", path_effects=halo, label="Drift track")

    # Punkter hvert tick_sec: avstanden mellom dem leses direkte som fart.
    def at_times(period):
        if period <= 0:
            return np.array([], dtype=int)
        grid = np.arange(t[0], t[-1] + 1e-9, period)
        k = np.searchsorted(t, grid)
        return np.unique(np.clip(k, 0, len(t) - 1))

    kt = at_times(args.tick_sec)
    ax.plot(x[kt], y[kt], "o", ms=3.4, mfc=C_TICK, mec="white", mew=0.7,
            ls="none", zorder=4, label=f"every {args.tick_sec:g} s")

    kl = at_times(args.label_sec)
    if len(kl):
        ax.plot(x[kl], y[kl], "o", ms=5.6, mfc="white", mec=C_TICK, mew=1.4,
                ls="none", zorder=5)
        span = bbox[2] - bbox[0]
        for k in kl:
            el = t[k] - t[0]
            if el < 1.0 or (t[-1] - t[k]) < args.label_sec * 0.4:
                continue                        # ikke rot til start/slutt-markørene
            ax.text(x[k] + 0.012 * span, y[k] + 0.008 * span,
                    f"{int(el) // 60:02d}:{int(el) % 60:02d}",
                    fontsize=6.5, color=C_LABEL, ha="left", va="bottom", zorder=6,
                    path_effects=[pe.Stroke(linewidth=2.0, foreground="white"),
                                  pe.Normal()])

    ax.plot(x[0], y[0], "o", ms=9, mfc=C_START, mec="white", mew=1.4,
            ls="none", zorder=7, label="Start")
    ax.plot(x[-1], y[-1], "s", ms=9, mfc=C_STOP, mec="white", mew=1.4,
            ls="none", zorder=7, label="End")

    load = corner_load(x[::step], y[::step], bbox)
    inset_c, north_c, scale_c = place_decorations(load)
    draw_scalebar(ax, bbox, stats["lat_mid"], scale_c)
    draw_north(ax, bbox, north_c)
    # Legenden tegnes UNDER kartet (av make_figure). Inne i kartet ville den
    # måttet konkurrere med spor, målestokk og oversiktskart om plassen.
    # Zoomnivået returneres ikke lenger - eneste bruker var attribusjonsteksten.
    return inset_c, ax.get_legend_handles_labels()


def draw_scalebar(ax, bbox, lat_mid, corner="bl"):
    """Målestokk i METER - Mercator-enhetene deles på 1/cos(lat)."""
    import matplotlib.patheffects as pe
    mpm = merc_per_metre(lat_mid)
    w, h_ax = bbox[2] - bbox[0], bbox[3] - bbox[1]
    span_m = w / mpm
    bar_m = nice_scale(span_m)
    bar = bar_m * mpm
    x0 = (bbox[0] + 0.045 * w) if corner == "bl" else (bbox[2] - 0.045 * w - bar)
    y0 = bbox[1] + 0.045 * h_ax
    h = 0.008 * h_ax
    halo = [pe.Stroke(linewidth=3.0, foreground="white"), pe.Normal()]
    ax.plot([x0, x0 + bar], [y0, y0], "-", color="black", lw=1.8, zorder=8,
            path_effects=halo)
    for xx in (x0, x0 + bar):
        ax.plot([xx, xx], [y0 - h, y0 + h], "-", color="black", lw=1.8, zorder=8,
                path_effects=halo)
    # Etiketten står PÅ LINJE med stolpen, ikke over den: over den ville den
    # kollidert med spor eller start/slutt-markør i trange utsnitt. Den legges
    # på innsiden av stolpen, ellers klippes den av kartkanten.
    label = f"{bar_m:.0f} m" if bar_m < 1000 else f"{bar_m / 1000:g} km"
    if corner == "bl":
        ax.text(x0 + bar + 0.012 * w, y0, label, fontsize=7, ha="left",
                va="center", zorder=8, path_effects=halo)
    else:
        ax.text(x0 - 0.012 * w, y0, label, fontsize=7, ha="right",
                va="center", zorder=8, path_effects=halo)


def draw_north(ax, bbox, corner="tl"):
    import matplotlib.patheffects as pe
    halo = [pe.Stroke(linewidth=3.0, foreground="white"), pe.Normal()]
    frac = 0.05 if corner == "tl" else 0.95
    x = bbox[0] + frac * (bbox[2] - bbox[0])
    y0 = bbox[1] + 0.78 * (bbox[3] - bbox[1])
    y1 = y0 + 0.10 * (bbox[3] - bbox[1])
    ax.annotate("", xy=(x, y1), xytext=(x, y0), zorder=8,
                arrowprops=dict(arrowstyle="-|>", color="black", lw=1.6,
                                path_effects=halo))
    ax.text(x, y1 + 0.012 * (bbox[3] - bbox[1]), "N", fontsize=8.5, weight="bold",
            ha="center", va="bottom", zorder=8, path_effects=halo)


INSET_RECT = {"tl": [0.012, 0.600, 0.235, 0.388],
              "tr": [0.753, 0.600, 0.235, 0.388],
              "bl": [0.012, 0.012, 0.235, 0.388],
              "br": [0.753, 0.012, 0.235, 0.388]}


def draw_badge(ax, x, y, nr, size=9.0, fs=7.5, zorder=10):
    """Nummerert markør. Samme utseende på oversiktskartet og på hver
    enkeltmåling, så måling nr. N er umiddelbart gjenkjennelig begge steder."""
    ax.plot([x], [y], marker="o", ms=size, mfc=C_BADGE, mec="white", mew=1.3,
            ls="none", zorder=zorder)
    ax.text(x, y, str(nr), fontsize=fs, weight="bold", color="white",
            ha="center", va="center", zorder=zorder + 1)


def draw_scale_inset(ax_map, cx, cy, lat_mid, span_km, rect, args,
                     frame_bbox=None, badge_nr=None):
    """Som draw_on_inset, men lager aksen som en innfelling oppi ax_map."""
    return draw_on_inset(ax_map.inset_axes(rect), cx, cy, lat_mid, span_km,
                         args, frame_bbox=frame_bbox, badge_nr=badge_nr)


def draw_on_inset(ax, cx, cy, lat_mid, span_km, args,
                  frame_bbox=None, badge_nr=None):
    """Tegn en oversiktsrute på span_km × span_km rundt (cx, cy) i EN GITT akse.

    frame_bbox tegnes som rød ramme oppi - det er utsnittet nivået under viser.
    Returnerer rutas egen bbox, så flere ruter kan kaskaderes: hver viser hvor
    den forrige ligger. Returnerer None hvis flisene ikke kunne hentes."""
    half = span_km * 1000.0 * merc_per_metre(lat_mid) / 2.0
    bbox = (cx - half, cy - half, cx + half, cy + half)
    zoom = pick_zoom(bbox, max_tiles=16)
    img, extent = build_basemap(bbox, zoom, args.tile_url, args.tile_cache, quiet=True)
    if img is None:
        ax.remove()
        return None
    ax.imshow(img, extent=extent, origin="upper", interpolation="bilinear")
    ax.set_xlim(bbox[0], bbox[2])
    ax.set_ylim(bbox[1], bbox[3])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    if frame_bbox is not None:
        import matplotlib.patches as mpatches
        w = frame_bbox[2] - frame_bbox[0]
        h = frame_bbox[3] - frame_bbox[1]
        # Er utsnittet under så lite at ramma blir noen få piksler, tegnes en
        # prikk i stedet - en usynlig ramme er verre enn ingen ramme.
        if w < 0.045 * (bbox[2] - bbox[0]):
            ax.plot([(frame_bbox[0] + frame_bbox[2]) / 2],
                    [(frame_bbox[1] + frame_bbox[3]) / 2], marker="o", ms=5,
                    mfc=C_STOP, mec="white", mew=1.0, zorder=5)
        else:
            ax.add_patch(mpatches.Rectangle(
                (frame_bbox[0], frame_bbox[1]), w, h, fill=False, ec=C_STOP,
                lw=1.2, zorder=5))
    if badge_nr:
        draw_badge(ax, cx, cy, badge_nr, size=11, fs=7.5)
    ax.text(0.5, 0.015, f"{span_km:g} km", transform=ax.transAxes,
            fontsize=5.8, color="#222222", ha="center", va="bottom",
            bbox=dict(fc="white", ec="none", alpha=0.75, pad=1.0), zorder=8)
    for s in ax.spines.values():
        s.set_edgecolor("#444444")
        s.set_linewidth(1.0)
    return bbox


def draw_kv_blocks(ax, blocks, w_key=0.42, rows_total=None):
    """Én kolonne med nøkkel/verdi-blokker under hverandre: [(tittel, rader)].
    rows_total gir felles radhøyde på tvers av flere akser."""
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    n = rows_total or sum(len(r) + 1.5 for _, r in blocks)
    dy = 1.0 / n
    y = 1.0
    for title, rows in blocks:
        ax.text(0, y, title, fontsize=8.0, weight="bold", color="#222222",
                va="top")
        for k, v in rows:
            y -= dy
            ax.text(0, y, k, fontsize=7.0, color=C_MUTED, va="top")
            ax.text(w_key, y, v, fontsize=7.0, color="#111111", va="top",
                    family="DejaVu Sans Mono")
        y -= 1.5 * dy


def draw_wave_table(ax, bolge, ana_src, ana, cfg, args):
    """Hs/Tz/Tc per metode + hvilken taper, seglengde og kilde tallene kommer fra."""
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    dy = 1.0 / (len(bolge) + 5.6)
    ax.text(0, 1.0, "Wave parameters", fontsize=8.0, weight="bold",
            color="#222222", va="top")
    cols = [0.0, 0.36, 0.58, 0.79]
    yy = 1.0 - 1.25 * dy
    for cx_, h in zip(cols, ["Method", "Hs [m]", "Tz [s]", "Tc [s]"]):
        ax.text(cx_, yy, h, fontsize=7.0, color="#222222", weight="bold", va="top")
    for i, row in enumerate(bolge):
        yy = 1.0 - (i + 2.35) * dy
        ax.text(cols[0], yy, row[0], fontsize=7.0, color=C_MUTED, va="top")
        for cx_, v in zip(cols[1:], row[1:]):
            ax.text(cx_, yy, v, fontsize=7.0, color="#111111", va="top",
                    family="DejaVu Sans Mono")
    # Taperen er selve premisset for tallene over: samme f1/f2 for alle
    # målingene er det som gjør Hs/Tz/Tc sammenlignbare på tvers.
    foot = [f"Taper {args.taper_f1:g}–{args.taper_f2:g} Hz"]
    seg = fmt_segment(ana, cfg)
    if seg:
        foot.append(seg)
    # Raten står nå i parentes per rad, så fotnoten trenger bare å si hva den
    # raten BETYR - at filteret kjørte på råstrømmen, ikke på radene. Leses fra
    # ana-fila og ikke fra args: med --skip-postprocess kan de to være ulike, og
    # da er fila fasiten.
    fra_raa = sorted(navn for key, navn in METHOD_ROWS
                     if str(ana.get(f"src_{key}", "")).startswith("raw"))
    if fra_raa:
        foot.append(f"{', '.join(fra_raa)} ran on the raw stream; "
                    f"SFLP is the on-chip fusion. Rates in parentheses.")
    elif ana.get("vacc_source") == "csv":
        foot.append("All filters replayed at row rate (--vacc-source csv)")
    foot.append(f"Source: {ana_src}" if ana_src else "No _ana_python.csv found")
    if args.skip_postprocess:
        foot.append("(--skip-postprocess: taper not verified against file)")
    for i, line in enumerate(foot):
        ax.text(0, 1.0 - (len(bolge) + 2.6 + 0.8 * i) * dy, line, fontsize=6.2,
                color=C_MUTED, va="top", style="italic")


def draw_psd(ax, spec, args, ana=None):
    """Elevasjonsspekteret S_eta(f) i log-log, én kurve per metode.

    Log-log er ikke pynt: bølgebåndet spenner en dekade, og et f⁻⁴-gulv fra
    dobbelintegrasjonen blir en RETT LINJE her - det er selve diagnosen på om
    lavfrekvensenden er sjø eller støy. Taper-rampen skyggelegges, så det er
    synlig hvilken del av spekteret som er dempet bort."""
    ax.set_xscale("log")
    ax.set_yscale("log")
    if spec is None:
        ax.text(0.5, 0.5, "No _spec_python.csv", fontsize=8, color=C_MUTED,
                ha="center", va="center", transform=ax.transAxes)
        ax.set_xticks([])
        ax.set_yticks([])
        return
    import matplotlib.ticker as mticker
    f = spec["f_hz"]
    # Under taperens f1 er alt nullet bort - det er ingen informasjon der, bare
    # tom akse. Grafen starter derfor på f1 med mindre --psd-fmin sier noe annet.
    fmin = args.psd_fmin if args.psd_fmin else args.taper_f1
    band = (f >= fmin) & (f <= args.psd_fmax)
    any_line = False
    for key, name, style, color in PSD_CURVES:
        col = f"psd_eta_{key}"
        if col not in spec:
            continue
        y = np.where(spec[col] > 0.0, spec[col], np.nan)   # 0 => utenfor taper
        # Samme rate-merking som i tabellen: kurvene skal kunne leses uten å
        # først slå opp hvilken av dem som kjørte på hvilken rate.
        merket = name + (method_rate(ana, key) if ana else "")
        ax.plot(f[band], y[band], style, lw=1.3, label=merket,
                **({"color": color} if color else {}))
        any_line = True
    ax.axvspan(args.taper_f1, args.taper_f2, color="orange", alpha=0.16, lw=0)
    ax.set_xlim(fmin, args.psd_fmax)
    ax.set_xlabel("frequency [Hz]", fontsize=7.5)
    ax.set_ylabel("$S_\\eta$  [m²/Hz]", fontsize=7.5)
    ax.tick_params(labelsize=6.8)
    ax.grid(True, which="both", alpha=0.25, lw=0.5)
    # Tall framfor 10⁻¹-notasjon på x. Major og minor må ha hver sine ticks,
    # ellers merkes samme punkt to ganger og etikettene legger seg oppå hverandre.
    ax.xaxis.set_major_locator(mticker.LogLocator(base=10.0, subs=(1.0,)))
    ax.xaxis.set_minor_locator(mticker.LogLocator(base=10.0, subs=(2.0, 5.0)))

    def _hz(v, _pos=None):
        if v <= 0.0:
            return ""
        return f"{v:.{max(0, int(math.ceil(-math.log10(v))))}f}"

    def _hz_minor(v, _pos=None):
        # Et minor-tick som lander på en dekade ville fått etikett fra BEGGE
        # formattererne, altså samme frekvens skrevet to ganger oppå hverandre.
        if v <= 0.0 or abs(math.log10(v) - round(math.log10(v))) < 1e-9:
            return ""
        return _hz(v)

    ax.xaxis.set_major_formatter(mticker.FuncFormatter(_hz))
    ax.xaxis.set_minor_formatter(mticker.FuncFormatter(_hz_minor))
    ax.tick_params(axis="x", which="minor", labelsize=5.8)
    if any_line:
        ax.legend(fontsize=5.8, framealpha=0.85, borderpad=0.35,
                  handletextpad=0.5, labelspacing=0.28, loc="best")
    ax.set_title("Elevation spectrum", fontsize=8.0, weight="bold", loc="left",
                 color="#222222", pad=3)


def load_session(path, args, nr=None):
    """Les alt som trengs om én økt -> dict. Brukes både av enkeltplottene og
    av oversiktskartet, så tallene garantert er de samme begge steder."""
    gps_path, stamp, directory = resolve_session(path)
    if not args.skip_postprocess:
        run_postprocess(directory, stamp, args)
    ses = read_kv(os.path.join(directory, f"{stamp}_ses.csv"))
    cfg = read_kv(os.path.join(directory, f"{stamp}_cfg.csv"))
    ana, ana_src = pick_ana(directory, stamp)
    gps = read_gps_track(gps_path)
    title, body = read_note(directory, stamp, ses)
    return dict(nr=nr, stamp=stamp, directory=directory, gps=gps,
                stats=track_stats(gps), ses=ses, cfg=cfg, ana=ana,
                ana_src=ana_src, spec=read_spec(directory, stamp),
                title=title, body=body,
                has_note=os.path.isfile(note_path(directory, stamp)))


def draw_header(ax, sess, title_fs=13.5):
    """Overskrift med nummer-badge foran, og beskrivelsen under.

    Aksen låses til 0..1 og ALT tegnes i de koordinatene. Blandet data- og
    transAxes-tegning her autoskalerer aksen og skyver tittelen ut av bildet."""
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    x0 = 0.0
    if sess.get("nr"):
        # Samme farge og form som markøren på oversiktskartet, så måling nr. N
        # er gjenkjennelig begge steder.
        ax.plot([0.010], [0.86], marker="o", ms=15, mfc=C_BADGE, mec="none",
                ls="none", clip_on=False)
        ax.text(0.010, 0.86, str(sess["nr"]), fontsize=9, weight="bold",
                color="white", ha="center", va="center")
        x0 = 0.026
    ax.text(x0, 1.0, sess["title"], fontsize=title_fs, weight="bold", va="top",
            color="#111111")
    if sess["body"]:
        ax.text(x0, 0.52, sess["body"], fontsize=8.6, va="top", color=C_MUTED,
                wrap=True)


def make_figure(sess, args):
    """Én A4-vennlig side per måling:

        [nr] overskrift + beskrivelse
        ┌──────────────────────┬───────────┐
        │  GPS-bane på OSM     │ nær-rute  │
        │                      ├───────────┤
        │                      │ fjern-rute│
        └──────────────────────┴───────────┘
        ┌───────────────────┬──────────────┐
        │ elevasjonsspekter │ Økt          │
        ├───────────────────┤ Logging      │
        │ Hs/Tz/Tc + taper  │              │
        └───────────────────┴──────────────┘
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    gps, stats = sess["gps"], sess["stats"]
    fw = args.width
    m_l, m_r = 0.45, 0.45
    cw = fw - m_l - m_r                      # innholdsbredde

    # Innfelt-kolonnen setter høyden på hele øvre rad: to kvadratiske ruter
    # over hverandre. Kartet får samme høyde, og bboxen strekkes til den formen
    # i stedet for omvendt - da blir ALLE måle-figurene like høye, som er verdt
    # mer i en oppgave enn at hver enkelt får sin optimale form.
    gap_x, gap_y = 0.14, 0.10
    ins_w = 0.265 * cw
    map_w = cw - ins_w - gap_x
    top_h = 2 * ins_w + gap_y
    bbox = fit_bbox(map_bbox(gps["lat"], gps["lon"]), top_h / map_w)

    psd_w = 0.475 * cw
    tab_w = cw - psd_w - 0.22
    psd_h, wave_h = 1.95, 1.28      # wave_h rommer 5 metoderader + fotnoter
    bot_h = psd_h + 0.42 + wave_h            # 0.42 = plass til akse-etiketter

    body = sess["body"]
    head_h = 0.52 + (0.17 * (1 + len(body) // 120) if body else 0.0)
    attr_h = 0.24
    m_b, m_t = 0.26, 0.26
    fh = m_b + bot_h + 0.30 + attr_h + top_h + 0.16 + head_h + m_t

    fig = plt.figure(figsize=(fw, fh))

    def rect(x, y, w, h):
        return [x / fw, y / fh, w / fw, h / fh]

    y_top = m_b + bot_h + 0.30 + attr_h      # underkant av kartraden
    draw_header(fig.add_axes(rect(m_l, y_top + top_h + 0.16, cw, head_h)), sess)

    ax_map = fig.add_axes(rect(m_l, y_top, map_w, top_h))
    _, (handles, labels) = draw_map(ax_map, gps, stats, args, bbox)

    # Innfellingene ligger UTENFOR kartet nå, i egen kolonne: nær-ruta øverst
    # med kartutsnittet som ramme, fjern-ruta under med nær-ruta som ramme.
    x_ins = m_l + map_w + gap_x
    if not args.no_inset and not args.no_map:
        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0
        spans = [args.inset_km] + [float(s) for s in
                                   str(args.inset_far_km).split(",") if s.strip()]
        frame = bbox
        for i, span in enumerate(spans[:2]):
            y_i = y_top + (top_h - ins_w if i == 0 else 0.0)
            axi = fig.add_axes(rect(x_ins, y_i, ins_w, ins_w))
            frame = draw_on_inset(axi, cx, cy, stats["lat_mid"], span, args,
                                  frame_bbox=frame,
                                  badge_nr=sess.get("nr") if i == 0 else None)
            if frame is None:
                break

    # Legende og attribusjon rett under kartraden - inne i kartet ville de
    # måttet konkurrere med sporet om plassen.
    y_attr = m_b + bot_h + 0.30
    fig.legend(handles, labels, loc="lower left", ncol=4, fontsize=6.6,
               frameon=False, handletextpad=0.5, columnspacing=1.2,
               bbox_to_anchor=(m_l / fw, (y_attr - 0.02) / fh))
    attr = ("Basemap © OpenStreetMap contributors" if not args.no_map
            else "No basemap (--no-map)")
    # Bare attribusjonen her: zoomnivået sto her før, men figuren har tre kart på
    # hvert sitt nivå, så ett enkelt z-tall sa ingenting entydig. Nivåene skrives
    # fortsatt til stdout under kjøring hvis de trengs.
    fig.text(1.0 - m_r / fw, y_attr / fh, attr,
             fontsize=6.0, color=C_MUTED, ha="right", va="bottom")

    draw_psd(fig.add_axes(rect(m_l + 0.34, m_b + wave_h + 0.42,
                               psd_w - 0.34, psd_h)), sess["spec"], args,
             sess["ana"])

    okt, logg, bolge = build_blocks(sess["cfg"], sess["ses"], sess["ana"], stats)
    draw_wave_table(fig.add_axes(rect(m_l, m_b, psd_w, wave_h)),
                    bolge, sess["ana_src"], sess["ana"], sess["cfg"], args)
    draw_kv_blocks(fig.add_axes(rect(fw - m_r - tab_w, m_b, tab_w, bot_h)),
                   [("Session", okt), ("Logging", logg)], w_key=0.44)
    return fig


# --- Oversiktskart -----------------------------------------------------------
def spread_badges(pts, bbox, min_frac=0.055, iters=60):
    """Skyv nummer-badgene fra hverandre så de ikke legger seg oppå hverandre.

    Målingene ligger typisk i samme fjord, og flere av dem starter nesten på
    samme punkt - uten dette blir tallene uleselige. Returnerer forskjøvne
    posisjoner; kallende kode tegner en tynn linje tilbake til det ekte punktet
    når badgen har flyttet seg merkbart."""
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    p = np.array(pts, dtype=float)
    if len(p) < 2:
        return p
    dmin = min_frac * max(w, h)
    for _ in range(iters):
        moved = False
        for i in range(len(p)):
            for j in range(i + 1, len(p)):
                d = p[j] - p[i]
                r = math.hypot(d[0], d[1])
                if r >= dmin:
                    continue
                if r < 1e-9:            # nøyaktig sammenfall: dytt vilkårlig
                    d = np.array([dmin, 0.0])
                    r = dmin
                push = (dmin - r) / 2.0 * d / r
                p[i] -= push
                p[j] += push
                moved = True
        if not moved:
            break
    return p


def overview_note(directory):
    """oversikt_notat.txt ved siden av oversiktsfiguren: linje 1 = overskrift,
    resten = beskrivelse. Samme regel som <stamp>_notat.txt per måling."""
    path = os.path.join(directory, "oversikt_notat.txt")
    if not os.path.isfile(path):
        return None, ""
    with open(path, encoding="utf-8") as f:
        lines = f.read().splitlines()
    while lines and (not lines[0].strip() or lines[0].lstrip().startswith("#")):
        lines.pop(0)
    if not lines:
        return None, ""
    return lines[0].strip(), "\n".join(lines[1:]).strip()


def make_overview(sessions, args, out_dir):
    """Ett kart med alle målingene, nummerert, og en tabell som oppsummerer dem."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lat = np.concatenate([s["gps"]["lat"] for s in sessions])
    lon = np.concatenate([s["gps"]["lon"] for s in sessions])

    fw = args.width
    m_l, m_r, m_b, m_t = 0.45, 0.45, 0.26, 0.26
    cw = fw - m_l - m_r
    # Samme todeling som enkeltfigurene: 2/3 til rutene, 1/3 til de innfelte
    # utsnittene stablet over hverandre. Kartet får høyden fra innfelt-kolonnen
    # (to kvadrater), og bboxen strekkes til den formen.
    gap_x, gap_y = 0.14, 0.10
    ins_w = (cw - gap_x) / 3.0
    map_w = cw - ins_w - gap_x
    map_h = 2 * ins_w + gap_y
    bbox = fit_bbox(map_bbox(lat, lon, pad_frac=0.18, min_span_m=1000.0),
                    map_h / map_w)

    title, body = overview_note(out_dir)
    if not title:
        title = f"{os.path.basename(out_dir.rstrip(os.sep))} – {len(sessions)} measurements"
    head_h = 0.52 + (0.17 * (1 + len(body) // 120) if body else 0.0)
    tab_h = 0.56 + 0.175 * len(sessions)
    gap, attr_h = 0.30, 0.24
    fh = m_b + tab_h + gap + attr_h + map_h + 0.16 + head_h + m_t

    fig = plt.figure(figsize=(fw, fh))

    def rect(x, y, w, h):
        return [x / fw, y / fh, w / fw, h / fh]

    y_map = m_b + tab_h + gap + attr_h
    draw_header(fig.add_axes(rect(m_l, y_map + map_h + 0.16, cw, head_h)),
                dict(nr=None, title=title, body=body))

    ax = fig.add_axes(rect(m_l, y_map, map_w, map_h))
    zoom = args.zoom if args.zoom else pick_zoom(bbox)
    if not args.no_map:
        img, extent = build_basemap(bbox, zoom, args.tile_url, args.tile_cache)
        if img is not None:
            ax.imshow(img, extent=extent, origin="upper",
                      interpolation="bilinear", zorder=0)
        else:
            args.no_map = True
    if args.no_map:
        ax.set_facecolor("#eef2f5")
    ax.set_xlim(bbox[0], bbox[2])
    ax.set_ylim(bbox[1], bbox[3])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_edgecolor("#8a8a8a")
        s.set_linewidth(0.8)

    import matplotlib.patheffects as pe
    halo = [pe.Stroke(linewidth=2.4, foreground="white"), pe.Normal()]
    anchors = []
    for s in sessions:
        x, y = lonlat_to_merc(s["gps"]["lon"], s["gps"]["lat"])
        ax.plot(x[::10], y[::10], "-", color=C_TRACK, lw=1.4, zorder=3,
                solid_capstyle="round", path_effects=halo)
        ax.plot([x[0]], [y[0]], "o", ms=4.5, mfc=C_START, mec="white", mew=0.9,
                ls="none", zorder=4)
        ax.plot([x[-1]], [y[-1]], "s", ms=4.5, mfc=C_STOP, mec="white", mew=0.9,
                ls="none", zorder=4)
        anchors.append((float(np.mean(x)), float(np.mean(y))))

    placed = spread_badges(anchors, bbox)
    for s, (ax0, ay0), (bx, by) in zip(sessions, anchors, placed):
        if math.hypot(bx - ax0, by - ay0) > 0.004 * (bbox[2] - bbox[0]):
            ax.plot([ax0, bx], [ay0, by], "-", color=C_BADGE, lw=0.8, alpha=0.7,
                    zorder=9)
        draw_badge(ax, bx, by, s["nr"], size=15, fs=8.5)

    lat_mid = float(np.mean(lat))
    # Nordpil og målestokk er alt som ligger inne i kartet nå - innfellingene
    # er flyttet ut i egen kolonne. De to får hver sin halvdel, minst dekket
    # av rutene.
    load = corner_load(np.array([p[0] for p in placed]),
                       np.array([p[1] for p in placed]), bbox)
    north_c = min(["tl", "tr"], key=lambda k: (load[k], k))
    scale_c = min(["bl", "br"], key=lambda k: (load[k], k))
    draw_scalebar(ax, bbox, lat_mid, scale_c)
    draw_north(ax, bbox, north_c)

    # Kaskade av innfelte ruter i høyre kolonne, f.eks. 10 km og 100 km: hver
    # viser hvor utsnittet under ligger, så man kan gå fra måleområdet og ut
    # til hvor i landet det er.
    spans = [float(s) for s in str(args.overview_inset_km).split(",") if s.strip()]
    x_ins = m_l + map_w + gap_x
    if not args.no_inset and not args.no_map:
        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0
        if len(spans) > 2:
            print("  (kolonnen rommer 2 innfellinger - bruker de to første "
                  "av --overview-inset-km)")
        frame = bbox
        for i, span in enumerate(spans[:2]):
            y_i = y_map + (map_h - ins_w if i == 0 else 0.0)
            axi = fig.add_axes(rect(x_ins, y_i, ins_w, ins_w))
            frame = draw_on_inset(axi, cx, cy, lat_mid, span, args,
                                  frame_bbox=frame)
            if frame is None:
                break

    y_attr = (m_b + tab_h + gap + 0.05) / fh
    attr = ("Basemap © OpenStreetMap contributors" if not args.no_map
            else "No basemap (--no-map)")
    fig.text(m_l / fw, y_attr,
             "green = start · red = end · number = measurement (see table)",
             fontsize=6.2, color=C_MUTED, ha="left", va="bottom")
    # Uten zoomnivå: oversikten har tre kart på hvert sitt nivå (jf. make_figure).
    fig.text(1.0 - m_r / fw, y_attr, attr,
             fontsize=6.0, color=C_MUTED, ha="right", va="bottom")

    # Tabellen går i FULL bredde under begge kolonnene - den har ni kolonner og
    # ville blitt uleselig klemt inn på kartets 2/3.
    draw_overview_table(fig.add_axes(rect(m_l, m_b, cw, tab_h)),
                        sessions, args.overview_method)
    return fig


def draw_overview_table(ax, sessions, method="madgwick"):
    """Én rad per måling: nr, overskrift, dato, klokkeslett, varighet, drift, Hs/Tz/Tc."""
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    dy = 1.0 / (len(sessions) + 2.2)
    # Kolonnene er satt etter mono-bredden på den bredeste verdien i hver, ved
    # figurbredde 7.5". Sluttidspunktet er droppet - start + varighet sier det
    # samme, og plassen går til målingsnavnet i stedet.
    cols = [0.000, 0.035, 0.400, 0.505, 0.615, 0.700, 0.775, 0.850, 0.925]
    heads = ["#", "Measurement", "Date", "Start (UTC)", "Duration", "Drift",
             "Hs [m]", "Tz [s]", "Tc [s]"]
    yy = 1.0
    for cx_, h in zip(cols, heads):
        ax.text(cx_, yy, h, fontsize=7.2, weight="bold", color="#222222", va="top")
    val_kw = dict(fontsize=7.2, color="#111111", va="top",
                  family="DejaVu Sans Mono")
    for i, s in enumerate(sessions):
        yy = 1.0 - (i + 1.30) * dy
        t0, _ = fmt_time(s["ses"].get("start_utc_iso", ""))
        ana, m = s["ana"], method
        vals = [str(s["nr"]),
                s["title"],
                t0.split()[0] if t0 != DASH else DASH,
                t0.split()[-1] if t0 != DASH else DASH,
                fmt_dur(s["ses"].get("duration_ms")),
                f"{s['stats']['net']:.0f} m",
                fmt_val(ana, f"Hs_{m}", dec=3),
                fmt_val(ana, f"Tz_{m}", dec=2),
                fmt_val(ana, f"Tc_{m}", dec=2)]
        for j, (cx_, v) in enumerate(zip(cols, vals)):
            if j == 1:                       # overskriften er proporsjonal tekst
                ax.text(cx_, yy, v, fontsize=7.2, color=C_MUTED, va="top")
            else:
                ax.text(cx_, yy, v, **val_kw)
    ax.text(0, 1.0 - (len(sessions) + 1.9) * dy,
            f"Wave parameters: {method.capitalize()}. "
            "See individual figures for all methods and source file.",
            fontsize=6.4, color=C_MUTED, va="top", style="italic")


# --- CLI ---------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Kartfigur (OpenStreetMap) for en ORB-loggeøkt.")
    ap.add_argument("path", nargs="*", default=[DEFAULT_SESSION],
                    help="én eller flere øktkataloger / *_gps.csv. Rekkefølgen "
                         "på kommandolinja er målingsnummeret (overstyres med "
                         "--nr). Default: Skjærhalden-økta")
    ap.add_argument("--nr", help="målingsnummer per økt, komma-separert "
                                 "(f.eks. --nr 3,7,8). Default 1,2,3,...")
    ap.add_argument("-o", "--out",
                    help="utfil (default <øktkatalog>/<stamp>_kart.png). "
                         "Kun gyldig med én økt.")
    ap.add_argument("--format", choices=("png", "pdf"), default=None,
                    help="utformat; utledes av --out hvis den er oppgitt")
    ap.add_argument("--dpi", type=int, default=200)
    ap.add_argument("--width", type=float, default=7.5,
                    help="figurbredde i tommer (default 7.5 = A4 med marger)")
    ap.add_argument("--taper-f1", type=float, default=0.1,
                    help="lavfrekvens-taper: T=0 under denne [Hz] (default 0.1). "
                         "Sendes til postprocess.py og skrives i figuren.")
    ap.add_argument("--taper-f2", type=float, default=0.2,
                    help="lavfrekvens-taper: T=1 over denne [Hz] (default 0.2)")
    ap.add_argument("--raw", default=None, metavar="STI",
                    help="sendes til postprocess.py: hvilken <stamp>_raw.bin "
                         "alternativfiltrene skal kjøres på. 'off' holder "
                         "analysen til det imu.csv selv inneholder")
    ap.add_argument("--vacc-source", choices=("auto", "raw", "csv"), default="auto",
                    help="sendes til postprocess.py: raw = vertikal-accelen fra "
                         "vacc_fir (AHRS på råstrømmen), csv = replay på "
                         "radraten, auto = raw når kolonnen finnes. Står i "
                         "bølgetabellen, for tallene er ikke sammenlignbare "
                         "på tvers av valget")
    ap.add_argument("--psd-fmin", type=float, default=None,
                    help="nedre frekvens i spekterplottet [Hz] "
                         "(default: taperens f1 - under den er alt nullet bort)")
    ap.add_argument("--psd-fmax", type=float, default=2.0,
                    help="øvre frekvens i spekterplottet [Hz] (default 2.0)")
    ap.add_argument("--skip-postprocess", action="store_true",
                    help="ikke kjør postprocess.py på nytt - bruk _ana_python.csv "
                         "slik den ligger (raskt, men tallene kan være gamle)")
    ap.add_argument("--tick-sec", type=float, default=30.0,
                    help="punkter langs ruta hvert N. sekund (default 30)")
    ap.add_argument("--label-sec", type=float, default=300.0,
                    help="merkede punkter med klokke hvert N. sekund (default 300)")
    ap.add_argument("--zoom", type=int, default=None, help="overstyr flis-zoom")
    ap.add_argument("--no-map", action="store_true",
                    help="ikke last ned fliser - tegn ruta på rutenett")
    ap.add_argument("--no-inset", action="store_true",
                    help="dropp oversiktsruta inne i hvert enkeltkart")
    ap.add_argument("--inset-km", type=float, default=1.5,
                    help="bredde i km på NÆR-ruta i enkeltkartene. Samme verdi "
                         "for alle målinger, så de kan sammenlignes (default 2)")
    ap.add_argument("--inset-far-km", type=float, default=50.0,
                    help="bredde i km på FJERN-ruta under nær-ruta i "
                         "enkeltkartene (default 50)")
    ap.add_argument("--overview-inset-km", default="10,100",
                    help="komma-separerte bredder i km på de innfelte rutene i "
                         "oversiktskartet. Hver rute viser hvor utsnittet under "
                         "ligger (default 10,100)")
    ap.add_argument("--overview-out",
                    help="utfil for oversiktskartet (default "
                         "<felleskatalog>/oversikt_kart.png)")
    ap.add_argument("--overview-method", default="madgwick",
                    help="metode for Hs/Tz/Tc i oversiktstabellen (default madgwick)")
    ap.add_argument("--no-overview", action="store_true",
                    help="ikke lag oversiktskart selv med flere økter")
    ap.add_argument("--only-overview", action="store_true",
                    help="lag KUN oversiktskartet, ingen enkeltfigurer")
    ap.add_argument("--tile-url", default=TILE_URL)
    ap.add_argument("--tile-cache", default=TILE_CACHE)
    ap.add_argument("--init-notat", action="store_true",
                    help="skriv <stamp>_notat.txt-stubb og avslutt")
    args = ap.parse_args()
    prefer_ipv4()

    paths = args.path or [DEFAULT_SESSION]
    if args.out and len(paths) > 1:
        sys.exit("--out kan bare brukes med én økt; med flere økter skrives "
                 "<stamp>_kart.png i hver øktkatalog (bruk --overview-out for "
                 "oversiktskartet).")

    numbers = list(range(1, len(paths) + 1))
    if args.nr:
        want = [s.strip() for s in args.nr.split(",") if s.strip()]
        if len(want) != len(paths):
            sys.exit(f"--nr har {len(want)} verdier, men {len(paths)} økter er "
                     f"oppgitt.")
        numbers = want

    if args.init_notat:
        for p in paths:
            _, stamp, directory = resolve_session(p)
            print(f"Økt: {stamp}  ({directory})")
            write_note_stub(directory, stamp)
        return

    ext = args.format or (os.path.splitext(args.out)[1].lstrip(".").lower()
                          if args.out else "png")
    # Nummeret vises bare når det faktisk skiller målinger fra hverandre - en
    # ensom "1"-badge på en enkeltstående figur er bare støy.
    show_nr = len(paths) > 1 or bool(args.nr)

    sessions = []
    for p, nr in zip(paths, numbers):
        sess = load_session(p, args, nr if show_nr else None)
        sess["nr_label"] = nr
        sessions.append(sess)
        print(f"Måling {nr}: {sess['stamp']}  ({sess['directory']})")
        if not sess["cfg"]:
            print("  ADVARSEL: fant ingen _cfg.csv - logge-parametrene blir tomme.")
        if not sess["ana_src"]:
            print("  ADVARSEL: fant ingen _ana_python.csv - bølgeparametrene "
                  "blir tomme.")
        else:
            print(f"  bølgeparametre fra {sess['ana_src']}")
        if sess["gps"]["n_dropped"]:
            print(f"  {sess['gps']['n_dropped']} GPS-rader uten 3D-fix forkastet "
                  f"(av {sess['gps']['n_raw']}).")
        # Avbrutt fangst: si det HER, sammen med radtellingen, ikke som en
        # fotnote. Figuren under viser en kortere måling enn økta var satt til,
        # og det skal man vite før man sammenligner den med de andre.
        kutt = sess["gps"]["kutt"]
        if kutt.grunn:
            print(f"  gps.csv slutter etter {kutt.rader} rader: {kutt.grunn}. "
                  f"{kutt.total - kutt.lest} B av {kutt.total} er hale fra en "
                  f"avbrutt fangst (fila ligger på preallokert lengde).")
        st = sess["stats"]
        print(f"  {len(sess['gps']['t_s'])} punkter, {st['dist']:.0f} m "
              f"tilbakelagt, netto {st['net']:.0f} m mot {st['bearing']:.0f}°")
        if not sess["has_note"]:
            print(f"  (ingen {sess['stamp']}_notat.txt - autogenerert overskrift. "
                  f"Lag en med --init-notat.)")

    # Oversiktskartet først: da ser man helheten før enkeltfigurene, og
    # rekkefølgen i utskriften følger arbeidsflyten.
    if len(sessions) > 1 and not args.no_overview:
        out_dir = args.overview_out and os.path.dirname(
            os.path.abspath(args.overview_out))
        if not out_dir:
            out_dir = os.path.commonpath([s["directory"] for s in sessions])
        for s in sessions:                     # oversikten nummererer alltid
            s["nr"] = s["nr_label"]
        print(f"Oversikt: {len(sessions)} målinger")
        fig = make_overview(sessions, args, out_dir)
        out = args.overview_out or os.path.join(out_dir, f"oversikt_kart.{ext}")
        fig.savefig(out, dpi=args.dpi, facecolor="white")
        print(f"  skrev {out}")
        for s in sessions:                     # tilbake til enkeltfigur-regelen
            s["nr"] = s["nr_label"] if show_nr else None

    if args.only_overview:
        return

    import matplotlib.pyplot as plt
    for sess in sessions:
        fig = make_figure(sess, args)
        out = args.out or os.path.join(sess["directory"],
                                       f"{sess['stamp']}_kart.{ext}")
        fig.savefig(out, dpi=args.dpi, facecolor="white")
        plt.close(fig)          # ellers hoper figurene seg opp ved mange økter
        print(f"  skrev {out}")


if __name__ == "__main__":
    main()
