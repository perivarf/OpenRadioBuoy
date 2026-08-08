"""Leser <stamp>_raw.bin - den UDESIMERTE IMU-loggen fra wave_raw_log.

Der imu.csv er én rad per kRowPeriodMs etter FIR trinn 1, er denne fila hvert
eneste FIFO-ord slik sensoren produserte det. Poenget er offline re-analyse på de
samplene enheten faktisk så: firmwaren kjører AHRS-en på råstrømmen
(kAhrsInputOdrHz), mens Python-speilet hittil har måttet kjøre den på radrate.

FORMATET (little-endian, definert ved wave_raw_log i wave_config.h):

    header  32 B: magic "ORWB", versjon, ordlengde, ODR-er,
                  sensitiviteter (LSB -> mg / mdps), capture-start UTC, reading_ID
    ord      7 B: tag_sensor + 6 B payload, uendret fra FIFO-en
    sync    17 B: 0xFF + u32 t_us + u32 accel_n + u32 millis + u16 n_words + u16 flags

FLAGGENE er to helt ulike feil, og forskjellen avgjør hva fila er verdt:

    bit 0  FIFO_OVF    sensoren overskrev ord. Fila er hel, men et tidsrom mangler
                       fra den, og tidsaksen er komprimert der. Brukbar med varsomhet.
    bit 1  WRITE_FAIL  DENNE FILA mistet bytes. Formatet er en byte-strøm der en post
                       kan krysse en blokkgrense, så et tapt parti forskyver ALT
                       etterpå: dekoderen leser en payload-byte som en tag og fortsetter
                       i god tro. Alt etter første slike flagg er uten verdi.

Taggen forteller lengden: 0xFF er sync, alt annet er et ord. De kan ikke kollidere -
tag_sensor er de øverste 5 bitene av FIFO-taggen, altså maks 0x1F.

TIDSAKSEN. Det finnes ikke tidsstempel per ord, med vilje: ved 480 Hz er
sampleperioden 2.08 ms, så et helt millisekund ville vært grovere enn dataene. Rekke-
følgen er tidsaksen, og sync-postene pinner den til klokka med t_us (brøkdels-ms) og
den kumulative accel-telleren. read() interpolerer accel-tidene mellom sync-postene.

    import rawlog
    cap = rawlog.read("20260731_110314_raw.bin")
    cap.acc_t, cap.acc      # (N,) s siden capture-start, (N,3) mg
    cap.gyr_t, cap.gyr      # (M,) s, (M,3) mdps
    cap.quat_t, cap.quat    # (K,) s, (K,4) [w,x,y,z]
"""

import struct
import sys

import numpy as np

MAGIC = 0x4257524F            # "ORWB"
SYNC_TAG = 0xFF
HEADER_BYTES = 32
FLAG_FIFO_OVF = 0x0001        # kRawFlagFifoOvf
FLAG_WRITE_FAIL = 0x0002      # kRawFlagWriteFail
TAG_GYR = 1
TAG_ACC = 2
TAG_SFLP = 0x13               # kSflpGameRotationTag


class RawCapture:
    """Dekodet råfangst. Feltene er numpy-arrays; tidene er sekunder fra start."""

    def __init__(self, hdr):
        self.__dict__.update(hdr)
        self.acc_t = self.gyr_t = self.quat_t = None
        self.acc = self.gyr = self.quat = None
        self.sync = []            # dicts, én per drenering
        self.n_overflow = 0
        self.n_write_fail = 0
        # Accel-indeksen der det FØRSTE tapte partiet ble meldt. Alt fra og med den er
        # potensielt feiltolket, så den er kuttepunktet, ikke bare en teller.
        self.first_write_fail_n = None
        self.unknown_tags = {}

    def trustworthy_upto(self):
        """Antall accel-samples som kan brukes. len(acc) når ingenting gikk tapt."""
        if self.first_write_fail_n is None:
            return len(self.acc)
        return min(int(self.first_write_fail_n), len(self.acc))


def read_header(buf):
    if len(buf) < HEADER_BYTES:
        sys.exit("raw: fila er kortere enn headeren")
    magic, ver = struct.unpack_from("<IB", buf, 0)
    if magic != MAGIC:
        sys.exit(f"raw: feil magic 0x{magic:08X} - ikke en _raw.bin")

    # v1 hadde build_seq (u16) mellom word_bytes og ODR-ene. Feltet er borte i v2, så
    # alt etter det ligger 2 bytes tidligere. Bare offsetene skiller versjonene - selve
    # ord- og sync-postene er uendret, så resten av parseren er felles.
    if ver == 1:
        wordlen, = struct.unpack_from("<B", buf, 5)
        imu_odr, sflp_odr = struct.unpack_from("<HH", buf, 8)
        acc_sens, gyr_sens = struct.unpack_from("<ff", buf, 12)
        t0, reading_id, sync_bytes = struct.unpack_from("<IHH", buf, 20)
    elif ver == 2:
        wordlen, imu_odr, sflp_odr = struct.unpack_from("<BHH", buf, 5)
        acc_sens, gyr_sens = struct.unpack_from("<ff", buf, 10)
        t0, reading_id, sync_bytes = struct.unpack_from("<IHH", buf, 18)
    else:
        sys.exit(f"raw: ukjent format v{ver} - denne rawlog.py kjenner v1 og v2")

    return dict(version=ver, word_bytes=wordlen,
                imu_odr_hz=imu_odr, sflp_odr_hz=sflp_odr,
                acc_sens_mg=acc_sens, gyr_sens_mdps=gyr_sens,
                t0_utc=t0, reading_id=reading_id, sync_bytes=sync_bytes)


def _half_to_float(h):
    """IEEE half -> float, samme som halfToFloat() i imu_sampler.cpp."""
    return np.frombuffer(np.array([h], dtype="<u2").tobytes(), dtype="<f2")[0]


def read(path):
    with open(path, "rb") as f:
        buf = f.read()
    cap = RawCapture(read_header(buf))
    wb, sb = cap.word_bytes, cap.sync_bytes

    acc_raw, gyr_raw, quat_raw = [], [], []
    acc_idx_at, acc_t_at = [], []      # sync-punkter for accel-tidsaksen
    acc_n = gyr_n = 0
    gyr_after_acc, quat_after_acc = [], []   # accel-teller ved hvert gyro/quat-ord

    o = HEADER_BYTES
    n = len(buf)
    while o < n:
        tag = buf[o]
        if tag == SYNC_TAG:
            if o + sb > n:
                break                  # avkortet hale
            t_us, a_n, ms, nw, flags = struct.unpack_from("<IIIHH", buf, o + 1)
            cap.sync.append(dict(t_us=t_us, accel_n=a_n, millis=ms,
                                 n_words=nw, flags=flags,
                                 fifo_ovf=bool(flags & FLAG_FIFO_OVF),
                                 write_fail=bool(flags & FLAG_WRITE_FAIL)))
            if flags & FLAG_FIFO_OVF:
                cap.n_overflow += 1
            if flags & FLAG_WRITE_FAIL:
                cap.n_write_fail += 1
                if cap.first_write_fail_n is None:
                    cap.first_write_fail_n = a_n
            acc_idx_at.append(a_n)
            acc_t_at.append(t_us * 1e-6)
            o += sb
            continue
        if o + wb > n:
            break
        p = buf[o + 1:o + wb]
        if tag == TAG_ACC:
            acc_raw.append(struct.unpack("<hhh", p)); acc_n += 1
        elif tag == TAG_GYR:
            gyr_raw.append(struct.unpack("<hhh", p)); gyr_n += 1
            gyr_after_acc.append(acc_n)
        elif tag == TAG_SFLP:
            x, y, z = struct.unpack("<HHH", p)
            quat_raw.append((x, y, z))
            quat_after_acc.append(acc_n)
        else:
            cap.unknown_tags[tag] = cap.unknown_tags.get(tag, 0) + 1
        o += wb

    # Accel-tidsaksen: lineær mellom sync-punktene, som er (accel_n, t_us)-par.
    #
    # np.interp alene duger ikke: den KLEMMER utenfor ytterpunktene, og siste sync-post
    # ligger ved starten av siste drenering - så hele den siste bunken ville fått samme
    # tid. Det ser ut som en for høy målt rate (489 Hz av 480), altså nøyaktig den
    # slags stille tidsakse-feil denne fila finnes for å unngå. Utenfor ytterpunktene
    # ekstrapoleres det derfor på nominell ODR.
    idx = np.arange(len(acc_raw), dtype=np.float64)
    nominal = 1.0 / float(cap.imu_odr_hz)
    if len(acc_idx_at) >= 2:
        xp = np.asarray(acc_idx_at, dtype=np.float64)
        fp = np.asarray(acc_t_at, dtype=np.float64)
        keep = np.diff(xp, prepend=xp[0] - 1) > 0      # sync-poster må være økende
        xp, fp = xp[keep], fp[keep]
        acc_t = np.interp(idx, xp, fp)
        lo, hi = idx < xp[0], idx > xp[-1]
        acc_t[lo] = fp[0] + (idx[lo] - xp[0]) * nominal
        acc_t[hi] = fp[-1] + (idx[hi] - xp[-1]) * nominal
    else:
        acc_t = idx * nominal

    cap.acc = np.asarray(acc_raw, dtype=np.float64) * cap.acc_sens_mg
    cap.gyr = np.asarray(gyr_raw, dtype=np.float64) * cap.gyr_sens_mdps
    cap.acc_t = acc_t
    # Gyro og SFLP har ingen egen teller; de plasseres på accel-tiden som gjaldt da
    # ordet kom. Det er samme paring firmwaren gjør (imu_sampler.cpp: gyroen latches
    # og accel-grenen bruker siste verdi), altså maks ett sample skjevhet.
    cap.gyr_t = (np.interp(np.asarray(gyr_after_acc, dtype=np.float64), idx, acc_t)
                 if len(gyr_after_acc) else np.empty(0))

    if quat_raw:
        q = np.asarray(quat_raw, dtype="<u2")
        xyz = q.view("<f2").astype(np.float64).reshape(-1, 3)
        # w rekonstrueres som i payloadToQuat: sumsq <= 1 -> w = sqrt(1 - sumsq).
        sumsq = np.clip((xyz ** 2).sum(axis=1), 0.0, 1.0)
        w = np.sqrt(1.0 - sumsq)
        cap.quat = np.column_stack([w, xyz])
        cap.quat_t = np.interp(np.asarray(quat_after_acc, dtype=np.float64), idx, acc_t)
    else:
        cap.quat = np.empty((0, 4))
        cap.quat_t = np.empty(0)
    return cap


def summary(cap):
    def rate(t):
        return len(t) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else float("nan")
    lines = [
        f"format v{cap.version}, reading {cap.reading_id}",
        f"cfg: imu_odr {cap.imu_odr_hz} Hz, sflp_odr {cap.sflp_odr_hz} Hz, "
        f"acc {cap.acc_sens_mg:g} mg/LSB, gyr {cap.gyr_sens_mdps:g} mdps/LSB",
        f"accel {len(cap.acc):7d} prøver, målt {rate(cap.acc_t):7.2f} Hz",
        f"gyro  {len(cap.gyr):7d} prøver, målt {rate(cap.gyr_t):7.2f} Hz",
        f"sflp  {len(cap.quat):7d} prøver, målt {rate(cap.quat_t):7.2f} Hz",
        f"sync  {len(cap.sync):7d} dreneringer, {cap.n_overflow} med FIFO-overflow",
    ]
    if len(cap.sync) > 1:
        d = np.diff([s["millis"] for s in cap.sync])
        # Hodrommet MÅ regnes fra fangstens egen ordrate, ikke stå som en konstant:
        # 512 ord er 427 ms ved 480 Hz men bare 237 ms ved 960, og en fast tekst ville
        # sagt at en 300 ms stall var ufarlig nettopp i den konfigurasjonen der den
        # ikke er det. Ordraten er accel + gyro + SFLP, hver på sin ODR.
        words = 2.0 * cap.imu_odr_hz + cap.sflp_odr_hz
        hodrom = 512.0 / words * 1000.0 if words > 0 else float("nan")
        over = int((d > hodrom).sum())
        lines.append(f"drenering: median {np.median(d):.0f} ms, maks {d.max():.0f} ms "
                     f"(FIFO-hodrom {hodrom:.0f} ms ved {words:.0f} ord/s)")
        if over:
            lines.append(f"           {over} dreneringer ({100*over/len(d):.2f} %) "
                         f"over hodrommet - der kan samples ha gaatt tapt")
    if cap.unknown_tags:
        lines.append(f"ukjente tagger: {cap.unknown_tags}")
    # Til slutt, og ikke som en fotnote: dette er den ene feilen som gjør at tallene
    # over kan være oppdiktet. Ukjente tagger like over er som regel bekreftelsen.
    if cap.n_write_fail:
        n = cap.trustworthy_upto()
        lines += [
            "",
            f"*** {cap.n_write_fail} tapte blokker under skriving (kRawFlagWriteFail).",
            f"*** Fila er desynkronisert fra accel-sample {cap.first_write_fail_n}"
            f" ({n / max(1, cap.imu_odr_hz):.1f} s inn); alt etter det er feiltolket.",
            "*** Bruk cap.trustworthy_upto() og forkast resten - ikke stol paa"
            " ratene over, de er regnet på hele fila.",
        ]
    return "\n".join(lines)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("bruk: python3 rawlog.py <stamp>_raw.bin")
    print(summary(read(sys.argv[1])))
