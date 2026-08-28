#!/usr/bin/env python3
"""<stamp>_raw.bin -> <stamp>_imu_from_raw.csv: decoding, and nothing more.

The file is every FIFO word exactly as the sensor produced it, where imu.csv is the
FIR-decimated row. This runs no FIR, no AHRS and no analysis - it writes out what the
binary actually holds, one row per accel sample, so the signal can be read without the
firmware that wrote it.

THE FORMAT (little-endian, defined by the kRaw* constants in log_config.h and by
RawLogWriter):

    header  32 B  magic "ORWB", version, word length, the ODRs, the sensitivities
                  (LSB -> mg / mdps), capture start as a UTC epoch, reading_ID and
                  the sync length. The rest is reserved and zeroed - read up to
                  kRawHeaderBytes, not to where the fields happen to end.
    word     7 B  tag_sensor + 6 B payload, unchanged from the FIFO
    sync    17 B  0xFF + u32 t_us + u32 accel_n + u32 millis + u16 n_words
                  + u16 flags, one per drain, written BEFORE the words it covers

The tag gives the length: 0xFF is a sync record, anything else is a word. They cannot
collide - tag_sensor is the top five bits of the FIFO tag, so at most 0x1F.

THE TIME AXIS. There is deliberately no per-word timestamp: at 480 Hz the sample period
is 2.08 ms, and a whole millisecond would be coarser than the data. Word ORDER is the
time axis, and the sync records pin it to the clock with t_us and the cumulative accel
counter. The times here are interpolated between those pairs.

    python3 raw_to_csv.py <capture>_raw.bin
    python3 raw_to_csv.py <session dir> -o /somewhere/else.csv
"""

import argparse
import os
import struct
import sys
from glob import glob

import numpy as np

MAGIC = 0x4257524F              # "ORWB", kRawMagic
SYNC_TAG = 0xFF                 # kRawSyncTag
HEADER_BYTES = 32               # kRawHeaderBytes
FLAG_FIFO_OVF = 0x0001          # kRawFlagFifoOvf
FLAG_WRITE_FAIL = 0x0002        # kRawFlagWriteFail
TAG_GYR = 1                     # imu_device.cpp: tag 1
TAG_ACC = 2                     # imu_device.cpp: tag 2
TAG_SFLP = 0x13                 # kSflpRotationTag

# Column order of the output. The names are imu.csv's wherever the quantity is the
# same, so a reader can use one set of column names across both files.
COLUMNS = ("n", "t_s", "ax_mg", "ay_mg", "az_mg", "gx_mdps", "gy_mdps", "gz_mdps",
           "qw_sflp", "qx_sflp", "qy_sflp", "qz_sflp", "fifo_ovf")

FIFO_DEPTH_WORDS = 256          # kFifoDepthWords, for the headroom figure


def read_header(buf):
    if len(buf) < HEADER_BYTES:
        sys.exit("raw: the file is shorter than the header")
    magic, ver = struct.unpack_from("<IB", buf, 0)
    if magic != MAGIC:
        sys.exit(f"raw: bad magic 0x{magic:08X} - not a _raw.bin")
    if ver != 2:
        sys.exit(f"raw: format v{ver}. This decoder follows the firmware in the repo, "
                 f"and that writes v2.")
    wordlen, imu_odr, sflp_odr = struct.unpack_from("<BHH", buf, 5)
    acc_sens, gyr_sens = struct.unpack_from("<ff", buf, 10)
    t0, reading_id, sync_bytes = struct.unpack_from("<IHH", buf, 18)
    return dict(version=ver, word_bytes=wordlen, imu_odr_hz=imu_odr,
                sflp_odr_hz=sflp_odr, acc_sens_mg=acc_sens, gyr_sens_mdps=gyr_sens,
                t0_utc=t0, reading_id=reading_id, sync_bytes=sync_bytes)


def parse(buf, hdr):
    """One pass over the byte stream. Gyro and SFLP are HELD until the next word of
    their kind, which is exactly what the firmware does: latestGx_/latestQx_ are
    updated when the word arrives, and the accel branch uses whatever stands there.
    Interpolating would give a prettier, but different, signal from the one the device
    computed on."""
    wb, sb = hdr["word_bytes"], hdr["sync_bytes"]
    acc, gyr, quat = [], [], []          # raw LSB, scaled at the end
    acc_gyr_i, acc_quat_i = [], []       # which gyro/quat word was current
    sync = []
    sync_n, sync_t = [], []              # (accel_n, t_us) - the time axis anchors
    ovf_at = []                          # accel index where a drain reported an overflow
    unknown = {}
    tail_at = None
    n_write_fail, first_write_fail_n = 0, None

    # THE TAIL. An interrupted capture never reaches truncate(), so the file is left at
    # its full preallocated length, and what lies past the last written block is
    # whatever was in those clusters before - usually an earlier capture's raw.bin.
    # That tail has valid tags and decodes into perfectly clean samples. Two things give
    # it away, and the capture's own stream never does either: a sync record where the
    # clock or the accel counter goes BACKWARDS, and a long run of unknown tags.
    MAX_UNKNOWN_IN_A_ROW = 64
    unknown_in_a_row = 0
    last_ms = last_an = None

    o, n = HEADER_BYTES, len(buf)
    while o < n:
        tag = buf[o]
        if tag == SYNC_TAG:
            if o + sb > n:
                break                                   # truncated tail
            t_us, a_n, ms, nw, flags = struct.unpack_from("<IIIHH", buf, o + 1)
            if last_ms is not None and (ms < last_ms or a_n < last_an):
                tail_at = (o, f"sync went backwards: {ms} ms / accel {a_n} after "
                              f"{last_ms} ms / accel {last_an}")
                break
            last_ms, last_an = ms, a_n
            unknown_in_a_row = 0
            sync.append(dict(t_us=t_us, accel_n=a_n, millis=ms, n_words=nw, flags=flags))
            sync_n.append(a_n)
            sync_t.append(t_us * 1e-6)
            if flags & FLAG_FIFO_OVF:
                ovf_at.append(a_n)
            if flags & FLAG_WRITE_FAIL:
                n_write_fail += 1
                if first_write_fail_n is None:
                    first_write_fail_n = a_n
            o += sb
            continue

        if o + wb > n:
            break
        p = buf[o + 1:o + wb]
        if tag == TAG_ACC:
            acc.append(struct.unpack("<hhh", p))
            acc_gyr_i.append(len(gyr) - 1)              # -1: no gyro word yet
            acc_quat_i.append(len(quat) - 1)
            unknown_in_a_row = 0
        elif tag == TAG_GYR:
            gyr.append(struct.unpack("<hhh", p))
            unknown_in_a_row = 0
        elif tag == TAG_SFLP:
            quat.append(struct.unpack("<HHH", p))       # three half-precision floats
            unknown_in_a_row = 0
        else:
            unknown[tag] = unknown.get(tag, 0) + 1
            unknown_in_a_row += 1
            if unknown_in_a_row >= MAX_UNKNOWN_IN_A_ROW:
                tail_at = (o - (unknown_in_a_row - 1) * wb,
                           f"{unknown_in_a_row} unknown tags in a row")
                break
        o += wb

    return dict(acc=acc, gyr=gyr, quat=quat, acc_gyr_i=acc_gyr_i,
                acc_quat_i=acc_quat_i, sync=sync, sync_n=sync_n, sync_t=sync_t,
                ovf_at=ovf_at, unknown=unknown, tail_at=tail_at,
                n_write_fail=n_write_fail, first_write_fail_n=first_write_fail_n,
                read_bytes=o, file_bytes=n)


def accel_times(n_acc, sync_n, sync_t, odr_hz):
    """Accel sample times: linear between the sync pairs, nominal ODR outside them.

    np.interp on its own will not do - it CLAMPS beyond the end points, and the last
    sync record sits at the START of the last drain, so that whole batch would get one
    time and read as a rate higher than the real one."""
    idx = np.arange(n_acc, dtype=np.float64)
    nominal = 1.0 / float(odr_hz)
    if len(sync_n) < 2:
        return idx * nominal
    xp = np.asarray(sync_n, dtype=np.float64)
    fp = np.asarray(sync_t, dtype=np.float64)
    keep = np.diff(xp, prepend=xp[0] - 1) > 0           # must be strictly increasing
    xp, fp = xp[keep], fp[keep]
    t = np.interp(idx, xp, fp)
    lo, hi = idx < xp[0], idx > xp[-1]
    t[lo] = fp[0] + (idx[lo] - xp[0]) * nominal
    t[hi] = fp[-1] + (idx[hi] - xp[-1]) * nominal
    return t


def quaternions(raw):
    """SFLP words -> [w,x,y,z], the same reconstruction payloadToQuat does in
    imu_device.cpp: x,y,z are halves, w = sqrt(1 - |xyz|²), and a sum of squares that
    rounds above 1 RENORMALISES xyz rather than being clipped."""
    if not raw:
        return np.empty((0, 4))
    xyz = (np.asarray(raw, dtype="<u2").view("<f2").astype(np.float64).reshape(-1, 3))
    sumsq = (xyz ** 2).sum(axis=1)
    over = sumsq > 1.0
    if over.any():
        xyz[over] /= np.sqrt(sumsq[over])[:, None]
        sumsq[over] = 1.0
    return np.column_stack([np.sqrt(1.0 - sumsq), xyz])


def hold_forward(vals, idx, first):
    """The value that was current at each accel sample. idx = -1 (no such word had
    arrived yet) gets `first`."""
    out = np.tile(np.asarray(first, dtype=np.float64), (len(idx), 1))
    if len(vals) == 0:
        return out
    i = np.asarray(idx)
    ok = i >= 0
    out[ok] = vals[i[ok]]
    return out


def drop_nan_quats(q):
    """The NaN guard from the pop loop: a corrupt FIFO word decodes to an invalid half,
    and the firmware then keeps the LAST VALID quaternion instead of letting the NaN
    through. Here that is a forward fill over the valid rows."""
    if len(q) == 0:
        return q, 0
    bad = ~np.isfinite(q).all(axis=1)
    if not bad.any():
        return q, 0
    idx = np.where(~bad, np.arange(len(q)), 0)
    np.maximum.accumulate(idx, out=idx)
    return q[idx], int(bad.sum())


def resolve(path):
    """Accept a .bin, or a session directory holding one."""
    if os.path.isdir(path):
        cand = sorted(glob(os.path.join(path, "*_raw.bin")))
        if not cand:
            sys.exit(f"found no *_raw.bin in {path}")
        return cand[0]
    if not os.path.isfile(path):
        sys.exit(f"found no {path}")
    return path


def summarise(hdr, d, t_acc):
    def rate(t):
        return len(t) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else float("nan")

    n_acc = len(d["acc"])
    duration = t_acc[-1] - t_acc[0] if n_acc > 1 else 0.0
    lines = [
        f"format v{hdr['version']}, reading {hdr['reading_id']}, "
        f"start {hdr['t0_utc']} (UTC epoch)",
        f"cfg: imu_odr {hdr['imu_odr_hz']} Hz, sflp_odr {hdr['sflp_odr_hz']} Hz, "
        f"acc {hdr['acc_sens_mg']:g} mg/LSB, gyr {hdr['gyr_sens_mdps']:g} mdps/LSB",
        f"accel {n_acc:7d} samples, measured {rate(t_acc):7.2f} Hz over {duration:.1f} s",
        f"gyro  {len(d['gyr']):7d} words",
        f"sflp  {len(d['quat']):7d} words",
        f"sync  {len(d['sync']):7d} drains, {len(d['ovf_at'])} with a FIFO overflow",
    ]
    if len(d["sync"]) > 1:
        ms = np.diff([s["millis"] for s in d["sync"]])
        # The headroom comes from the capture's OWN word rate: 256 words is 213 ms at
        # 480 Hz but only 118 ms at 960, and a fixed number would call a 150 ms stall
        # harmless in exactly the configuration where it is not.
        words = 2.0 * hdr["imu_odr_hz"] + hdr["sflp_odr_hz"]
        headroom = FIFO_DEPTH_WORDS / words * 1000.0 if words else float("nan")
        over = int((ms > headroom).sum())
        lines.append(f"drains: median {np.median(ms):.0f} ms, max {ms.max():.0f} ms "
                     f"(FIFO headroom {headroom:.0f} ms at {words:.0f} words/s)")
        if over:
            lines.append(f"        {over} drains ({100 * over / len(ms):.2f} %) over the "
                         f"headroom - samples may have been lost there")
    if d["unknown"]:
        lines.append(f"unknown tags: {d['unknown']}")
    if d["tail_at"]:
        o, reason = d["tail_at"]
        lines.append(f"interrupted capture: read {o} B of {d['file_bytes']} - the rest is "
                     f"not this capture's ({reason})")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("path", help="<stamp>_raw.bin or the session directory")
    ap.add_argument("-o", "--out",
                    help="write here (default: <stamp>_imu_from_raw.csv beside the .bin)")
    ap.add_argument("--allow-damaged", action="store_true",
                    help="convert even though the raw log lost blocks. Everything past "
                         "the first loss is misparsed")
    args = ap.parse_args()

    binpath = resolve(args.path)
    with open(binpath, "rb") as f:
        buf = f.read()
    hdr = read_header(buf)
    d = parse(buf, hdr)

    n_acc = len(d["acc"])
    if n_acc == 0:
        sys.exit("raw: no accel words in the file")
    t_acc = accel_times(n_acc, d["sync_n"], d["sync_t"], hdr["imu_odr_hz"])
    print(summarise(hdr, d, t_acc))

    # A HARD stop, deliberately. Converting a file that has lost bytes does not break
    # visibly - it produces a full set of columns and clean-looking numbers, computed
    # from payload bytes read as tags.
    if d["n_write_fail"]:
        msg = (f"the raw log lost {d['n_write_fail']} blocks and is desynchronised from "
               f"accel sample {d['first_write_fail_n']} of {n_acc}")
        if not args.allow_damaged:
            sys.exit(f"ABORTING: {msg}.\n  See also raw_write_failures in ana.csv. Use "
                     f"--allow-damaged to look at the part that is intact.")
        print(f"  [!] --allow-damaged: {msg}")

    acc = np.asarray(d["acc"], dtype=np.float64) * hdr["acc_sens_mg"]
    gyr_all = np.asarray(d["gyr"], dtype=np.float64) * hdr["gyr_sens_mdps"] \
        if d["gyr"] else np.empty((0, 3))
    quat_all, n_nan = drop_nan_quats(quaternions(d["quat"]))
    if n_nan:
        print(f"  [!] {n_nan} SFLP words were invalid halves - the last valid "
              f"quaternion is held, as on board")

    gyr = hold_forward(gyr_all, d["acc_gyr_i"], (0.0, 0.0, 0.0))
    quat = hold_forward(quat_all, d["acc_quat_i"], (1.0, 0.0, 0.0, 0.0))
    if len(quat_all) == 0:
        print("  [!] no SFLP words in the file (kEnableSflp off?) - q*_sflp stay unit")

    # fifo_ovf is marked on the FIRST sample of the drain that reported the overflow:
    # the loss happened BEFORE that sample, between this drain and the previous one. The
    # column comes from the sensor's status register, so the raw log is its only source.
    ovf = np.zeros(n_acc, dtype=np.int8)
    for a_n in d["ovf_at"]:
        if 0 <= a_n < n_acc:
            ovf[a_n] = 1

    out = args.out or binpath.replace("_raw.bin", "_imu_from_raw.csv")
    with open(out, "w", newline="") as f:
        f.write(",".join(COLUMNS) + "\n")
        for i in range(n_acc):
            f.write(f"{i},{t_acc[i]:.6f},"
                    f"{acc[i, 0]:.4f},{acc[i, 1]:.4f},{acc[i, 2]:.4f},"
                    f"{gyr[i, 0]:.4f},{gyr[i, 1]:.4f},{gyr[i, 2]:.4f},"
                    f"{quat[i, 0]:.6f},{quat[i, 1]:.6f},{quat[i, 2]:.6f},"
                    f"{quat[i, 3]:.6f},{ovf[i]}\n")
    print(f"\nwrote {out}  ({n_acc} rows, {len(COLUMNS)} columns)")


if __name__ == "__main__":
    main()
