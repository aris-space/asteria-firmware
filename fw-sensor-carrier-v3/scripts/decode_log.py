#!/usr/bin/env python3
"""Decode a binary SD log produced by the sensor-carrier firmware SD logger.

Format (see app/src/tasks/sd.rs):
  8-byte header b"SCV3LOG\\x01", then a stream of little-endian records, each
  framed as 0xA5 0x5A <tag:u8> <payload>.

  tag 0x01 IMU  : sensor_id:u8, ts_us:u64, accel_xyz:f32x3 (g), gyro_xyz:f32x3 (dps)
  tag 0x02 GNSS : sensor_id:u8, ts_us:u64, lat:f64, lon:f64, fix:u8, sats:u8,
                  h_ellipsoid:f32, h_msl:f32, heading:f32, heading_acc:f32,
                  heading_veh:f32, vn:f32, ve:f32, vd:f32, pdop:u16, v_acc:u32,
                  h_acc:u32, mag_decl:f32, mag_decl_acc:f32
  tag 0x03 EKF  : ts_us:u64, pos_ned:f32x3, vel_ned:f32x3, quat_wxyz:f32x4,
                  cov_diag:f32x9, predict_us:f32
  tag 0x04 GNSS correction : ts_us:u64, residual:f32x3 (m), innovation_cov:f32x3
                  (m^2), adapted_var:f32x3 (m^2; NaN when not adaptive / rejected),
                  correct_us:f32

Writes imu.csv, gnss.csv, ekf.csv, gnss_corr.csv into the output directory.

Usage: python3 decode_log.py LOG00000.BIN [-o OUTDIR]
"""
import argparse
import csv
import struct
import sys
from pathlib import Path

MAGIC = b"SCV3LOG\x01"
SYNC = b"\xA5\x5A"
# Firmware pads the header to a full SD sector so record data is 512-aligned.
HEADER_LEN = 512

TAG_IMU, TAG_GNSS, TAG_EKF, TAG_GNSS_CORR = 0x01, 0x02, 0x03, 0x04

IMU_FMT = struct.Struct("<BQ6f")
GNSS_FMT = struct.Struct("<BQdd" + "BB" + "f" * 8 + "H" + "II" + "ff")
EKF_FMT = struct.Struct("<Q" + "f" * 19 + "f")
GNSS_CORR_FMT = struct.Struct("<Q" + "f" * 9 + "f")

GPS_FIX = {
    0: "NoFix", 1: "DeadReckoningOnly", 2: "Fix2D", 3: "Fix3D",
    4: "GPSPlusDeadReckoning", 5: "TimeOnlyFix",
}

IMU_COLS = ["t_s", "sensor", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]
GNSS_COLS = [
    "t_s", "sensor", "lat_deg", "lon_deg", "fix", "sats", "h_ellipsoid_m",
    "h_msl_m", "heading_deg", "heading_acc_deg", "heading_veh_deg",
    "vn_mps", "ve_mps", "vd_mps", "pdop", "v_acc_mm", "h_acc_mm",
    "mag_decl_deg", "mag_decl_acc_deg",
]
EKF_COLS = (
    ["t_s", "n_m", "e_m", "d_m", "vn_mps", "ve_mps", "vd_mps",
     "qw", "qx", "qy", "qz"]
    + [f"cov{i}" for i in range(9)]
    + ["predict_us"]
)
GNSS_CORR_COLS = [
    "t_s", "res_n_m", "res_e_m", "res_d_m",
    "S_n_m2", "S_e_m2", "S_d_m2",
    "Rhat_n_m2", "Rhat_e_m2", "Rhat_d_m2",
    "correct_us",
]


def decode(data: bytes):
    if data[:len(MAGIC)] != MAGIC:
        sys.exit(f"bad header: {data[:len(MAGIC)]!r} (expected {MAGIC!r})")
    imu, gnss, ekf, gcorr = [], [], [], []
    i = HEADER_LEN
    n = len(data)
    resyncs = 0
    while i + 3 <= n:
        if data[i:i + 2] != SYNC:
            i += 1  # scan forward to the next sync marker
            resyncs += 1
            continue
        tag = data[i + 2]
        payload = i + 3
        if tag == TAG_IMU and payload + IMU_FMT.size <= n:
            sid, ts, ax, ay, az, gx, gy, gz = IMU_FMT.unpack_from(data, payload)
            imu.append([ts / 1e6, sid, ax, ay, az, gx, gy, gz])
            i = payload + IMU_FMT.size
        elif tag == TAG_GNSS and payload + GNSS_FMT.size <= n:
            f = GNSS_FMT.unpack_from(data, payload)
            sid, ts, lat, lon, fix, sats = f[0:6]
            rest = f[6:]
            gnss.append([ts / 1e6, sid, lat, lon, GPS_FIX.get(fix, fix), sats, *rest])
            i = payload + GNSS_FMT.size
        elif tag == TAG_EKF and payload + EKF_FMT.size <= n:
            f = EKF_FMT.unpack_from(data, payload)
            ekf.append([f[0] / 1e6, *f[1:]])
            i = payload + EKF_FMT.size
        elif tag == TAG_GNSS_CORR and payload + GNSS_CORR_FMT.size <= n:
            f = GNSS_CORR_FMT.unpack_from(data, payload)
            gcorr.append([f[0] / 1e6, *f[1:]])
            i = payload + GNSS_CORR_FMT.size
        else:
            i += 1  # unknown tag or truncated record; resync
            resyncs += 1
    return imu, gnss, ekf, gcorr, resyncs


def write_csv(path: Path, cols, rows):
    with path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        w.writerows(rows)


def continuity(label, times):
    """Report rate and timestamp gaps for one sorted stream of t_s values.

    A gap > 2x the median sample interval flags likely dropped samples
    (silent pubsub lag is invisible to the firmware's drop counter)."""
    if len(times) < 3:
        print(f"  {label}: {len(times)} records (too few for continuity check)")
        return
    times = sorted(times)
    dur = times[-1] - times[0]
    diffs = [b - a for a, b in zip(times, times[1:])]
    med = sorted(diffs)[len(diffs) // 2]
    rate = (len(times) - 1) / dur if dur > 0 else float("nan")
    thresh = 2 * med
    gaps = [(times[i], d) for i, d in enumerate(diffs) if d > thresh]
    missed = sum(round(d / med) - 1 for _, d in gaps)
    print(f"  {label}: {len(times)} records, {dur:.1f} s, {rate:.1f} Hz, "
          f"median dt {med * 1e3:.2f} ms")
    if gaps:
        worst_t, worst_d = max(gaps, key=lambda g: g[1])
        print(f"    {len(gaps)} gap(s) > {thresh * 1e3:.1f} ms; "
              f"~{missed} sample(s) missing; worst {worst_d * 1e3:.1f} ms at t={worst_t:.2f}s")
    else:
        print("    no gaps — continuous")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path)
    ap.add_argument("-o", "--outdir", type=Path, default=None,
                    help="output directory for CSVs (default: alongside input)")
    args = ap.parse_args()

    data = args.input.read_bytes()
    imu, gnss, ekf, gcorr, resyncs = decode(data)

    outdir = args.outdir or args.input.parent
    outdir.mkdir(parents=True, exist_ok=True)
    write_csv(outdir / "imu.csv", IMU_COLS, imu)
    write_csv(outdir / "gnss.csv", GNSS_COLS, gnss)
    write_csv(outdir / "ekf.csv", EKF_COLS, ekf)
    write_csv(outdir / "gnss_corr.csv", GNSS_CORR_COLS, gcorr)

    print(f"decoded {len(data)} bytes from {args.input}")
    print(f"  IMU  records: {len(imu)}")
    print(f"  GNSS records: {len(gnss)}")
    print(f"  EKF  records: {len(ekf)}")
    print(f"  GNSS corr   : {len(gcorr)}")
    if resyncs:
        print(f"  resync skips: {resyncs} byte(s) (truncated tail / corruption)")
    print(f"  CSVs written to {outdir}/")

    print("continuity:")
    for sid in sorted({r[1] for r in imu}):
        continuity(f"IMU{sid}", [r[0] for r in imu if r[1] == sid])
    for sid in sorted({r[1] for r in gnss}):
        continuity(f"GNSS{sid}", [r[0] for r in gnss if r[1] == sid])
    continuity("EKF", [r[0] for r in ekf])


if __name__ == "__main__":
    main()
