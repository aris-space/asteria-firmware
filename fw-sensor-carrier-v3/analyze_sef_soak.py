"""Summarize a stationary sensor-carrier SEF-light hardware log.

Usage: python3 analyze_sef_soak.py PATH_TO_LOG
"""

import argparse
import math
import re
import statistics
from collections import defaultdict


LINE = re.compile(r"^(?P<time>\d+\.\d+) \[[^]]+\] (?P<message>.*)$")
STATE = re.compile(
    r"SEF-light: altitude_msl=(?P<h>-?[\d.]+)±[^ ]+ m, "
    r"v=(?P<v>-?[\d.]+)±[^ ]+ m/s, IMU=(?P<imu>IMU_[01])"
)
GNSS = re.compile(
    r"(?P<source>GNSS_[01]) GNSS: PVT=(?P<pvt>\d+), STATUS=(?P<status>\d+), "
    r"report_ms=(?P<ms>\d+), max_epoch_gap=(?P<gap>\d+) ms, "
    r"MSL=(?P<h>-?[\d.]+) m, vAcc=(?P<vacc>\d+) mm, "
    r"vDown=(?P<vdown>-?[\d.]+) m/s, sAcc=(?P<sacc>-?[\d.]+) m/s, "
    r"PDOP=(?P<pdop>\d+), sats=(?P<sats>\d+), "
    r'fix="(?P<fix>[^\"]+)", fixOk=(?P<fixok>true|false)'
)
GNSS_SOURCE = re.compile(r"SEF GNSS source: (?P<source>GNSS_[01])")
BARO = re.compile(
    r"SEF baro (?P<source>BARO_BUS_[12]): "
    r"pressure_altitude=(?P<h>-?[\d.]+) m, pressure=(?P<pressure>-?[\d.]+) mbar"
)
IMU = re.compile(
    r"IMU (?P<source>IMU_[01]) bench: n=(?P<n>\d+), "
    r"gravity_error_mean=(?P<gravity>-?[\d.]+) m/s2, .*"
    r"gyro_mean=\[(?P<gyro>[^]]+)\] rad/s, "
    r"gyro_noise=(?P<noise>[\d.]+) rad/s, max_gyro=(?P<max_gyro>[\d.]+) rad/s"
)
MAG = re.compile(
    r"SEF mag (?P<source>MAG_BUS_[12]): field=(?P<field>[\d.]+) nT, "
    r"calibrated=(?P<calibrated>true|false), accepted=(?P<accepted>true|false)"
)
QUEUE = re.compile(
    r"imu_dropped=\[(?P<imu>[^]]+)\], aiding_dropped=\[(?P<aiding>[^]]+)\], "
    r"late=\[(?P<late>[^]]+)\], max_imu_backlog=\[(?P<backlog>[^]]+)\]"
)
ATTITUDE = re.compile(
    r"SEF (?P<source>IMU_[01]): h=(?P<h>-?[\d.]+) dh=[^,]+, "
    r"v=(?P<v>-?[\d.]+) dv=[^,]+, .*"
    r"score=(?P<score>[\d.]+), q=\[(?P<q>[^]]+)\].*"
    r"mag_ignored=(?P<ignored>true|false)"
)
PATTERNS = (
    ("state", STATE), ("gnss", GNSS), ("gnss_source", GNSS_SOURCE), ("baro", BARO),
    ("imu", IMU), ("mag", MAG), ("queue", QUEUE),
    ("attitude", ATTITUDE),
)


def mean_sd(values):
    if not values:
        return "n=0"
    return (
        f"n={len(values)} mean={statistics.fmean(values):.3f} "
        f"sd={statistics.pstdev(values):.3f} "
        f"range={min(values):.3f}..{max(values):.3f}"
    )


def yaw_deg(quaternion):
    w, x, y, z = quaternion
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def quaternion_separation_deg(first, second):
    dot = sum(a * b for a, b in zip(first, second))
    norm = math.sqrt(sum(value * value for value in first) * sum(value * value for value in second))
    return math.degrees(2 * math.acos(min(1.0, abs(dot) / norm)))


def unwrap_degrees(values):
    if not values:
        return 0.0
    total = 0.0
    for previous, current in zip(values, values[1:]):
        total += (current - previous + 180) % 360 - 180
    return total


def parse(path):
    rows = defaultdict(list)
    with open(path, errors="replace") as source:
        for line in source:
            match = LINE.match(line)
            if match is None:
                continue
            time = float(match["time"])
            message = match["message"]
            for kind, pattern in PATTERNS:
                item = pattern.search(message)
                if item is not None:
                    rows[kind].append((time, item.groupdict()))
                    break
            if any(word in message for word in ("read error", "parse error", "offline", "recovered")):
                rows["fault"].append((time, message))
    return rows


def in_window(rows, start, end, source=None):
    return [
        item for time, item in rows
        if start <= time < end and (source is None or item.get("source") == source)
    ]


def gnss_qualified(item):
    return (
        item["fixok"] == "true"
        and item["fix"] in ("Fix3D", "GPSPlusDeadReckoning")
        and int(item["vacc"]) <= 3_000
        and int(item["pdop"]) <= 600
    )


def window_motion(rows, start, end, height_key, velocity_key=None, velocity_sign=1):
    samples = [(time, item) for time, item in rows if start <= time < end]
    if len(samples) < 120:
        return "insufficient samples"
    first = [float(item[height_key]) for time, item in samples if time < start + 60]
    last = [float(item[height_key]) for time, item in samples if time >= end - 60]
    if not first or not last:
        return "incomplete window"
    height_change = statistics.fmean(last) - statistics.fmean(first)
    result = f"height_change={height_change:+.3f} m (last versus first 60 s)"
    if velocity_key is not None:
        distance = sum(
            (next_time - time)
            * velocity_sign
            * (float(item[velocity_key]) + float(next_item[velocity_key]))
            / 2
            for (time, item), (next_time, next_item) in zip(samples, samples[1:])
        )
        result += f", velocity_integral={distance:+.3f} m"
    return result


def report(rows, start, end):
    print(f"\n{start / 60:.0f}-{end / 60:.0f} min")
    state = in_window(rows["state"], start, end)
    print("  fused_msl_m", mean_sd([float(item["h"]) for item in state]))
    print("  fused_v_mps", mean_sd([float(item["v"]) for item in state]))
    print("  fused_motion", window_motion(rows["state"], start, end, "h", "v"))
    for source in ("GNSS_0", "GNSS_1"):
        gnss = in_window(rows["gnss"], start, end, source)
        if gnss:
            rate = sum(int(item["pvt"]) for item in gnss) * 1000 / sum(int(item["ms"]) for item in gnss)
            print(
                f"  {source} PVT_Hz={rate:.2f} "
                f"max_gap_ms={max(int(item['gap']) for item in gnss)} "
                f"vAcc_median_m={statistics.median(int(item['vacc']) for item in gnss)/1000:.2f} "
                f"PDOP_median={statistics.median(int(item['pdop']) for item in gnss)/100:.2f} "
                f"sats_median={statistics.median(int(item['sats']) for item in gnss):.1f}"
            )
            qualified = sum(gnss_qualified(item) for item in gnss)
            print(f"  {source}_quality_reports={qualified}/{len(gnss)}")
            print(f"  {source}_msl_m", mean_sd([float(item["h"]) for item in gnss]))
            print(f"  {source}_vup_mps", mean_sd([-float(item["vdown"]) for item in gnss]))
            source_rows = [(time, item) for time, item in rows["gnss"] if item["source"] == source]
            print(f"  {source}_motion", window_motion(source_rows, start, end, "h", "vdown", -1))
    for source in ("BARO_BUS_1", "BARO_BUS_2"):
        baro = in_window(rows["baro"], start, end, source)
        print(f"  {source}_pressure_alt_m", mean_sd([float(item["h"]) for item in baro]))
        source_rows = [(time, item) for time, item in rows["baro"] if item["source"] == source]
        print(f"  {source}_motion", window_motion(source_rows, start, end, "h"))
    for source in ("IMU_0", "IMU_1"):
        imu = in_window(rows["imu"], start, end, source)
        if imu:
            gyro_mean = [
                math.degrees(statistics.fmean(float(item["gyro"].split(",")[axis]) for item in imu))
                for axis in range(3)
            ]
            print(
                f"  {source} gravity_residual_mps2={statistics.fmean(float(item['gravity']) for item in imu):.4f} "
                f"gyro_mean_dps={[round(value, 4) for value in gyro_mean]} "
                f"max_gyro_rad_s={max(float(item['max_gyro']) for item in imu):.4f}"
            )
    for source in ("MAG_BUS_1", "MAG_BUS_2"):
        mag = in_window(rows["mag"], start, end, source)
        if mag:
            accepted = sum(item["accepted"] == "true" for item in mag)
            print(
                f"  {source}_field_nt",
                mean_sd([float(item["field"]) for item in mag]),
                f"accepted={accepted}/{len(mag)}",
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    args = parser.parse_args()
    rows = parse(args.log)
    end = max((time for records in rows.values() for time, _ in records), default=0.0)
    queues = rows["queue"]
    bad_queues = []
    max_backlog = 0
    for time, item in queues:
        if any(int(value) for key in ("imu", "aiding", "late") for value in item[key].split(",")):
            bad_queues.append(time)
        max_backlog = max(max_backlog, *(int(value) for value in item["backlog"].split(",")))
    print(
        f"elapsed_s={end:.1f} queue_reports={len(queues)} "
        f"queue_failures={len(bad_queues)} max_imu_backlog={max_backlog}"
    )
    print(
        f"faults_total={len(rows['fault'])} "
        f"faults_after_1s={sum(time > 1 for time, _ in rows['fault'])}"
    )
    print("gnss_source_changes", [(round(time, 1), item["source"]) for time, item in rows["gnss_source"]])
    for source in ("GNSS_0", "GNSS_1"):
        bad_run = 0
        longest_bad_run = 0
        reports = [(time, item) for time, item in rows["gnss"] if item["source"] == source]
        for _, item in reports:
            bad_run = 0 if gnss_qualified(item) else bad_run + 1
            longest_bad_run = max(longest_bad_run, bad_run)
        qualified = sum(gnss_qualified(item) for _, item in reports)
        print(
            f"{source} qualified_reports={qualified}/{len(reports)} "
            f"longest_unqualified_reports={longest_bad_run}"
        )
    selected = [(time, item["imu"]) for time, item in rows["state"]]
    switches = [(time, imu) for (time, imu), (_, old) in zip(selected[1:], selected[:-1]) if imu != old]
    print(f"imu_switches={switches}")
    for source in ("IMU_0", "IMU_1"):
        attitude = in_window(rows["attitude"], 0, end + 1, source)
        angles = [yaw_deg([float(value) for value in item["q"].split(",")]) for item in attitude]
        ignored = sum(item["ignored"] == "true" for item in attitude)
        print(
            f"{source} yaw_change_deg={unwrap_degrees(angles):.2f} "
            f"mag_ignored={ignored}/{len(attitude)}"
        )
    paired_scores = []
    paired_attitudes = []
    for (first_time, first), (second_time, second) in zip(rows["attitude"][::2], rows["attitude"][1::2]):
        if first["source"] == "IMU_0" and second["source"] == "IMU_1" and second_time - first_time < 0.01:
            paired_scores.append(float(second["score"]) - float(first["score"]))
            paired_attitudes.append((second_time, first, second))
    if paired_scores:
        print(
            f"imu_score_delta_1_minus_0 median={statistics.median(paired_scores):+.4f} "
            f"min={min(paired_scores):+.4f} max={max(paired_scores):+.4f} "
            f"IMU_1_better_by_0.003={sum(delta < -0.003 for delta in paired_scores)}/{len(paired_scores)}"
        )
    for switch_time, selected_imu in switches:
        if not paired_attitudes:
            break
        pair_time, first, second = min(paired_attitudes, key=lambda pair: abs(pair[0] - switch_time))
        if abs(pair_time - switch_time) > 0.3:
            continue
        first_q = [float(value) for value in first["q"].split(",")]
        second_q = [float(value) for value in second["q"].split(",")]
        angle = quaternion_separation_deg(first_q, second_q)
        height = abs(float(first["h"]) - float(second["h"]))
        velocity = abs(float(first["v"]) - float(second["v"]))
        print(
            f"handover t={switch_time:.1f}s to={selected_imu} "
            f"orientation_separation={angle:.2f}deg "
            f"height_separation={height:.3f}m velocity_separation={velocity:.3f}m/s"
        )
    for start in range(0, int(end) + 1, 600):
        report(rows, start, start + 600)


if __name__ == "__main__":
    main()
