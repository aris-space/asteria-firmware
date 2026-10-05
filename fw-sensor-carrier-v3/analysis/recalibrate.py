"""Applies `cal set` lines to a logged session, as if the board had had them.

The calibrated columns (`cal_us`, `ax_g`, ..., `x_nt`, ...) are recomputed from
the raw ones the way the firmware does, so a replay shows what a calibration
would have done before it is pasted into the board:

    log = recalibrate.apply(sdlog.read("/Volumes/SD"), Path("cal.txt").read_text())

Sensors without a line keep the calibration they were logged with.
"""

import re

import numpy as np

import sdlog

# Console sensor name to SD table and row index.
SENSORS = {
    "IMU_0": ("imu", 0),
    "IMU_1": ("imu", 1),
    "MAG_BUS_1": ("mag", 0),
    "MAG_BUS_2": ("mag", 1),
    "BARO_BUS_1": ("baro", 0),
    "BARO_BUS_2": ("baro", 1),
    "GNSS_0": ("gnss", 0),
    "GNSS_1": ("gnss", 1),
}
LINE = re.compile(r"^\s*cal set (\S+)((?:\s+\S+=\S+)+)\s*$")


def parse(text: str) -> dict[str, dict[str, list[float]]]:
    """The fields of every `cal set` line, by sensor; other lines are ignored."""
    lines = {}
    for line in text.splitlines():
        match = LINE.match(line)
        if not match or match[1] not in SENSORS:
            continue
        fields = dict(field.split("=", 1) for field in match[2].split())
        lines[match[1]] = {key: [float(v) for v in value.split(",")] for key, value in fields.items() if key != "name"}
    return lines


def apply(log: sdlog.Log, text: str) -> sdlog.Log:
    """`log` with the calibration of every `cal set` line in `text` applied."""
    for sensor, fields in parse(text).items():
        name, index = SENSORS[sensor]
        table = getattr(log, name).copy()
        rows = table[name] == index
        table.loc[rows, "cal_us"] = table.loc[rows, "raw_us"] - int(fields["latency_us"][0])
        if name == "imu":
            imu = table[rows]
            accel = (sdlog.imu_board_g(imu) - fields["accel_offset_g"]) * fields["accel_scale"]
            gyro = sdlog.imu_board_dps(imu) - fields["gyro_bias_dps"]
            table.loc[rows, ["ax_g", "ay_g", "az_g"]] = accel
            table.loc[rows, ["gx_dps", "gy_dps", "gz_dps"]] = gyro
        elif name == "mag":
            board_nt = sdlog.mag_board_counts(table[rows]) * sdlog.MAG_NT_PER_LSB
            soft_iron = np.reshape(fields["soft"], (3, 3))
            table.loc[rows, ["x_nt", "y_nt", "z_nt"]] = (board_nt - fields["hard_nt"]) @ soft_iron.T
        setattr(log, name, table)
    return log
