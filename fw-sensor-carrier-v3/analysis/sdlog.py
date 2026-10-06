"""Reads one fw-sensor-carrier-v3 SD logging session (a LOGnnnn directory).

Every table gets a column `t`: seconds since the first IMU sample. Sensor
tables take it from `raw_us`, so fits do not depend on the latencies stored on
the board; STATE.CSV takes it from `cal_us` and MARKS.CSV from `uptime_us`.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

STANDARD_GRAVITY = 9.806_65
# Raw-count units and sensor-to-board axes, as in the firmware's
# calibration/<kind>.rs.
GYRO_DPS_PER_LSB = 0.07
# Per accelerometer range; a log's range is recognised from its data.
ACCEL_G_PER_LSB = {8: 1 / 4096, 16: 1 / 2048}
MAG_NT_PER_LSB = 150.0


@dataclass
class Log:
    dir: Path
    imu: pd.DataFrame
    mag: pd.DataFrame
    baro: pd.DataFrame
    gnss: pd.DataFrame
    state: pd.DataFrame
    marks: pd.DataFrame
    drops: pd.DataFrame

    def crop(self, start: float, end: float) -> None:
        """Keeps only sensor samples within `[start, end]` seconds."""
        for name in ["imu", "mag", "baro", "gnss"]:
            table = getattr(self, name)
            setattr(self, name, table[table.t.between(start, end)])

    def sensor(self, name: str, index: int) -> pd.DataFrame:
        table = getattr(self, name)
        return table[table[name] == index]


def read(path: str | Path) -> Log:
    """A LOGnnnn directory, or the newest one under `path` (e.g. the SD card root)."""
    dir = session_dir(Path(path))
    tables = {name: read_csv(dir / f"{name.upper()}.CSV") for name in Log.__annotations__ if name != "dir"}
    if tables["imu"].empty:
        raise SystemExit(f"{dir / 'IMU.CSV'} has no rows")
    start_us = tables["imu"].raw_us.min()
    for name, column in [
        ("imu", "raw_us"),
        ("mag", "raw_us"),
        ("baro", "raw_us"),
        ("gnss", "raw_us"),
        ("state", "cal_us"),
        ("marks", "uptime_us"),
    ]:
        table = tables[name]
        table["t"] = (table[column] - start_us) * 1e-6 if not table.empty else pd.Series(dtype=float)
        tables[name] = table.sort_values("t", kind="stable", ignore_index=True)
    return Log(dir=dir, **tables)


def session_dir(path: Path) -> Path:
    if (path / "IMU.CSV").exists():
        return path
    sessions = sorted(d for d in path.glob("LOG[0-9][0-9][0-9][0-9]") if (d / "IMU.CSV").exists())
    if not sessions:
        raise SystemExit(f"no LOGnnnn directory with IMU.CSV in {path}")
    return sessions[-1]


def read_csv(path: Path) -> pd.DataFrame:
    """Skips (and counts) rows that don't parse, e.g. one cut off by power loss."""
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()
    table = pd.read_csv(path, on_bad_lines="skip", dtype=str)
    numeric = [c for c in table.columns if c != "label"]
    table[numeric] = table[numeric].apply(pd.to_numeric, errors="coerce")
    complete = table.dropna()
    if len(complete) < len(table):
        print(f"  skipped {len(table) - len(complete)} malformed rows in {path.name}")
    return complete.reset_index(drop=True)


def usable_fix(gnss: pd.DataFrame) -> pd.Series:
    """The fixes the firmware passes to the estimator: 3D (u-blox 3D or
    GNSS+DR), and of each receiver only those newer than all its fixes before;
    older logs hold stale ones that a UART error replayed."""
    has_fix = (gnss.fix_ok != 0) & gnss.fix_type.isin([2, 3, 4])
    itow = gnss.itow_ms.where(has_fix)
    newest_before = itow.groupby(gnss.gnss).transform(lambda s: s.cummax().shift())
    return has_fix & gnss.fix_type.isin([3, 4]) & ~(itow <= newest_before)


def pressure_altitude_m(pressure_mbar):
    """ISA pressure altitude, the same conversion the firmware uses."""
    return 44_330.0 * (1.0 - (np.asarray(pressure_mbar) / 1_013.25) ** 0.190_294_95)


def imu_board_dps(imu) -> np.ndarray:
    """LSM6DSO32 sensor-to-board remap: negate x and z."""
    return imu[["gx_raw", "gy_raw", "gz_raw"]].to_numpy() * [-1, 1, -1] * GYRO_DPS_PER_LSB


def accel_g_per_lsb(imu) -> float:
    """The log's accelerometer range, recognised from calibrated vs raw values
    (calibration moves them a few percent at most)."""
    raw = np.abs(imu["ax_raw"].to_numpy())
    big = raw > 1_000
    ratio = np.median(np.abs(imu["ax_g"].to_numpy()[big]) / raw[big])
    return min(ACCEL_G_PER_LSB.values(), key=lambda lsb: abs(np.log(ratio / lsb)))


def imu_board_g(imu) -> np.ndarray:
    """LSM6DSO32 sensor-to-board remap, as for the gyro."""
    return imu[["ax_raw", "ay_raw", "az_raw"]].to_numpy() * [-1, 1, -1] * accel_g_per_lsb(imu)


def mag_board_counts(mag) -> np.ndarray:
    """LSM303AGR sensor-to-board remap: negate all axes."""
    return -mag[["x_raw", "y_raw", "z_raw"]].to_numpy()
