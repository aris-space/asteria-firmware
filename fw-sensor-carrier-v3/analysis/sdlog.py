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
        """The samples of one sensor, e.g. `sensor("imu", 0)`."""
        table = getattr(self, name)
        return table[table[name] == index]


def read(path: str | Path) -> Log:
    """Reads `path`, a LOGnnnn directory or one holding them (such as the SD
    card root, in which case the newest session is read)."""
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
    """The rows of a CSV file; rows that do not parse, e.g. one cut off by
    power loss, are skipped and counted. A missing or empty file has no rows."""
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()
    table = pd.read_csv(path, on_bad_lines="skip", dtype=str)
    numeric = [c for c in table.columns if c != "label"]
    table[numeric] = table[numeric].apply(pd.to_numeric, errors="coerce")
    complete = table.dropna()
    if len(complete) < len(table):
        print(f"  skipped {len(table) - len(complete)} malformed rows in {path.name}")
    return complete.reset_index(drop=True)


def pressure_altitude_m(pressure_mbar):
    """ISA pressure altitude, the same conversion the firmware uses."""
    return 44_330.0 * (1.0 - (np.asarray(pressure_mbar) / 1_013.25) ** 0.190_294_95)
