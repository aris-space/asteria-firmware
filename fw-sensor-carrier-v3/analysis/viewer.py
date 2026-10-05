"""Writes a 3D replay of one SD logging session as a single HTML file and opens it.

The board moves along the GNSS track at the estimated height, turned by the
estimated attitude, with height, vertical velocity and the log's marks shown
alongside. The file carries its data, so it can be sent on as it is.

    uv run viewer.py /Volumes/SD [--replay] [--out replay.html]

With `--replay`, height and velocity come from `replay.py` (the current
estimator settings) instead of the estimate logged on the board; the attitude
always comes from the board.
"""

import argparse
import json
import webbrowser
from pathlib import Path

import numpy as np
import pandas as pd

import sdlog

TEMPLATE = Path(__file__).with_name("viewer.html")
EARTH_RADIUS_M = 6_371_000.0
# The estimator has no horizontal state, so the path is the GNSS track averaged
# over this long.
TRACK_SMOOTHING_S = 1.3


def track_en_m(log: sdlog.Log, t: np.ndarray) -> np.ndarray:
    """East and north of the receiver with the most fixes, relative to its
    first fix, at times `t`; zero without any fix."""
    fixes = log.gnss[log.gnss.fix_ok == 1]
    if fixes.empty:
        return np.zeros((len(t), 2))
    fixes = fixes[fixes.gnss == fixes.gnss.value_counts().idxmax()]
    lat0, lon0 = np.radians(fixes.latitude_deg.iloc[0]), np.radians(fixes.longitude_deg.iloc[0])
    east = (np.radians(fixes.longitude_deg) - lon0) * EARTH_RADIUS_M * np.cos(lat0)
    north = (np.radians(fixes.latitude_deg) - lat0) * EARTH_RADIUS_M
    en = pd.DataFrame({"east": east.to_numpy(), "north": north.to_numpy()}, index=pd.to_timedelta(fixes.t, unit="s"))
    en = en.rolling(pd.Timedelta(seconds=TRACK_SMOOTHING_S), center=True).mean()
    return np.c_[np.interp(t, fixes.t, en.east), np.interp(t, fixes.t, en.north)]


def estimate(log: sdlog.Log, use_replay: bool) -> pd.DataFrame:
    """The selected chain's estimate, with height and velocity replaced by a
    replay's if asked."""
    state = log.state[log.state.selected == 1].reset_index(drop=True)
    if state.empty:
        raise SystemExit(f"{log.dir / 'STATE.CSV'} has no rows")
    if use_replay:
        import replay

        replayed = replay.replay(log)
        replayed = replayed[replayed.selected]
        for column in ["height_msl_m", "height_std_m", "velocity_mps", "velocity_std_mps"]:
            state[column] = np.interp(state.t, replayed.t, replayed[column])
    return state


def data(log: sdlog.Log, use_replay: bool) -> dict:
    state = estimate(log, use_replay)
    t = state.t.to_numpy()
    en = track_en_m(log, t)
    rounded = lambda values, digits: np.round(np.asarray(values, dtype=float), digits).tolist()
    return dict(
        title=log.dir.name + (" (replayed)" if use_replay else ""),
        t=rounded(t, 3),
        position_enu_m=rounded(np.c_[en, state.height_msl_m], 3),
        orientation_body_to_ned_wxyz=rounded(state[["qw", "qx", "qy", "qz"]], 5),
        readouts=[
            dict(label="height", unit="m", values=rounded(state.height_msl_m, 2), std=rounded(state.height_std_m, 2)),
            dict(
                label="velocity up",
                unit="m/s",
                values=rounded(state.velocity_mps, 2),
                std=rounded(state.velocity_std_mps, 2),
            ),
        ],
        marks=[[round(m.t, 3), m.label] for m in log.marks.itertuples()],
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="a LOGnnnn directory, or the SD card root to use its newest session")
    parser.add_argument("--replay", action="store_true", help="height and velocity from replay.py")
    parser.add_argument("--out", type=Path, help="where to write the HTML (default: viewer.html in the log)")
    args = parser.parse_args()

    log = sdlog.read(args.log)
    out = args.out or log.dir / "viewer.html"
    payload = "const DATA = " + json.dumps(data(log, args.replay), separators=(",", ":")) + ";"
    out.write_text(TEMPLATE.read_text().replace("/* DATA */", payload))
    print(f"wrote {out}")
    webbrowser.open(out.resolve().as_uri())


if __name__ == "__main__":
    main()
