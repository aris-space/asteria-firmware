"""Writes a self-contained 3D replay of one log as HTML and opens it: the board
along the GNSS track at the estimated height and attitude. `--replay` uses the
current estimator settings instead of the on-board estimate; `--cal` also
applies `cal set` lines.

    uv run viewer.py /Volumes/SD [--replay] [--cal cal.txt] [--out replay.html]
"""

import argparse
import json
import webbrowser
from pathlib import Path

import numpy as np
import pandas as pd

import recalibrate
import sdlog

TEMPLATE = Path(__file__).with_name("viewer.html")
EARTH_RADIUS_M = 6_371_000.0
# The estimator has no horizontal state, so the path is the GNSS track averaged
# over this long.
TRACK_SMOOTHING_S = 1.3


def track_en_m(log: sdlog.Log, t: np.ndarray) -> np.ndarray:
    """East and north of the receiver with the most fixes, relative to its
    first fix, at times `t`; zero without any fix."""
    fixes = log.gnss[sdlog.usable_fix(log.gnss)] if len(log.gnss) else log.gnss
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
    """The selected chain's estimate, logged on the board or replayed."""
    if use_replay:
        import replay

        states = replay.replay(log)
        state = states[states.selected]
        # Only the attitude's start-up is skipped; a later reset shows as it happened.
        state = state[state.ready.cummax()].reset_index(drop=True)
    else:
        state = log.state[log.state.selected == 1].reset_index(drop=True)
    if state.empty:
        raise SystemExit(f"{log.dir}: no estimate")
    return state


def data(log: sdlog.Log, use_replay: bool, title: str) -> dict:
    state = estimate(log, use_replay)
    t = state.t.to_numpy()
    en = track_en_m(log, t)
    rounded = lambda values, digits: np.round(np.asarray(values, dtype=float), digits).tolist()
    return dict(
        title=title,
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
    parser.add_argument("--replay", action="store_true", help="the estimate of replay.py instead of the logged one")
    parser.add_argument("--cal", type=Path, help="a file of `cal set` lines to apply; implies --replay")
    parser.add_argument("--out", type=Path, help="where to write the HTML (default: viewer.html in the log)")
    args = parser.parse_args()

    log = sdlog.read(args.log)
    if args.cal:
        log = recalibrate.apply(log, args.cal.read_text())
    out = args.out or log.dir / "viewer.html"
    title = log.dir.name + (f" replayed with {args.cal.name}" if args.cal else " replayed" if args.replay else "")
    payload = "const DATA = " + json.dumps(data(log, args.replay or bool(args.cal), title), separators=(",", ":")) + ";"
    out.write_text(TEMPLATE.read_text().replace("/* DATA */", payload))
    print(f"wrote {out}")
    webbrowser.open(out.resolve().as_uri())


if __name__ == "__main__":
    main()
