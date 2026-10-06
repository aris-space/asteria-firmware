"""Replays an SD logging session through SEF-light, to tune the estimator offline.

The calibrated readings (`cal_us` and calibrated values) are fed to the same
estimator the firmware runs, in timestamp order and with the firmware's
settings unless overridden. From Python:

    import sdlog, replay
    states = replay.replay(sdlog.read("/Volumes/SD"), acceleration_noise_std_mps2=5.0)

or from the shell, with plots of the replayed and the on-board estimate:

    uv run replay.py /Volumes/SD [--set acceleration_noise_std_mps2=5.0 ...] [--cal cal.txt]

`--cal` replays as if the board had had the `cal set` lines in that file (see
recalibrate.py).
"""

import argparse
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sef_light import Estimator

import recalibrate
import sdlog

# The settings of fw-sensor-carrier-v3's tasks/state_estimation.rs.
FIRMWARE = dict(
    acceleration_noise_std_mps2=10.0,
    degraded_acceleration_noise_std_mps2=20.0,
    barometer_bias_walk_std_m_per_sqrt_s=[0.02, 0.02],
    initial_height_std_m=1_000.0,
    initial_velocity_std_mps=3.0,
    initial_barometer_bias_std_m=[200.0, 200.0],
    innovation_gate_sigma=5.0,
    ahrs_gain=2.0,
    gyroscope_range_deg_s=2_000.0,
    acceleration_rejection_deg=10.0,
    recovery_trigger_period=300,
    magnetic_rejection_deg=20.0,
    maximum_magnetometer_age_us=250_000,
    switch_hysteresis=0.0025,
    switch_dwell_us=5_000_000,
    score_memory=0.95,
    maximum_nis_contribution=25.0,
    degraded_score_penalty=10.0,
    maximum_imu_age_us=100_000,
    gnss_minimum_fix_tier=3,
    gnss_switch_dwell_us=500_000,
    maximum_aiding_delay_us=400_000,
    baro_height_std_m=1.5,
    gnss_height_std_scale=10.0,
    gnss_speed_std_scale=10.0,
)
# Smallest GNSS standard deviation passed on, for receivers reporting zero.
GNSS_MIN_STD = 0.1
# As on the board, the height is MSL-referenced once its standard deviation is
# below this.
MSL_REFERENCED_HEIGHT_STD_M = 100.0
# As on the board, magnetometer samples whose field strength differs from the
# calibrated one by more than this fraction are not fused, and neither are
# uncalibrated magnetometers. The log does not hold the calibrated field
# strength, so the median of the logged one stands in for it.
MAX_FIELD_ERROR = 0.1
OUTPUT_PERIOD_US = 50_000


def fix_tier(fix_ok: bool, fix_type: int) -> int:
    """SEF-light's convention: 3 is a usable 3D fix (u-blox 3D or GNSS+DR)."""
    if not fix_ok:
        return 0
    return {3: 3, 4: 3, 2: 2}.get(fix_type, 0)


def mag_calibrated(log: sdlog.Log, index: int) -> bool:
    """Whether magnetometer `index` was logged with a calibration other than
    the identity, which the firmware does not fuse."""
    mag = log.sensor("mag", index)
    identity_nt = sdlog.mag_board_counts(mag) * sdlog.MAG_NT_PER_LSB
    return len(mag) > 0 and not np.allclose(mag[["x_nt", "y_nt", "z_nt"]].to_numpy(), identity_nt)


def replay(log: sdlog.Log, **overrides) -> pd.DataFrame:
    """Both chains' state every 50 ms of log time, one row per chain."""
    settings = FIRMWARE | overrides
    baro_std = settings.pop("baro_height_std_m")
    gnss_height_scale = settings.pop("gnss_height_std_scale")
    gnss_speed_scale = settings.pop("gnss_speed_std_scale")
    estimator = Estimator(**settings)

    imu, mag, baro, gnss = log.imu, log.mag, log.baro, log.gnss
    mag = mag[mag.mag.isin([i for i in range(2) if mag_calibrated(log, i)])]
    field_nt = pd.Series(np.linalg.norm(mag[["x_nt", "y_nt", "z_nt"]].to_numpy(), axis=1), index=mag.index)
    expected_nt = field_nt.groupby(mag.mag).transform("median")
    mag = mag[(field_nt - expected_nt).abs() <= MAX_FIELD_ERROR * expected_nt]
    events = [
        *zip(imu.cal_us, ["imu"] * len(imu), imu.itertuples()),
        *zip(mag.cal_us, ["mag"] * len(mag), mag.itertuples()),
        *zip(baro.cal_us, ["baro"] * len(baro), baro.itertuples()),
        *zip(gnss.cal_us, ["gnss"] * len(gnss), gnss.itertuples()),
    ]
    events.sort(key=lambda event: event[0])

    errors = Counter()
    rows = []
    last_output = None
    for time_us, kind, s in events:
        time_us = int(time_us)
        try:
            if kind == "imu":
                accel = [
                    s.ax_g * sdlog.STANDARD_GRAVITY,
                    s.ay_g * sdlog.STANDARD_GRAVITY,
                    s.az_g * sdlog.STANDARD_GRAVITY,
                ]
                gyro = np.radians([s.gx_dps, s.gy_dps, s.gz_dps]).tolist()
                estimator.update_imu(int(s.imu), time_us, accel, gyro)
            elif kind == "mag":
                # Each magnetometer aids the attitude chain of the IMU with the same index.
                estimator.update_magnetometer(int(s.mag), time_us, [s.x_nt, s.y_nt, s.z_nt])
            elif kind == "baro":
                height = float(sdlog.pressure_altitude_m(s.pressure_mbar))
                estimator.update_pressure(time_us, int(s.baro), height, baro_std)
            else:
                estimator.update_gnss(
                    time_us,
                    int(s.gnss),
                    s.height_msl_m,
                    -s.velocity_down_mps,
                    max(s.vertical_accuracy_mm / 1000 * gnss_height_scale, GNSS_MIN_STD),
                    max(s.speed_accuracy_mps * gnss_speed_scale, GNSS_MIN_STD),
                    fix_tier(bool(s.fix_ok), int(s.fix_type)),
                    int(s.pdop_centi),
                )
        except ValueError as error:
            errors[(kind, str(error))] += 1
            continue
        if last_output is not None and time_us - last_output < OUTPUT_PERIOD_US:
            continue
        last_output = time_us
        selected = estimator.selected_imu()
        for chain in range(2):
            height, velocity, bias = estimator.state(chain)
            height_var, velocity_var, _ = estimator.uncertainty(chain)
            qw, qx, qy, qz = estimator.orientation_body_to_ned_wxyz(chain)
            rows.append(
                dict(
                    cal_us=time_us,
                    imu=chain,
                    selected=chain == selected,
                    ready=estimator.imu_ready(chain),
                    msl_ready=np.sqrt(height_var) < MSL_REFERENCED_HEIGHT_STD_M,
                    height_msl_m=height,
                    velocity_mps=velocity,
                    bias0_m=bias[0],
                    bias1_m=bias[1],
                    height_std_m=np.sqrt(height_var),
                    velocity_std_mps=np.sqrt(velocity_var),
                    score=estimator.consistency_scores()[chain],
                    qw=qw,
                    qx=qx,
                    qy=qy,
                    qz=qz,
                )
            )
    for (kind, error), count in errors.items():
        print(f"  {count} {kind} updates failed: {error}")
    states = pd.DataFrame(rows)
    states["t"] = (states.cal_us - log.imu.raw_us.min()) * 1e-6
    return states


def plot(log: sdlog.Log, states: pd.DataFrame) -> None:
    _, (height_ax, velocity_ax) = plt.subplots(2, 1, sharex=True, num="Replay")
    gnss = log.gnss[(log.gnss.fix_ok != 0) & log.gnss.fix_type.isin([3, 4])] if len(log.gnss) else log.gnss
    if len(gnss):
        height_ax.plot(gnss.t, gnss.height_msl_m, ".", markersize=2, label="GNSS")
        velocity_ax.plot(gnss.t, -gnss.velocity_down_mps, ".", markersize=2, label="GNSS")
    for chain in range(2):
        chain_states = states[(states.imu == chain) & states.ready]
        height_ax.plot(chain_states.t, chain_states.height_msl_m, label=f"replay IMU_{chain}")
        velocity_ax.plot(chain_states.t, chain_states.velocity_mps, label=f"replay IMU_{chain}")
    on_board = log.state[log.state.selected != 0] if len(log.state) else log.state
    if len(on_board):
        height_ax.plot(on_board.t, on_board.height_msl_m, label="on board")
        velocity_ax.plot(on_board.t, on_board.velocity_mps, label="on board")
    height_ax.set(title="Height", ylabel="height MSL (m)")
    velocity_ax.set(title="Vertical velocity", xlabel="time (s)", ylabel="velocity up (m/s)")
    height_ax.legend()
    velocity_ax.legend()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log", help="a LOGnnnn directory, or the SD card root to use its newest session")
    parser.add_argument(
        "--set", action="append", default=[], metavar="KEY=VALUE", help="override one setting, e.g. ahrs_gain=0.5"
    )
    parser.add_argument("--cal", type=Path, help="a file of `cal set` lines to apply before replaying")
    args = parser.parse_args()
    overrides = {}
    for item in args.set:
        key, _, value = item.partition("=")
        if key not in FIRMWARE:
            parser.error(f"unknown setting {key}; one of {', '.join(FIRMWARE)}")
        default = FIRMWARE[key]
        overrides[key] = [float(v) for v in value.split(",")] if isinstance(default, list) else type(default)(value)

    log = sdlog.read(args.log)
    if args.cal:
        log = recalibrate.apply(log, args.cal.read_text())
    states = replay(log, **overrides)
    selected = states[states.selected & states.ready]
    print(f"{log.dir}: replayed {log.imu.t.max():.0f} s, {len(selected)} outputs")
    plot(log, states)


if __name__ == "__main__":
    main()
