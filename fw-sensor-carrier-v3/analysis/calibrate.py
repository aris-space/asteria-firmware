"""Fits fw-sensor-carrier-v3 calibration from one SD logging session and
prints it as `cal set` lines to paste into the board's USB console.

Gyro bias is the mean rate over still stretches, magnetometer hard and soft
iron an ellipsoid fit over all orientations the board was turned through,
both from raw counts. Latencies come from latency.py. `mark still` and
`mark tumble` in the console narrow which stretches are used; without marks,
still stretches are found from the gyro and the magnetometer fit uses every
sample.

    uv run calibrate.py /Volumes/SD [--from S] [--to S] [--plot]
"""

import argparse
from dataclasses import dataclass
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

import latency
import sdlog

# Units and axes as in the firmware's calibration/<kind>.rs.
GYRO_DPS_PER_LSB = 0.07
ACCEL_G_PER_LSB = 1 / 4096
MAG_NT_PER_LSB = 150.0
IMU_HZ = 833.0
# A one-second window counts as still if no axis varies more than this.
STILL_WINDOW_S = 1.0
STILL_MAX_SPREAD_DPS = 0.5
# A board resting in a hand wobbles by about 1 dps, which spoils a gyro bias
# but leaves the measured gravity unchanged.
ACCEL_STILL_MAX_SPREAD_DPS = 2.0
MIN_STILL_S = 5.0
# Plausible accelerometer correction per axis, as in the firmware; anything
# beyond means a bad fit.
MAX_ACCEL_OFFSET_G = 0.2
ACCEL_SCALE_RANGE = (0.9, 1.1)
# Plausible Earth-field magnitude, as in the firmware.
MIN_FIELD_NT = 22_000.0
MAX_FIELD_NT = 67_000.0
# A clean tumble fits to under 1 %; more means disturbances or poor coverage.
MAX_MAG_FIT_ERROR_PERCENT = 3.0
# Both IMUs are the same chip with the same settings and timestamping, so
# they should agree to within one 833 Hz sample.
MAX_IMU_LATENCY_US = 1_000
# Still samples are averaged over this long in the plot so noise does not hide drift.
PLOT_MEAN_S = 0.1


def imu_board_dps(imu) -> np.ndarray:
    """LSM6DSO32 sensor-to-board remap: negate x and z."""
    return imu[["gx_raw", "gy_raw", "gz_raw"]].to_numpy() * [-1, 1, -1] * GYRO_DPS_PER_LSB


def imu_board_g(imu) -> np.ndarray:
    """LSM6DSO32 sensor-to-board remap, as for the gyro."""
    return imu[["ax_raw", "ay_raw", "az_raw"]].to_numpy() * [-1, 1, -1] * ACCEL_G_PER_LSB


def mag_board_counts(mag) -> np.ndarray:
    """LSM303AGR sensor-to-board remap: negate all axes."""
    return -mag[["x_raw", "y_raw", "z_raw"]].to_numpy()


@dataclass
class GyroFit:
    sensor: str
    t: np.ndarray
    rate_dps: np.ndarray
    still: np.ndarray
    bias_dps: np.ndarray | None


@dataclass
class AccelFit:
    sensor: str
    # Mean acceleration of each still window, in board axes and g.
    rest_g: np.ndarray
    offset_g: np.ndarray | None = None
    scale: np.ndarray | None = None
    error_before_percent: float = 0.0
    error_percent: float = 0.0

    def corrected(self) -> np.ndarray:
        """`(board − offset) · scale` per axis, in g."""
        return (self.rest_g - self.offset_g) * self.scale


@dataclass
class MagFit:
    sensor: str
    t: np.ndarray
    board_counts: np.ndarray
    used: np.ndarray
    hard_iron: np.ndarray | None = None
    soft_iron: np.ndarray | None = None
    field_strength: float = 0.0
    error_percent: float = 0.0

    def corrected(self) -> np.ndarray:
        """`soft_iron · (board − hard_iron)`, in counts."""
        return (self.board_counts - self.hard_iron) @ self.soft_iron.T


def intervals(log, label: str) -> list[tuple[float, float]]:
    """Time ranges that start at a `label` mark and end at the next mark."""
    t = log.marks.t.to_numpy() if len(log.marks) else np.array([])
    labels = log.marks.label.to_numpy() if len(log.marks) else np.array([])
    ends = np.append(t[1:], np.inf)
    return [(start, end) for start, end, l in zip(t, ends, labels) if l == label]


def within(t: np.ndarray, ranges) -> np.ndarray:
    return np.any([(t >= start) & (t < end) for start, end in ranges], axis=0) if ranges else np.zeros(len(t), bool)


def auto_still(t: np.ndarray, rate: np.ndarray, max_spread_dps: float = STILL_MAX_SPREAD_DPS) -> np.ndarray:
    """Samples in one-second windows where the board did not rotate."""
    window = np.floor(t / STILL_WINDOW_S)
    still = np.zeros(len(t), bool)
    for w in np.unique(window):
        inside = window == w
        still[inside] = np.all(rate[inside].std(axis=0) < max_spread_dps)
    return still


def fit_gyro(log, imu: int) -> GyroFit:
    samples = log.sensor("imu", imu)
    t, rate = samples.t.to_numpy(), imu_board_dps(samples)
    ranges = intervals(log, "still")
    still = within(t, ranges) if ranges else auto_still(t, rate)
    bias = rate[still].mean(axis=0) if still.sum() / IMU_HZ >= MIN_STILL_S else None
    return GyroFit(f"IMU_{imu}", t, rate, still, bias)


def fit_accel(log, imu: int) -> AccelFit:
    samples = log.sensor("imu", imu)
    t, accel = samples.t.to_numpy(), imu_board_g(samples)
    still = auto_still(t, imu_board_dps(samples), ACCEL_STILL_MAX_SPREAD_DPS)
    window = np.floor(t / STILL_WINDOW_S)
    rest = np.array([accel[still & (window == w)].mean(axis=0) for w in np.unique(window[still])]).reshape(-1, 3)
    fit = AccelFit(f"IMU_{imu}", rest)
    # Every axis must have pointed both up and down, or its offset and scale
    # are not determined.
    if len(rest) == 0 or np.any(rest.max(axis=0) < 0.5) or np.any(rest.min(axis=0) > -0.5):
        return fit
    # Axis-aligned ellipsoid a·x² + b·y² + c·z² + 2d·x + 2e·y + 2f·z = 1.
    coefficients = np.linalg.lstsq(np.column_stack([rest**2, 2 * rest]), np.ones(len(rest)), rcond=None)[0]
    squares, linear = coefficients[:3], coefficients[3:]
    if np.any(squares <= 0):
        return fit
    offset = -linear / squares
    radius = np.sqrt((1 + np.sum(squares * offset**2)) / squares)
    scale = 1 / radius
    if np.any(np.abs(offset) > MAX_ACCEL_OFFSET_G) or np.any(
        (scale < ACCEL_SCALE_RANGE[0]) | (scale > ACCEL_SCALE_RANGE[1])
    ):
        return fit
    fit.offset_g, fit.scale = offset, scale
    fit.error_before_percent = magnitude_error_percent(rest)
    fit.error_percent = magnitude_error_percent(fit.corrected())
    # Never apply a correction that does not improve the data it was fitted on.
    if fit.error_percent >= fit.error_before_percent:
        fit.offset_g = fit.scale = None
    return fit


def magnitude_error_percent(accel_g: np.ndarray) -> float:
    """RMS deviation of |a| from 1 g, in percent."""
    return float(np.sqrt(np.mean((np.linalg.norm(accel_g, axis=1) - 1) ** 2)) * 100)


def fit_mag(log, mag: int) -> MagFit:
    samples = log.sensor("mag", mag)
    t, board = samples.t.to_numpy(), mag_board_counts(samples).astype(float)
    ranges = intervals(log, "tumble")
    used = within(t, ranges) if ranges else np.ones(len(t), bool)
    fit = MagFit(f"MAG_BUS_{mag + 1}", t, board, used)
    if used.sum() < 10:
        return fit
    # Ellipsoid (x − c)ᵀ M (x − c) = 1 from the quadric a·x² + … + 2i·z = 1,
    # fitted on scaled counts for conditioning.
    scale = np.median(np.linalg.norm(board[used], axis=1))
    x, y, z = (board[used] / scale).T
    design = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z])
    a, b, c, d, e, f, g, h, i = np.linalg.lstsq(design, np.ones(len(x)), rcond=None)[0]
    quadric = np.array([[a, d, e], [d, b, f], [e, f, c]])
    center = -np.linalg.solve(quadric, [g, h, i])
    shape = quadric / (1 + center @ quadric @ center) / scale**2
    eigenvalues, eigenvectors = np.linalg.eigh(shape)
    if np.any(eigenvalues <= 0):
        return fit
    # soft_iron maps the ellipsoid onto a sphere of the same volume.
    field = float(np.prod(eigenvalues) ** (-1 / 6))
    fit.hard_iron = center * scale
    fit.soft_iron = eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T * field
    fit.field_strength = field
    magnitudes = np.linalg.norm(fit.corrected()[used], axis=1)
    fit.error_percent = float(np.sqrt(np.mean((magnitudes / field - 1) ** 2)) * 100)
    if not MIN_FIELD_NT <= field * MAG_NT_PER_LSB <= MAX_FIELD_NT or fit.error_percent > MAX_MAG_FIT_ERROR_PERCENT:
        fit.hard_iron = None
    return fit


def floats(values, digits: int) -> str:
    return ",".join(f"{v:.{digits}f}" for v in np.ravel(values))


def print_lines(name: str, gyros, accels, mags, estimates) -> None:
    latencies = {"IMU_0": 0.0} | {e.name: e.latency_s for e in estimates if e.observable}
    latency_us = {sensor: round(s * 1e6) for sensor, s in latencies.items()}
    print("Paste into the console, then `reset` and `cal show`:")
    for fit, accel in zip(gyros, accels):
        us = latency_us.get(fit.sensor)
        if fit.bias_dps is None:
            print(f"# {fit.sensor}: only {fit.still.sum() / IMU_HZ:.1f} s still, need {MIN_STILL_S:.0f} s; no line")
        elif us is None:
            print(f"# {fit.sensor}: latency not observable; no line")
        else:
            if abs(us) > MAX_IMU_LATENCY_US:
                print(
                    f"# {fit.sensor}: WARNING IMU timestamps disagree by {us / 1000:.1f} ms; check the readout before using this"
                )
            if accel.offset_g is None:
                print(f"# {fit.sensor}: accelerometer not fitted or not improved (rest it on all six sides); identity")
                offset, scale = np.zeros(3), np.ones(3)
            else:
                print(
                    f"# {fit.sensor}: accelerometer |a| error {accel.error_before_percent:.2f} % -> "
                    f"{accel.error_percent:.2f} % over {len(accel.rest_g)} still s"
                )
                offset, scale = accel.offset_g, accel.scale
            print(
                f"cal set {fit.sensor} name={name} latency_us={us} gyro_bias_dps={floats(fit.bias_dps, 4)}"
                f" accel_offset_g={floats(offset, 4)} accel_scale={floats(scale, 5)}"
            )
    for fit in mags:
        us = latency_us.get(fit.sensor)
        if fit.hard_iron is None:
            print(
                f"# {fit.sensor}: no good fit from {fit.used.sum()} samples (error {fit.error_percent:.1f} %; "
                "tumble slowly through all orientations, away from metal); no line"
            )
        elif us is None:
            print(f"# {fit.sensor}: latency not observable; no line")
        else:
            print(
                f"# {fit.sensor}: field {fit.field_strength * MAG_NT_PER_LSB / 1000:.1f} uT, error {fit.error_percent:.2f} %, {fit.used.sum()} samples"
            )
            print(
                f"cal set {fit.sensor} name={name} latency_us={us} hard_nt={floats(fit.hard_iron * MAG_NT_PER_LSB, 1)} soft={floats(fit.soft_iron, 5)}"
            )
    for sensor in ["BARO_BUS_1", "BARO_BUS_2", "GNSS_0", "GNSS_1"]:
        if sensor in latency_us:
            print(f"cal set {sensor} name={name} latency_us={latency_us[sensor]}")
        else:
            print(f"# {sensor}: latency not observable; no line")


def plot(gyros, accels, mags, estimates) -> None:
    _, axes = plt.subplots(len(gyros), 1, sharex=True, num="Gyro while still")
    for ax, fit in zip(axes, gyros):
        t, rate = fit.t[fit.still], fit.rate_dps[fit.still]
        window = np.floor(t / PLOT_MEAN_S)
        starts = np.flatnonzero(np.diff(window, prepend=-1))
        means_t = np.add.reduceat(t, starts) / np.diff(np.append(starts, len(t)))
        for axis, name in enumerate("xyz"):
            means = np.add.reduceat(rate[:, axis], starts) / np.diff(np.append(starts, len(t)))
            dots = ax.plot(means_t, means, ".", label=name)[0]
            if fit.bias_dps is not None:
                ax.axhline(fit.bias_dps[axis], color=dots.get_color())
        ax.set(
            title=f"{fit.sensor} rate while still ({PLOT_MEAN_S} s means), lines at the fitted bias",
            ylabel="rate (dps)",
        )
        ax.legend()
    axes[-1].set_xlabel("time (s)")

    _, axes = plt.subplots(len(accels), 1, num="Accelerometer at rest")
    for ax, fit in zip(axes, accels):
        ax.plot(np.linalg.norm(fit.rest_g, axis=1), ".", label="uncorrected")
        if fit.offset_g is not None:
            ax.plot(np.linalg.norm(fit.corrected(), axis=1), ".", label="corrected")
        ax.axhline(1.0, color="gray")
        ax.set(title=f"{fit.sensor} |a| per still second, line at 1 g", xlabel="still second", ylabel="|a| (g)")
        ax.legend()

    _, axes = plt.subplots(len(mags), 1, sharex=True, num="Magnetometer field magnitude")
    for ax, fit in zip(axes, mags):
        to_ut = MAG_NT_PER_LSB / 1000
        ax.plot(fit.t, np.linalg.norm(fit.board_counts, axis=1) * to_ut, label="uncorrected")
        if fit.hard_iron is not None:
            ax.plot(fit.t, np.linalg.norm(fit.corrected(), axis=1) * to_ut, label="corrected")
        ax.set(title=f"{fit.sensor} field magnitude", ylabel="field (uT)")
        ax.legend()
    axes[-1].set_xlabel("time (s)")

    if estimates:
        _, axes = plt.subplots(len(estimates), 1, num="Latency fits", squeeze=False)
        for ax, e in zip(axes[:, 0], estimates):
            ax.plot(e.profile_latency_s * 1e3, e.profile_increase, "o-", label="fit")
            ax.plot(e.profile_latency_s * 1e3, e.profile_predicted, label="predicted")
            ax.set(title=f"{e.name} latency fit", xlabel="latency (ms)", ylabel="χ² increase")
            ax.legend()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log", help="a LOGnnnn directory, or the SD card root to use its newest session")
    parser.add_argument("--from", dest="start", type=float, default=0.0, help="ignore samples before this many seconds")
    parser.add_argument("--to", dest="end", type=float, default=np.inf, help="ignore samples after this many seconds")
    parser.add_argument("--plot", action="store_true", help="show plots of the fits")
    args = parser.parse_args()

    log = sdlog.read(args.log)
    duration = log.imu.t.max()
    print(f"{log.dir}: {duration:.0f} s")
    for kind in ["imu", "mag", "baro", "gnss"]:
        table = getattr(log, kind)
        for index in sorted(table[kind].unique()) if len(table) else []:
            count = (table[kind] == index).sum()
            print(f"  {kind.upper()} {int(index)}  {count:>8} samples  {count / duration:7.1f} Hz")
    if len(log.drops):
        dropped = int(log.drops.drop(columns=["uptime_us", "t"], errors="ignore").to_numpy().sum())
        if dropped:
            print(f"  SD writer dropped {dropped} readings (see DROPS.CSV)")
    print()

    log.crop(args.start, args.end)
    estimates = latency.fit_all(log)
    gyros = [fit_gyro(log, i) for i in range(2)]
    accels = [fit_accel(log, i) for i in range(2)]
    mags = [fit_mag(log, i) for i in range(2)]
    # When the fit was made; exactly the firmware's 16-character name limit.
    print_lines(datetime.now().strftime("%Y-%m-%dT%H:%M"), gyros, accels, mags, estimates)
    if args.plot:
        plot(gyros, accels, mags, estimates)


if __name__ == "__main__":
    main()
