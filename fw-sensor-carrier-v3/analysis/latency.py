"""Per-sensor latencies relative to IMU_0's timestamps.

A sample stamped `t` by a sensor with latency `τ` measures the signal at
`t − τ`. Each fit models the motion as a cubic B-spline and compares sensors
that see the same motion: IMU_1 against IMU_0's rotation rate, each
magnetometer against IMU_0's gyro, and the barometers and GNSS receivers
against IMU_0's vertical acceleration.

Every measurement is linear in the spline coefficients and biases, and only
the latencies enter non-linearly. For given latencies the linear unknowns are
solved exactly (one sparse solve); a trust-region least-squares solver fits
the latencies on what remains.
"""

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp
from scipy.optimize import least_squares
from scipy.sparse.linalg import spsolve
from scipy.spatial.transform import Rotation, Slerp

from sdlog import STANDARD_GRAVITY, Log, pressure_altitude_m, usable_fix

# A latency whose standard deviation exceeds this is not determined by the log.
OBSERVABLE_SIGMA_S = 0.02
# The covariance describes the fit only near its minimum. Without real motion
# that minimum is a ripple in the noise, and moving the latency changes the
# fit far less than the covariance predicts. A latency counts as determined
# only if the fit at shifts of at least TEST_MIN_SHIFT_S worsens by at least a
# quarter of the predicted amount, and clearly (five standard deviations).
PROFILE_SHIFTS_S = np.array([-0.1, -0.05, -0.02, -0.01, -0.005, 0.0, 0.005, 0.01, 0.02, 0.05, 0.1])
TEST_MIN_SHIFT_S = 0.05
MIN_PREDICTED_FRACTION = 0.25
MIN_CHI2_INCREASE = 25.0
# Latency-shifted times must stay inside the spline.
MARGIN_S = 0.5


class Spline:
    """Uniform cubic B-spline with knots every `dt` seconds."""

    def __init__(self, start: float, end: float, dt: float):
        self.t0 = start - MARGIN_S
        self.dt = dt
        self.count = int(np.ceil((end + MARGIN_S - self.t0) / dt)) + 3

    def basis(self, t, derivative: int = 0):
        """Columns and weights `(n, 4)` of the four non-zero basis functions."""
        u = np.clip((np.asarray(t) - self.t0) / self.dt, 0.0, self.count - 3 - 1e-9)
        first = np.floor(u).astype(int)
        s = u - first
        if derivative == 0:
            w = [(1 - s) ** 3 / 6, (3 * s**3 - 6 * s**2 + 4) / 6, (-3 * s**3 + 3 * s**2 + 3 * s + 1) / 6, s**3 / 6]
        elif derivative == 1:
            w = [-((1 - s) ** 2) / 2, (3 * s**2 - 4 * s) / 2, (-3 * s**2 + 2 * s + 1) / 2, s**2 / 2]
        else:
            w = [1 - s, 3 * s - 2, -3 * s + 1, s]
        return first[:, None] + np.arange(4), np.stack(w, axis=1) / self.dt**derivative


@dataclass
class Rows:
    """Measurements `Σ vals·x[cols] = rhs ± sigma`, one per row of `cols`."""

    cols: np.ndarray
    vals: np.ndarray
    rhs: np.ndarray
    sigma: np.ndarray | float


@dataclass
class Model:
    names: list[str]
    unknowns: int
    rows: Callable[[np.ndarray], list[Rows]]


@dataclass
class Estimate:
    name: str
    latency_s: float
    sigma_s: float
    observable: bool
    # The fit at latencies around the estimate: increase of the normalized χ²
    # over the best fit, and the increase the covariance predicts.
    profile_latency_s: np.ndarray
    profile_increase: np.ndarray
    profile_predicted: np.ndarray


def axis_block(spline: Spline, t, derivative: int, axis: int, scale=1.0):
    """Spline columns and weights for one axis of a three-axis signal."""
    cols, vals = spline.basis(t, derivative)
    return axis * spline.count + cols, vals * np.reshape(scale, (-1, 1))


def column(index: int, scale, n: int):
    """One extra column, such as a bias, for `n` rows."""
    return np.full((n, 1), index), np.broadcast_to(np.reshape(scale, (-1, 1)), (n, 1))


def joined(*blocks) -> tuple[np.ndarray, np.ndarray]:
    return np.hstack([c for c, _ in blocks]), np.hstack([v for _, v in blocks])


def decimate(table, n: int, columns: list[str]):
    """Means of consecutive groups of `n` samples of `t` and `columns`."""
    data = table[["t", *columns]].to_numpy()
    groups = len(data) // n
    return data[: groups * n].reshape(groups, n, -1).mean(axis=1)


def noise_sigma(values, floor: float) -> float:
    """Robust noise standard deviation of a slowly varying signal, from the
    median absolute difference between successive samples."""
    diffs = np.abs(np.diff(np.asarray(values)))
    return max(float(np.median(diffs)) / (0.6745 * np.sqrt(2)), floor) if len(diffs) else floor


GYRO = ["gx_dps", "gy_dps", "gz_dps"]
ACCEL = ["ax_g", "ay_g", "az_g"]
FIELD = ["x_nt", "y_nt", "z_nt"]


def imu_twin(log: Log) -> Model | None:
    """IMU_1 against IMU_0: both see the same board rotation."""
    reference = decimate(log.sensor("imu", 0), 4, GYRO)
    other = decimate(log.sensor("imu", 1), 4, GYRO)
    if len(reference) < 10 or len(other) < 10:
        return None
    spline = Spline(reference[:, 0].min(), reference[:, 0].max(), 0.05)
    sigma = [[noise_sigma(s[:, 1 + a], 1e-3) for a in range(3)] for s in (reference, other)]
    bias = 3 * spline.count

    def rows(tau):
        blocks = []
        for a in range(3):
            blocks.append(Rows(*axis_block(spline, reference[:, 0], 0, a), reference[:, 1 + a], sigma[0][a]))
            shifted = axis_block(spline, other[:, 0] - tau[0], 0, a)
            blocks.append(Rows(*joined(shifted, column(bias + a, 1.0, len(other))), other[:, 1 + a], sigma[1][a]))
        return blocks

    return Model(["IMU_1"], 3 * spline.count + 3, rows)


def mag_rotation(log: Log, index: int) -> Model | None:
    """One magnetometer against IMU_0's gyro. In the board frame the field
    turns opposite to the board, `ṁ = −ω × (m − h)`, with `h` a residual
    hard-iron offset that the stored calibration did not remove."""
    mag = log.sensor("mag", index)
    if len(mag) < 10:
        return None
    t, field = mag.t.to_numpy(), mag[FIELD].to_numpy()
    gyro = decimate(log.sensor("imu", 0), 16, GYRO)
    gyro = gyro[(gyro[:, 0] >= t.min()) & (gyro[:, 0] <= t.max())]
    if len(gyro) < 10:
        return None
    w = np.radians(gyro[:, 1:])
    spline = Spline(t.min(), t.max(), 0.2)
    offset = 3 * spline.count
    # Gyro noise turns into field-rate noise through |m|.
    field_strength = np.median(np.linalg.norm(field, axis=1))
    rate_sigma = max(max(noise_sigma(w[:, a], 1e-5) for a in range(3)) * field_strength, 1.0)
    field_sigma = [noise_sigma(field[:, a], 1.0) for a in range(3)]
    n = len(gyro)

    def rows(tau):
        blocks = []
        for axis in range(3):
            # (ω × v)_axis = ω[a]·v[b] − ω[b]·v[a] for the axis's cyclic pair (a, b).
            a, b = (axis + 1) % 3, (axis + 2) % 3
            entries = joined(
                axis_block(spline, gyro[:, 0], 1, axis),
                axis_block(spline, gyro[:, 0], 0, b, w[:, a]),
                axis_block(spline, gyro[:, 0], 0, a, -w[:, b]),
                column(offset + b, -w[:, a], n),
                column(offset + a, w[:, b], n),
            )
            blocks.append(Rows(*entries, np.zeros(n), rate_sigma))
            blocks.append(Rows(*axis_block(spline, t - tau[0], 0, axis), field[:, axis], field_sigma[axis]))
        return blocks

    return Model([f"MAG_BUS_{index + 1}"], 3 * spline.count + 3, rows)


def vertical(log: Log) -> Model | None:
    """Barometers and GNSS against IMU_0's vertical acceleration, all on one
    height trajectory. Each barometer has a constant offset from MSL; without
    GNSS nothing fixes absolute height, so the first barometer defines it."""
    attitude = log.state[log.state.imu == 0]
    accel = decimate(log.sensor("imu", 0), 8, ACCEL)
    if len(attitude) < 2 or len(accel) < 10:
        return None
    accel = accel[(accel[:, 0] >= attitude.t.iloc[0]) & (accel[:, 0] <= attitude.t.iloc[-1])]
    quaternions = attitude[["qx", "qy", "qz", "qw"]].to_numpy()
    body_to_ned = Slerp(attitude.t.to_numpy(), Rotation.from_quat(quaternions))(accel[:, 0])
    a_up = -(body_to_ned.apply(accel[:, 1:] * STANDARD_GRAVITY)[:, 2] + STANDARD_GRAVITY)
    accel_sigma = noise_sigma(a_up, 1e-3)

    baros = []
    for i in range(2):
        baro = log.sensor("baro", i)
        if len(baro):
            altitude = pressure_altitude_m(baro.pressure_mbar)
            baros.append((i, baro.t.to_numpy(), altitude, noise_sigma(altitude, 0.01)))
    gnsss = []
    for i in range(2):
        gnss = log.sensor("gnss", i)
        gnss = gnss[usable_fix(gnss)]
        if len(gnss):
            gnsss.append((i, gnss))
    if not baros:
        return None

    spline = Spline(accel[:, 0].min(), accel[:, 0].max(), 0.1)
    accel_bias = spline.count
    first_free = 0 if gnsss else 1

    def rows(tau):
        cols, vals = spline.basis(accel[:, 0], 2)
        blocks = [Rows(*joined((cols, vals), column(accel_bias, 1.0, len(accel))), a_up, accel_sigma)]
        for slot, (_, t, altitude, sigma) in enumerate(baros):
            entries = spline.basis(t - tau[slot])
            if slot >= first_free:
                entries = joined(entries, column(spline.count + 1 + slot - first_free, 1.0, len(t)))
            blocks.append(Rows(*entries, altitude, sigma))
        for slot, (_, gnss) in enumerate(gnsss):
            t = gnss.t.to_numpy() - tau[len(baros) + slot]
            height_sigma = np.maximum(gnss.vertical_accuracy_mm.to_numpy() / 1000, 0.05)
            velocity_sigma = np.maximum(gnss.speed_accuracy_mps.to_numpy(), 0.05)
            blocks.append(Rows(*spline.basis(t), gnss.height_msl_m.to_numpy(), height_sigma))
            blocks.append(Rows(*spline.basis(t, 1), -gnss.velocity_down_mps.to_numpy(), velocity_sigma))
        return blocks

    names = [f"BARO_BUS_{i + 1}" for i, *_ in baros] + [f"GNSS_{i + 1}" for i, _ in gnsss]
    return Model(names, spline.count + 1 + len(baros) - first_free, rows)


def residuals(model: Model, latencies) -> np.ndarray:
    """Whitened residuals after solving the linear unknowns exactly."""
    blocks = model.rows(np.asarray(latencies))
    offsets = np.cumsum([0] + [len(b.rhs) for b in blocks])
    rows = np.concatenate([np.repeat(o + np.arange(len(b.rhs)), b.cols.shape[1]) for b, o in zip(blocks, offsets)])
    weights = [np.broadcast_to(1.0 / np.asarray(b.sigma, dtype=float), b.rhs.shape) for b in blocks]
    vals = np.concatenate([(b.vals * w[:, None]).ravel() for b, w in zip(blocks, weights)])
    cols = np.concatenate([b.cols.ravel() for b in blocks])
    rhs = np.concatenate([b.rhs * w for b, w in zip(blocks, weights)])
    a = sp.csr_matrix((vals, (rows, cols)), shape=(len(rhs), model.unknowns))
    # A small ridge keeps coefficients without data well defined.
    normal = (a.T @ a + 1e-9 * sp.identity(model.unknowns)).tocsc()
    return a @ spsolve(normal, a.T @ rhs) - rhs


def fit(model: Model) -> tuple[list[Estimate], int, float]:
    """The latency estimates, the number of rows and the normalized residual RMS."""
    # Finite-difference steps of 10 µs: the residuals are smooth at that scale.
    result = least_squares(lambda tau: residuals(model, tau), np.zeros(len(model.names)), diff_step=1e-5)
    chi2 = float(result.fun @ result.fun)
    rows = len(result.fun)
    scale = chi2 / max(rows - model.unknowns - len(model.names), 1)
    try:
        sigmas = np.sqrt(np.diag(np.linalg.inv(result.jac.T @ result.jac)) * scale)
    except np.linalg.LinAlgError:
        sigmas = np.full(len(model.names), np.inf)

    estimates = []
    for i, name in enumerate(model.names):
        increase = []
        for shift in PROFILE_SHIFTS_S:
            shifted = result.x.copy()
            shifted[i] += shift
            r = residuals(model, shifted)
            increase.append((r @ r - chi2) / scale)
        increase = np.array(increase)
        predicted = (PROFILE_SHIFTS_S / sigmas[i]) ** 2
        tested = np.abs(PROFILE_SHIFTS_S) >= TEST_MIN_SHIFT_S
        determined = np.isfinite(sigmas[i]) and bool(
            np.all(increase[tested] > np.maximum(MIN_CHI2_INCREASE, MIN_PREDICTED_FRACTION * predicted[tested]))
        )
        estimates.append(
            Estimate(
                name=name,
                latency_s=float(result.x[i]),
                sigma_s=float(sigmas[i]),
                observable=determined and sigmas[i] < OBSERVABLE_SIGMA_S,
                profile_latency_s=result.x[i] + PROFILE_SHIFTS_S,
                profile_increase=increase,
                profile_predicted=predicted,
            )
        )
    return estimates, rows, float(np.sqrt(chi2 / max(rows, 1)))


def fit_all(log: Log) -> list[Estimate]:
    """Fits every latency the log has data for and prints each fit."""
    estimates = []
    for title, model in [
        ("IMU_1 vs IMU_0 rotation", imu_twin(log)),
        ("MAG_BUS_1 vs gyro", mag_rotation(log, 0)),
        ("MAG_BUS_2 vs gyro", mag_rotation(log, 1)),
        ("Baro and GNSS vs vertical acceleration", vertical(log)),
    ]:
        print(title)
        if model is None:
            print("  not enough data\n")
            continue
        found, rows, rms = fit(model)
        for e in found:
            if e.observable:
                print(f"  {e.name:<11} {e.latency_s * 1e3:+8.2f} ms ± {e.sigma_s * 1e3:.2f}")
            else:
                print(f"  {e.name:<11} not observable (too little motion)")
        print(f"  {rows} rows, normalized residual RMS {rms:.2f}\n")
        estimates += found
    return estimates
