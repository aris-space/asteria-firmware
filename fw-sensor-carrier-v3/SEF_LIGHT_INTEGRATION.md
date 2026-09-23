# SEF-light integration

## Milestones

1. Feed calibrated, timestamped IMU, barometer, GNSS, and magnetometer samples to SEF-light in time order. Verify full-rate delivery and bounded queues on the board.
2. Publish the selected SEF-light height, velocity, and attitude through one state watch. Verify the CAN vertical and orientation messages use that watch and the documented coordinate frames.
3. Tune from measured stationary noise and a known height change. Verify height convergence, sensor failover, and dynamic response on hardware.

## Current measurements

The stationary indoor run in `/private/tmp/asteria-reception-order-run.log` lasted 236 s. Every 10 s queue report showed zero dropped and zero late IMU or aiding events. The two barometric pressure altitudes had standard deviations of about 0.27 m and 0.30 m; a 1.5 m observation standard deviation leaves margin for unmeasured effects. Their raw pressure altitudes were about 305 m and 295 m. SEF-light's bias states reconcile them with GNSS MSL height.

GNSS_0 ranged from 391 m to 428 m MSL indoors. GNSS_1 ranged from 418 m to 426 m while the carrier was stationary. The selected estimate moved from 425 m to 422 m. The receiver's reported `vAcc` did not capture all of this slow indoor drift, so the filter uses a 3 m GNSS height uncertainty floor. This run proves timing and short-term stability; it cannot establish outdoor absolute accuracy.

The stationary gravity-norm residual averaged about 0.053 m/s² for IMU_0 and 0.139 m/s² for IMU_1 after the stored gyro calibration loaded. A stationary gyro calibration does not correct accelerometer scale or offset. Both magnetometers initialized, but neither has a stored magnetic calibration; their identity fallback is excluded from AHRS fusion. Magnetic aiding can be checked after a tumble calibration and reset.

## Selection and output policy

GNSS readout requires the UBX `GPS_FIX_OK` flag. Vertical fusion requires a 3D fix and `vAcc` at most 3 m. Among fresh receivers, lower `vAcc` wins; the selected receiver stays until the other is at least 1.5 times better. SEF-light receives only the chosen receiver, so it does not blend the two heights. `vAcc` measures the height dimension directly. PDOP describes overall satellite geometry and remains an additional SEF validity check; `hAcc` would be relevant to a horizontal position estimator, which SEF-light does not provide.

Each calibrated magnetometer supplies the attitude chain of the same index when its field magnitude is physically plausible. AHRS magnetic rejection handles angular disagreement, and samples expire after 250 ms. The selected IMU chain supplies both the published vertical state and orientation. The CAN orientation quaternion is the inverse of SEF-light's body-to-NED quaternion. Without magnetic calibration, roll and pitch remain available while yaw may drift.

Orientation is published once the selected IMU attitude is ready. Vertical CAN telemetry waits until a GNSS fix has established the MSL reference, so a raw pressure altitude is never labeled MSL.

## Remaining verification

- Flash and observe the new build with `just run --release`. The latest attempt failed because no debug probe was detected; the firmware build and host tests passed.
- Calibrate both magnetometers while tumbling the board, reset, then verify AHRS reports accepted magnetic samples. This requires physical movement.
- Measure a known vertical displacement and return to the start. Check MSL height response and recovery without changing the GNSS selection policy.
- Observe actual CAN frames with a bus peer or analyzer. The firmware build verifies message construction, but no CAN bus was available for this run.
