# SEF-light integration

## Milestones

1. Feed calibrated, timestamped IMU, barometer, GNSS, and magnetometer samples to SEF-light in time order. Verify full-rate delivery and bounded queues on the board.
2. Publish the selected SEF-light height, velocity, and attitude through one state watch. Verify the CAN vertical and orientation messages use that watch and the documented coordinate frames.
3. Tune from measured stationary noise and a known height change. Verify height convergence, sensor failover, and dynamic response on hardware.

## Current measurements

The stationary indoor run in `/private/tmp/asteria-reception-order-run.log` lasted 236 s. Every 10 s queue report showed zero dropped and zero late IMU or aiding events. The two barometric pressure altitudes had standard deviations of about 0.27 m and 0.30 m; a 1.5 m observation standard deviation leaves margin for unmeasured effects. Their raw pressure altitudes were about 305 m and 295 m. SEF-light's bias states reconcile them with GNSS MSL height.

GNSS_0 ranged from 391 m to 428 m MSL indoors. GNSS_1 ranged from 418 m to 426 m while the carrier was stationary. The selected estimate moved from 425 m to 422 m. The receiver's reported `vAcc` did not capture all of this slow indoor drift, so the filter uses a 3 m GNSS height uncertainty floor. This run proves timing and short-term stability; it cannot establish outdoor absolute accuracy.

The stationary gravity-norm residual averaged about 0.053 m/s² for IMU_0 and 0.139 m/s² for IMU_1 after the stored gyro calibration loaded. A stationary gyro calibration does not correct accelerometer scale or offset. Both magnetometers initialized, but neither has a stored magnetic calibration; their identity fallback is excluded from AHRS fusion. Magnetic aiding can be checked after a tumble calibration and reset.

On 23 September, `just run --release` flashed the current firmware through the attached ST-Link. A five-minute stationary run reported no dropped or late samples, with a peak IMU backlog of 27 after startup. One subsequent reset initialized all four I²C sensors, then both barometers and both magnetometers timed out and stopped. Another reset brought all four back. Read tasks now retry once per second after ten consecutive errors and report recovery; the new build ran for two minutes with all four sensors streaming. The retry path has not yet been observed recovering a failed bus.

In the next logged stationary run, GNSS_1 reported 420.92–421.92 m MSL with mean speed accuracy about 0.16 m/s. GNSS_0 ranged from 425.68 m to 481.58 m with median vertical accuracy about 12 m, so only GNSS_1 was selected. The fused MSL height averaged 421.76 m with 0.05 m standard deviation. Its reported vertical velocity averaged +0.08 m/s while the carrier was stationary, within the roughly 0.2–0.3 m/s filter uncertainty. The two barometer pressure-altitude standard deviations were 0.27 m and 0.25 m. An earlier repeat showed GNSS_1 height drifting upward by about 0.063 m/s while its reported vertical velocity averaged about −0.006 m/s; the estimate partly followed the indoor height drift. These data do not justify treating the GNSS height trend as physical motion or tuning against one indoor trajectory alone.

Five stationary logs showed healthy IMU consistency scores near 0.03. Their score differences were only a few thousandths, so the previous 2.0 selector margin prevented quality-based handover. With a 0.003 margin, the board switched from IMU_1 to the lower-scoring IMU_0 once at 5 s and did not switch back during a 155 s run. Over the same stationary interval, the IMU_0 chain's mean reported vertical velocity was +0.038 m/s versus +0.089 m/s for IMU_1. The selected height averaged 419.59 m MSL with 0.31 m standard deviation. All fifteen queue reports had zero dropped and late samples, with a peak IMU backlog of 35. Three additional reset runs had no I²C read faults; actual retry recovery remains unobserved.

A regression replay with a biased but still valid IMU found that the old 250 ms dwell could switch on a brief GNSS innovation and switch back. The final dwell is 1.5 s; invalid IMUs still hand over immediately. In a 113 s board run with this setting, the selector switched once to IMU_0 at 5 s, all eleven queue reports had zero drops and late events, and there were no I²C timeouts or estimator errors. The selected vertical velocity averaged +0.039 m/s. GNSS_1 MSL height wandered from 417.0 m to 421.3 m indoors during this run, so its height standard deviation cannot serve as a physical-motion error measurement.

The pinned Embassy I²C driver's async timeout cancels a DMA transfer without resetting the controller state machine. After ten consecutive read failures, each affected read task now resets its controller under the shared bus lock, then retries after one second. The change passed the host suite and release build. A one-minute `just run --release` board check after flashing showed both barometers and both magnetometers streaming normally. A later startup experiment caused both barometers to reach ten consecutive timeouts; both reported recovery after the controller reset and one-second retry, and their measurements resumed. The startup experiment was reverted because it made I²C readout less reliable.

The GNSS status log now counts NAV-PVT packets and records the largest gap in GPS epoch time over each roughly one-second report. On the attached indoor board, GNSS_0 delivered 20–21 packets per report with 50 ms maximum gaps. GNSS_1 delivered 11–20 with occasional 100 ms epoch gaps despite its 50 ms rate command; after one startup overrun/checksum error, it had no further UART or parser errors. The skipped epochs originate before estimator input, but these logs do not distinguish receiver scheduling from silent loss of complete UART packets. GNSS_1 remained the selected receiver by vertical accuracy, and the estimator queue reported no dropped or late samples.

A 313 s stationary soak on this build produced 31 queue reports with zero drops and late samples. After the first 30 s, selected MSL height averaged 414.21 m with a 0.73 m standard deviation and 413.08–415.33 m range; mean reported vertical velocity was +0.036 m/s. GNSS_1 itself ranged from 410.53 to 416.58 m indoors, so the absolute height change cannot be attributed to board motion. The selected IMU remained IMU_0. The only I²C errors were two barometer timeouts during the first second of startup.

Restoring the original startup order and flashing again gave a 69 s final check with both barometers and magnetometers streaming, six queue reports with zero drops and late samples, and no I²C read errors after two first-second barometer timeouts. GNSS_0 and GNSS_1 each had one startup UART overrun and then initialized.

During the first 22 minutes of the longer stationary soak, both stored gyro calibrations loaded from flash, but their residual angular rates remained nonzero. Across the 20–1,250 s interval, IMU_0 averaged about `[-0.020, +0.051, +0.059]` °/s and IMU_1 about `[+0.040, +0.014, +0.051]` °/s. Their body-to-NED yaw angles changed by about 100° and 70°, respectively, over 22 minutes. All magnetic samples were ignored because neither magnetometer has a valid stored calibration. The observed yaw drift is consistent with the residual gyro rates; this stationary run cannot validate magnetic heading or correct the stored calibration without a new gyro calibration run.

An offline SEF-light replay held both barometer altitudes and both IMUs stationary while the reported GNSS height rose 10 m over 300 s with zero GNSS vertical velocity. The estimate moved 4.51 m from 30 s to 300 s with either zero barometer bias walk or the configured 0.001 m/√s; increasing the walk to 0.01 m/√s moved it 4.72 m. Varying barometer observation standard deviation from 0.3 m to 3 m likewise produced 4.50–4.58 m drift. Neither barometer tuning knob rejects a sustained correlated GNSS height error by itself.

## Selection and output policy

GNSS readout requires the UBX `GPS_FIX_OK` flag. Vertical fusion requires a 3D fix and `vAcc` at most 3 m. Among fresh receivers, lower `vAcc` wins; the selected receiver stays until the other is at least 1.5 times better. SEF-light receives only the chosen receiver, so it does not blend the two heights. `vAcc` measures the height dimension directly. PDOP is logged for diagnosis but does not gate fusion; `hAcc` would be relevant to a horizontal position estimator, which SEF-light does not provide.

Each calibrated magnetometer supplies the attitude chain of the same index when its field magnitude is physically plausible. AHRS magnetic rejection handles angular disagreement, and samples expire after 250 ms. The selected IMU chain supplies both the published vertical state and orientation. The CAN orientation quaternion is the inverse of SEF-light's body-to-NED quaternion. Without magnetic calibration, roll and pitch remain available while yaw may drift.

Orientation is published once the selected IMU attitude is ready. Vertical CAN telemetry waits until a GNSS fix has established the MSL reference, so a raw pressure altitude is never labeled MSL.

## Remaining verification

- Calibrate both magnetometers while tumbling the board, reset, then verify AHRS reports accepted magnetic samples. This requires physical movement.
- Measure a known vertical displacement and return to the start. Check MSL height response and recovery without changing the GNSS selection policy.
- Observe actual CAN frames with a bus peer or analyzer. The firmware build verifies message construction, but no CAN bus was available for this run.
