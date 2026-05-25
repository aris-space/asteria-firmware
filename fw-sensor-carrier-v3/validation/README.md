# sensor-carrier-v3 hardware validation

Standalone bring-up firmware for a freshly assembled sensor-carrier-v3 board. It
exercises every sensor and indicator and reports one pass/fail verdict, to catch
dead parts, swapped buses, and wiring mistakes before the real firmware runs.

This is a best-effort smoke test, not exhaustive: it confirms each part is
present and reading in a rough nominal range. It does not test calibration,
accuracy, or every failure mode, so a pass means "nothing obviously broken,"
not "fully qualified."

## What it checks

- **LEDs + buzzer** (operator confirms by eye/ear)
- **2x IMU** (LSM6DSO32, SPI): WHO_AM_I, an INT1 data-ready edge, accel/gyro/temp
- **2x barometer** (MS5607, I2C): PROM read, then pressure/temp
- **2x magnetometer** (LSM303AGR, I2C): WHO_AM_I, then |B|
- **2x humidity/temp** (SHT4x, I2C): serial number, then RH/temp
- **2x GNSS** (u-blox, UART): link up, valid UBX packets, UBX-NAV-STATUS present
- **Flash** (W25Q256JV, OCTOSPI): manufacturer + device ID
- **SD card** (SDMMC): register-level CMD0 / CMD8 / ACMD41 power-up

One I2C device per bus: bus1 = I2C5, bus2 = I2C4.

## What it does NOT check

- **GNSS antennas / fix:** it never waits for a fix, so a bad antenna passes as long as the receiver talks.
- **Full GNSS config:** only that NAV-STATUS arrives, not the complete message/rate setup.
- **USB-C:** not exercised at all.
- **Reset button:** not exercised.
- **CAN and backplane comms:** not exercised.
- **SD card** is optional: it runs after the core verdict and only drops the verdict to red, so a board with no card still passes the core checks.

## Run

1. Set the board flat on the bench and **leave it alone** for the whole run. The
   IMU check expects ~1 g and near-zero rotation, so touching or tilting it fails
   the range check.
2. Make sure the two GNSS modules are installed on the back of the board.
3. Attach a debug probe and, from this directory, run:
   ```
   cargo run --release
   ```
   This flashes the STM32H723ZG and streams defmt over RTT (see `.cargo/config.toml`).
4. Watch the console log and the board's LEDs/buzzer for the verdict (below).

## Result

- **Console:** per-sensor readings, then `CORE CHECKS PASSED` / `SOME CHECKS FAILED`;
  failures name the sensor and reason.
- **Board:** rising chime + all LEDs lit = pass, low tone + solid red = fail.

## Watch out for

- Magnetometer bounds are wide (raw |B|, hard-iron uncorrected): catches dead/railed, not exact field.
- SD card runs last because embassy's H723 SDMMC driver can hang; it falls back to bounded register-level polling. A missing/unseated card fails CMD8.
- `main` moves the SDMMC kernel clock to PLL2_R (200 MHz); the default 240 MHz hangs init. The real firmware will need the same.

## Features

- **`use-i2c4`** (default on): bus2 (sensor block 2) runs on I2C4. I2C4 can only
  DMA via BDMA, and BDMA reaches SRAM4 only, so transfers are staged through an
  SRAM4 bounce buffer. Disable with `--no-default-features` to fall back to the
  hardware-bridged I2C2 path (no BDMA, no SRAM4).
