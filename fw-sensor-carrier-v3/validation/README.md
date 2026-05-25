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
- **SD card** (SDMMC): initialise the card, then read back block 0

## What it does NOT check

- **GNSS antennas / fix:** it never waits for a fix, so a bad antenna passes as long as the receiver talks.
- **Full GNSS config:** only that NAV-STATUS arrives, not the complete message/rate setup.
- **USB-C:** not exercised at all.
- **Reset button:** not exercised.
- **CAN and backplane comms:** not exercised.
- **SD write:** it initialises the card and reads block 0, but never writes, so it does not prove the card is writable. It runs after the core verdict, so the printed result excludes it; a clean failure still turns the board LED red, though a wholly unresponsive card can stall this last step (the core verdict is already shown by then).
- **Card detect (PD3):** the detect line is logged but not yet asserted on, so its polarity is unverified and a missing card is only caught by init failing.

## Run

1. Make sure the two GNSS modules are installed on the back of the board.
2. Power the board (via usb-c, the backplane or a backplane-adapter)
3. Set the board flat on the bench and **leave it alone** for the whole run. The
   IMU check expects ~1 g and near-zero rotation, so touching or tilting it fails
   the range check.
4. Attach a debug probe and, from this directory, run:
   ```
   cargo run --release
   ```
   This flashes the STM32H723ZG and streams defmt over RTT (see `.cargo/config.toml`).
5. Watch for the verdict:
   - **Console:** per-sensor readings, then `CORE CHECKS PASSED` / `SOME CHECKS FAILED`; failures name the sensor and reason.
   - **Board:** rising chime + all LEDs lit = pass, low tone + solid red = fail.

One thing to note: the magnetometer bounds are wide (raw |B|, hard-iron
uncorrected), so they catch a dead or railed part, not the exact field. If |B| is
still out of range, there is likely a hard-iron bias and the board will definitely
need calibration.

## Features

- **`use-i2c4`** (default off): by default bus2 (sensor block 2) uses the
  hardware-bridged I2C2 path (no BDMA, no SRAM4). Enable with `--features use-i2c4`
  to instead run bus2 on I2C4, which can only DMA via BDMA (SRAM4 only), so
  transfers are staged through an SRAM4 bounce buffer. Boards with the red bridge
  installed, or v4 boards, should stay on the default I2C2 path.
