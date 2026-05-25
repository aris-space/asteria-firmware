# sensor-carrier-v3 hardware validation

Standalone bring-up firmware for a freshly assembled sensor-carrier-v3 board. It
exercises every sensor and indicator and reports one pass/fail verdict, to catch
dead parts, swapped buses, and wiring mistakes before the real firmware runs.

This is a best-effort smoke test, not exhaustive: it confirms each part responds
and reads without error, and prints the readings for the operator to eyeball. It
does not range-check the values or test calibration, accuracy, or every failure
mode, so a pass means "nothing obviously broken," not "fully qualified."

## What it checks

- **LEDs + buzzer** (operator confirms by eye/ear)
- **2x IMU** (LSM6DSO32, SPI): WHO_AM_I, an INT1 data-ready edge, accel/gyro/temp
- **2x barometer** (MS5607, I2C): PROM read, then pressure/temp
- **2x magnetometer** (LSM303AGR, I2C): WHO_AM_I, then |B|
- **2x humidity/temp** (SHT4x, I2C): serial number, then RH/temp
- **2x GNSS** (u-blox, UART): link up, valid UBX packets, UBX-NAV-STATUS present
- **Flash** (W25Q256JV, OCTOSPI): manufacturer + device ID
- **SD card** (SDMMC + FAT): initialise the card, then write HELLO_WORLD.txt to the first FAT partition and read it back

## What it does NOT check

- **GNSS antennas / fix:** it never waits for a fix, so a bad antenna passes as long as the receiver talks.
- **Full GNSS config:** only that NAV-STATUS arrives, not the complete message/rate setup.
- **USB-C (data path):** the port is brought up as a CDC-ACM serial logger that mirrors the console output (see Run), so enumeration is exercised, but no further USB function is tested.
- **Reset button:** not exercised.
- **CAN and backplane comms:** not exercised.
- **Card detect (PD3):** the detect line is logged but not yet asserted on, so its polarity is unverified and a missing card is only caught by init failing.
- **SD card formatting:** the card must already hold a FAT16/FAT32 filesystem with an MBR partition table; the check writes a file but does not format the card.

## Run

1. Make sure the two GNSS modules are installed on the back of the board.
2. Power the board (via usb-c, the backplane or a backplane-adapter)
3. Set the board flat on the bench and **leave it alone** for the whole run. The
   readings aren't range-checked, but a still board prints clean ~1 g, near-zero
   rotation values that are easy to eyeball.
4. Attach a debug probe and, from this directory, run:
   ```
   cargo run --release
   ```
   This flashes the STM32H723ZG and streams defmt over RTT (see `.cargo/config.toml`).
5. Watch for the verdict:
   - **Console:** per-sensor readings, then `CORE CHECKS PASSED` / `SOME CHECKS FAILED`; failures name the sensor and reason.
   - **USB-C:** the same lines are mirrored over the USB-C port as a CDC-ACM serial device, so a probe is not required to read the result. It enumerates as "Asteria Sensor Carrier v3 Validation" (serial `scv3val`), so on macOS the port is `/dev/tty.usbmodemscv3val1`; connect with e.g. `screen /dev/tty.usbmodemscv3val1 115200` (the baud rate is ignored). Early lines are buffered, so the full run still appears if you connect a little late.
   - **Board:** rising chime + all LEDs lit = pass, low tone + solid red = fail.

One thing to note: the magnetometer prints raw |B| (hard-iron uncorrected), which
can sit well off the ~48 uT Earth field. That is expected and not a failure here;
the board still needs calibration before the field reading means anything.

## Features

- **`use-i2c4`** (default off): by default bus2 (sensor block 2) uses the
  hardware-bridged I2C2 path (no BDMA, no SRAM4). Enable with `--features use-i2c4`
  to instead run bus2 on I2C4, which can only DMA via BDMA (SRAM4 only), so
  transfers are staged through an SRAM4 bounce buffer. Boards with the red bridge
  installed, or v4 boards, should stay on the default I2C2 path.
