# Sensor Board hardware validation

This piece of firmware is used to validate the hardware of a fully assembled sensor board without having to debug any overhead complexity resulting from using the actual sensor board firmware.

This firmware exercises every sensor and indicator and reports one pass/fail verdict, to catch
dead parts, swapped buses, and wiring mistakes before the real firmware runs.

This is a best-effort smoke test, and by no means exhaustive: it confirms each part responds
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

1. Install the two GNSS modules on the back of the board.
2. Power the board (USB-C, the backplane, or a backplane adapter).
3. Set it flat on the bench and **leave it alone** for the whole run. The readings
   aren't range-checked, but a still board prints clean ~1 g and near-zero rotation
   that are easy to eyeball.
4. With a debug probe attached, run `cargo run --release` from this directory. It
   flashes the STM32H723ZG and streams the log over RTT (see `.cargo/config.toml`).

## Reading the result

The verdict comes out three ways:

- **Board:** rising chime + all LEDs lit = pass; low tone + solid red = fail.
- **RTT console:** the `cargo run` terminal prints each sensor's reading, then
  `CORE CHECKS PASSED` / `SOME CHECKS FAILED` (failures name the sensor and reason).
- **USB-C serial:** the same lines mirrored over the USB-C port, so you can read
  them without a debug probe (see [Reading over USB-C](#reading-over-usb-c)).

## Reading over USB-C

Plug into the USB-C port; the board shows up as a USB CDC-ACM serial device named
"Asteria Sensor Board Validation" (serial `sensorboard`). List the serial ports and
pick the one that matches:

```
ls /dev/tty.usbmodem*   # macOS
ls /dev/ttyACM*         # Linux
```

Then open that port with any serial console clients. Really any works (picocom, minicom, screen, cu), but [`tio`](https://github.com/tio/tio) is nice because it handles disconnects/reconnects gracefully.

```
tio <the port from above>
```

Early lines are buffered, so the whole run still shows if you connect a little late.

## Note on the magnetometer

It prints raw |B| (hard-iron uncorrected), which can sit well off the ~48 uT Earth field. That is expected and not a failure here. It's primarily an indicator of how much calibration the board needs.
