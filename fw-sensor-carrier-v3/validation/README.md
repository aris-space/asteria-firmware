# Sensor Board hardware validation

This firmware validates a fully assembled sensor board with a minimal test program, avoiding the overhead of the full flight firmware.

It is also a bring-up sandbox for the clock tree, bus speeds, and peripheral setup. The goal is to prove that the low-level configuration is sound in isolation before reusing it in the more complex flight firmware. Ideally, the clock config and resource map here will later be shared with that firmware instead of duplicated (see the TODOs in `clocks.rs` and `resources/mod.rs`).

The firmware exercises every sensor and indicator, then reports a single pass/fail verdict to catch dead parts, swapped buses, and wiring mistakes early.

This is a best-effort smoke test, not a full qualification. It confirms that each checked part responds and can be read without obvious errors, and prints the readings for the operator to inspect. It does not range-check values or test calibration, accuracy, or every failure mode, so a pass means "nothing obviously broken," not "fully qualified."

## What it checks

- **LEDs + buzzer** (operator confirms by eye/ear)
- **2x IMU** (LSM6DSO32, SPI): WHO_AM_I, an INT1 data-ready edge, accel/gyro/temp
- **2x barometer** (MS5607, I2C): PROM read, then pressure/temp
- **2x magnetometer** (LSM303AGR, I2C): WHO_AM_I, then |B|
- **2x humidity/temp** (SHT4x, I2C): serial number, then RH/temp
- **2x GNSS** (u-blox, UART): link up, valid UBX packets, UBX-NAV-STATUS present
- **Flash** (W25Q01JV, OCTOSPI quad): JEDEC ID, then a quad readback compared against a single-line read to exercise IO2/IO3 (sets the QE bit, a one-time persistent write)
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
3. Set it flat on the bench and **leave it alone** for the whole run. The readings aren't range-checked, but a still board prints clean ~1 g and near-zero rotation that are easy to eyeball.
4. With a debug probe attached, run `cargo run --release` from this directory. It flashes the STM32H723ZG and streams the log over RTT (see `.cargo/config.toml`).

## CPU frequency boost option byte

The 544 MHz core clock needs the **CPUFREQ_BOOST** option byte set, once per board. If a fresh board panics in `embassy_stm32::init()` at startup, that's why. Set it with STM32CubeProgrammer (probe attached):

```
STM32_Programmer_CLI -c port=SWD -ob CPUFREQ_BOOST=1
```

## Reading the result

The verdict comes out three ways:

- **Board:** rising chime + all LEDs lit = pass; low tone + solid red = fail.
- **RTT console:** the `cargo run` terminal prints each sensor's reading, then `CORE CHECKS PASSED` / `SOME CHECKS FAILED` (failures name the sensor and reason).
- **USB-C serial:** the same lines mirrored over the USB-C port, so you can read them without a debug probe (see [Reading over USB-C](#reading-over-usb-c)).

## Reading over USB-C

Plug into the USB-C port; the board shows up as a USB CDC-ACM serial device named "Asteria Sensor Board Validation" (serial `sensorboard`). List the serial ports and pick the one that matches:

```
ls /dev/tty.usbmodem*   # macOS
ls /dev/ttyACM*         # Linux
```

Then open that port with any serial console clients. Really any works (picocom, minicom, screen, cu), but [`tio`](https://github.com/tio/tio) is nice because it handles disconnects/reconnects gracefully.

```
tio <the port from above>
```

Early lines are buffered, so the whole run still shows if you connect a little late.
