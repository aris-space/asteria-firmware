# Camera Board hardware validation

This firmware validates an assembled camera control board with a minimal test program, avoiding the overhead of the full firmware. It reports a single pass/fail verdict to catch dead parts and wiring mistakes early.

This is a best-effort smoke test, not a full qualification. A pass means "nothing obviously broken," not "fully qualified."

## What it checks

- **Debug LED** (operator confirms by eye)
- **Address jumpers** (ADD0..ADD3): reads and prints the 4-bit address; the operator compares it against the bridged jumpers
- **Camera power switch** (EN_CAMERA, TPS22917): switches the camera on; the operator confirms the green camera power LED
- **Camera UART** (USART1, RunCam Device Protocol at 115200 baud): sends GET_DEVICE_INFO and asserts a reply with a valid CRC, then prints the protocol version and feature bits

## What it does NOT check

- **CAN:** not exercised.
- **Reset button:** not exercised.
- **Address jumper wiring:** the address is printed, not compared against an expected value.

## Run

1. Mount the board on a RunCam Split and power it from the harness.
2. With a debug probe attached, run `cargo run --release` from this directory. It flashes the STM32G4A1KE and streams the log over RTT (see `.cargo/config.toml`).

## Reading the result

- **Board:** debug LED solid on = pass; fast blinking = fail.
- **RTT console:** each check's result, then `CORE CHECKS PASSED` / `SOME CHECKS FAILED` (failures name the part and reason).
