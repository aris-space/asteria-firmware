# Camera board validation

Small firmware to check an assembled camera board by eye. It turns on the debug LED, prints the jumper address and switches the camera on.

## Run

Mount the board on a RunCam Split, power it from the harness, attach a debug probe and run from this directory:

```
cargo run --release
```

## What to check

- The debug LED is on.
- The printed address matches the bridged ADD jumpers (ADD0 is the lowest bit).
- The green camera power LED is on and the camera starts recording.

Not covered: CAN, the camera UART and the reset button.
