# recovery-sender

Terminal UI to manually send CAN messages to the Recovery Board and watch its replies
(board status, CATS states, steering positions, separation/deployment occurred).

```bash
cargo run --manifest-path tools/recovery-sender/Cargo.toml -- can0
```

Needs a SocketCAN interface (CAN FD) on Linux. On other OSes the UI runs but messages are dropped.

Keys: `up/down` select, `left/right` change value (shift: x10), `enter` send, `0` zero a position,
`s` stream the steering target at 10 Hz (for the watchdog), `q` quit.
Separation, deployment and reset need `enter` twice.
