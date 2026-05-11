# DEVELOPMENT

`just` is the primary interface for this repo. Run `just --list` in any `fw-*` folder to see available commands.
The repo root has cross-workspace commands, which are mostly for CI.

## Building and flashing

```bash
just build          # compile firmware (thumbv7em-none-eabihf)
just run {args}     # build, timestamp, flash, and attach RTT output
just attach         # re-attach RTT output without reflashing
```

`just run` handles everything: it timestamps the artifact, flashes the board (via `probe-rs` or `STM32_Programmer_CLI` depending on configuration in the `fw-*/justfile`), and attaches RTT. Firmware defaults to the `debug` mode, which enables `defmt` log output.

To build for a critical test or launch, use `just build --release --no-default-features`.

At the repo root, `just build` compiles all workspaces, `just fmt` formats everything, and `just ci-checks` / `just clippy` / `just test` run lints and host-side tests.

## Device communication and logs

`asteria-tool` is the host CLI for USB RPC communication with a running board. Access it through `just`:

```bash
just connect                                          # interactive shell
just cli fs ls /                                      # single command
just fetch-logs latest                                # fetch and decode stored logs
just fetch-logs 3                                     # fetch last 3 log directories
just decode-logs .fetched-logs/<timestamp>/log_0      # re-decode an existing log
```

Logs are stored on-device as `defmt.bin` streams alongside `build_info.txt`, which contains an `artifact_timestamp_ms` linking the log to the exact ELF used at flash time. `just fetch-logs` pulls and decodes them using the matching ELF from `.artifacts/` or object storage (configured via `.b2.env`). If no ELF is found, fetching still works but decoding is skipped.

`asteria-tool` defaults to raw USB with a serial fallback. Pass `--connect raw` to force USB-only, or `--connect serial --port <PATH>` for serial.
