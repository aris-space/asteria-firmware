# Development

Use `just` for normal workflows. Run `just --list` for available commands. There is no root Cargo workspace.

## Board Workflow

Run board-specific commands inside a `fw-*` directory:

```sh
just build        # build for thumbv7em-none-eabihf
just run          # build, upload ELF, flash, attach RTT
just flash        # build, upload ELF, flash only
just attach       # attach RTT using latest local ELF
just ci-checks    # clippy with warnings denied
just clean        # clean this workspace
```

`just run` reads `chip`, `bin`, and `use_probe_rs` from the board `justfile`. If `use_probe_rs` is false, flashing uses `STM32_Programmer_CLI` plus `arm-none-eabi-objcopy`.

## Release Builds

```sh
just build --release
```

Many boards use default features for `debug`, `defmt`, and `panic-probe`. For production-style builds on those boards:

```sh
just build --release --no-default-features
just run --release --no-default-features
```

Do not use `--no-default-features` blindly; some newer firmware uses default features for real functionality, for example storage on `fw-sensor-carrier-v3`.

## Root Commands

Run cross-workspace commands from the repository root.

`just test` runs host-side tests for `crates/` and `tools/`. Embedded firmware test recipes are no-ops.

See [Contributing](../CONTRIBUTING.md#5-required-local-checks-before-push) for checks to run before pushing.

## Artifacts

`just run` and `just flash` timestamp the ELF, cache it in `.artifacts/`, upload it to object storage if configured, and flash the board. This lets fetched logs find the matching ELF later.

Useful root recipes:

```sh
just upload-elf path/to/file.elf
just upload-elf-as path/to/file.elf 1234567890.elf
just clean-artifacts
just sync-artifacts
```

## Device Communication

`asteria-tool` talks to a running board over USB RPC. Run these commands from the repository root.

```sh
just connect
just cli info
just cli fs ls /
just cli fs pull -r /log_0 ./log_0
```

Connection options pass through:

```sh
just connect --connect raw
just connect --connect serial --port /dev/ttyACM0 --baud 115200
```

See [../tools/asteria-tool/README.md](../tools/asteria-tool/README.md) for the full CLI.

## Logs

From the repository root, fetch and decode logs:

```sh
just fetch-logs latest
just fetch-logs 3      # fetch log_3
just fetch-logs all
```

Decode an already fetched log (substitute its path):

```sh
just decode-logs '.fetched-logs/<timestamp>/log_0'
```

Logs are decoded with `defmt-print` using the matching ELF from `.artifacts/` or object storage. If no ELF is found, fetching still works but decoding is skipped.

## Troubleshooting

- Build fails before project code: check the pinned Rust toolchain and target installed.
- Flashing fails on fallback boards: check `STM32_Programmer_CLI` and `arm-none-eabi-objcopy`.
- Artifact upload fails: check `.b2.env`, `s5cmd`, and network access.
- Log fetch fails immediately: check `defmt-print`.
- USB connect fails: close other `asteria-tool` instances.
