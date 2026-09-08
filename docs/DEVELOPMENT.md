# Development

`just` is the command interface for this repository. Run it inside a specific `fw-*` directory for board recipes, or from the repository root for commands that operate across workspaces. Run `just --list` in either location to see the available recipes. The repository has several independent Cargo workspaces rather than one root workspace, so `just` provides a consistent command interface across them.

## Building and flashing

From a board directory:

```sh
cd fw-test-board
just --list
just build
just run
```

`just run` builds the firmware, stores its timestamped ELF artifact, flashes the board, and attaches RTT output. Use `just flash` to flash without attaching, or `just attach` to reconnect to the most recently built ELF. Add `--release` for a release build. Do not add `--no-default-features` unless you know what you are doing.

From the repository root, `just build`, `just fmt`, `just ci-checks`, `just clippy`, and `just test` operate across the relevant workspaces. `just test` runs host-side tests; embedded firmware test recipes are no-ops.

## Flashing paths

Each board's `justfile` selects its flashing path. `probe-rs` is convenient because it flashes and attaches RTT in one step, and is often fast on smaller boards. `STM32_Programmer_CLI` is kept for boards where it is more compatible or reliable, and can be faster in some cases; its path flashes a converted binary and then uses `probe-rs attach` for RTT. `probe-rs` can hang on some target and probe combinations, so the configured fallback should be used when that happens.

## Device communication and logs

`asteria-tool` is the host CLI for USB RPC communication with a running board. Run these recipes from the repository root:

```sh
just connect
just cli fs ls /
just fetch-logs latest
just decode-logs path/to/fetched/log_0
```

Logs are decoded with the matching ELF from `.artifacts/` or configured object storage. If no ELF is available, fetching still works but decoding is skipped. See [`asteria-tool`'s README](../tools/asteria-tool/README.md) for connection modes and the full CLI.
