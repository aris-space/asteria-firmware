# DEVELOPMENT

## Daily workflow with `just`

`just` is the primary interface for this repo. Run `just --list` in your current directory to see available recipes.

Board workspaces (`fw-*`) include flashing/debug recipes. Repo root includes cross-workspace orchestration commands.

## Board workspace commands (`fw-*`)

```bash
just build
just run
just run --release
just attach
just fetch-logs latest
just decode-logs .fetched-logs/<timestamp>/log_0
just connect
just cli fs ls /
```

| Command | Purpose |
| --- | --- |
| `just build` | Compile board firmware for `thumbv7em-none-eabihf`. |
| `just run` | Build, timestamp artifact, upload ELF, flash, and attach RTT output. |
| `just run --release` | Same as `just run`, but with release profile. |
| `just attach` | Re-attach to RTT output without reflashing. |
| `just connect` | Start interactive `asteria-tool` shell for device filesystem/RPC. |
| `just cli ...` | Run one non-interactive `asteria-tool` command. |
| `just fetch-logs ...` | Fetch log directories and decode with matching ELF when possible. |
| `just decode-logs <dir>` | Re-decode an already-fetched log directory. |

## Repo root commands

```bash
just fmt
just build
just ci-checks
just clippy
just test
```

| Command | Purpose |
| --- | --- |
| `just fmt` | Run formatting in each workspace. |
| `just build` | Build all workspaces (`fw-*`, `crates`, `tools`). |
| `just ci-checks` | Run CI-style checks locally. |
| `just clippy` | Run clippy across all workspaces. |
| `just test` | Run host-side tests (`tools` workspace). |

## `asteria-tool` (MCU communication)

`asteria-tool` is the host CLI used to communicate with running boards over USB RPC.

You usually invoke it through `just`:

- `just connect` starts the interactive shell
- `just cli ...` runs one non-interactive command
- `just fetch-logs ...` and `just decode-logs ...` build on top of it

Common usage:

```bash
just connect
just cli fs ls /
just cli fs info
just cli fs pull -r /log_0 ~/Downloads
```

Connection modes (from `asteria-tool`):

- `--connect auto` (default): raw USB first, serial fallback if configured
- `--connect raw`: raw USB only
- `--connect serial --port <PATH>`: serial only

## Debugging basics (`probe-rs` + `defmt`)

- `probe-rs` handles target programming, attach, and RTT transport
- `defmt` emits compact binary logs from firmware
- `defmt-print` decodes logs on host using the matching ELF
- Board firmware defaults to the `debug` feature, enabling `defmt` and `panic-probe`

Practical options:

| Goal | Recommended command |
| --- | --- |
| Flash + start logs in one step | `just run` |
| Re-attach logs without reflashing | `just attach` |
| Build with release optimizations | `just run --release` |
| Fetch and decode stored logs | `just fetch-logs latest` |
| Re-decode an existing log folder | `just decode-logs <log_dir>` |

Notes:

- `just run` chooses per-board flash path (`probe-rs run` or `STM32_Programmer_CLI` + `probe-rs attach`)
- RTT live output depends on `defmt` being enabled in the firmware build
- If you build without default features, debug output may be reduced or absent

## Logs and decoding

The logging pipeline is based on `defmt` plus timestamped firmware artifacts.

Each log directory on-device contains:

- `defmt.bin` (raw binary log stream)
- `build_info.txt` with `artifact_timestamp_ms`

That timestamp links the log to the exact firmware ELF used at flash time.

End-to-end flow:

1. `just run` sets `ASTERIA_ARTIFACT_TIMESTAMP_MS` and builds firmware
2. The ELF is cached in `.artifacts/` and uploaded to object storage as `<timestamp>.elf`
3. Firmware writes logs to `/log_N/...` with matching `artifact_timestamp_ms`
4. `just fetch-logs` pulls logs via `asteria-tool`
5. Decode resolves ELF from local cache first, then object storage fallback via `.b2.env`
6. `defmt-print` decodes into `decoded.txt` (and prints unless `--quiet`)

Lookup order for matching ELF:

1. `.artifacts/<artifact_timestamp_ms>.elf`
2. object storage download using `.b2.env`

Commands:

```bash
just fetch-logs
just fetch-logs latest
just fetch-logs 3
just fetch-logs latest --quiet
just decode-logs .fetched-logs/<timestamp>/log_0
```

If no matching ELF is available, fetching still works, but decoding is skipped for that log.
