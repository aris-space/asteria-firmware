# Architecture

ASTERIA firmware is embedded Rust for STM32 boards. It is `#![no_std]`, uses [Embassy](https://embassy.dev/) for async tasking, and compiles for `thumbv7em-none-eabihf`.

Each `fw-*` workspace builds one board firmware binary. Shared embedded code lives in `crates/`; host tooling lives in `tools/`; message definitions come from `data-definitions/` and `hermes-can/`.

## Board Targets

| Workspace | Binary | MCU | `just run` flash path |
| --- | --- | --- | --- |
| `fw-communication-board` | `fw-communication-board` | `STM32G473RCTx` | `probe-rs run` |
| `fw-engine-board` | `fw-engine-board` | `STM32G473RCTx` | `STM32_Programmer_CLI` + `probe-rs attach` |
| `fw-fuel-control-board` | `fw-fuel-control-board` | `STM32G473RCTx` | `STM32_Programmer_CLI` + `probe-rs attach` |
| `fw-lox-valve-control-board` | `fw-lox-valve-control-board` | `STM32G4A1KETx` | `probe-rs run` |
| `fw-oxidizer-control-board` | `fw-oxidizer-control-board` | `STM32G473RCTx` | `STM32_Programmer_CLI` + `probe-rs attach` |
| `fw-power-board` | `fw-power-board` | `STM32G473RCTx` | `probe-rs run` |
| `fw-recovery-board` | `fw-recovery-board` | `STM32G473RCTx` | `probe-rs run` |
| `fw-sensor-carrier` | `fw-sensor-carrier` | `STM32H723ZG` | `STM32_Programmer_CLI` + `probe-rs attach` |
| `fw-sensor-carrier-v3` | `fw-sensor-carrier-v3` | `STM32H723ZG` | `probe-rs run` |
| `fw-test-board` | `fw-test-board` | `STM32G473RCTx` | `probe-rs run` |

The flash path comes from each board's `justfile`.

## Layout

```text
asteria-firmware/
|-- fw-*/                 # board firmware workspaces
|   |-- app/              # main firmware binary crate
|   |-- Cargo.toml
|   |-- Embed.toml
|   `-- justfile
|-- crates/               # shared embedded crates
|-- tools/                # host-side tools
|-- data-definitions/     # message/datapoint definitions submodule
|-- hermes-can/           # legacy CAN message library submodule
|-- just/                 # shared recipes and scripts
|-- shared/               # include!() snippets
|-- docs/
`-- justfile              # root orchestration
```

There is no root Cargo workspace. `fw-*`, `crates/`, and `tools/` are separate workspaces. Use root `just` for cross-workspace commands.

## Firmware Shape

Most board crates follow this shape:

- `main.rs` - startup, peripheral initialization, task spawning
- `build.rs` / `build_info.rs` - build metadata used in logs
- `can_impl.rs` - board-specific CAN handling where present
- `drivers.rs`, `sensors.rs`, `actuators.rs`, `tasks/`, `resources/` - board hardware and async task code

`fw-sensor-carrier-v3` also has a `validation/` crate for sensor carrier v3 hardware bring-up.

## Shared Code

- `crates/embedded-utils` - common embedded helpers
- `crates/filters` - filtering utilities
- `crates/can-utils` - derive-based CAN helpers
- `crates/devices/*` - reusable device drivers
- `crates/wire-types` - shared wire-level types
- `tools/asteria-tool` - host CLI for USB RPC and board filesystem access

## Automation

- `just/firmware.just` - board build/run/flash/attach/check recipes
- `just/artifacts.just` - ELF artifact upload/cache recipes
- `just/logs.just` - `asteria-tool` wrappers and log fetch/decode recipes
- `just/scripts/` - shell implementations used by the recipes
