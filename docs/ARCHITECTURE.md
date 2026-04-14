# ARCHITECTURE

## Firmware and platform overview

All firmware is embedded `#![no_std]` Rust and uses `embassy` for hardware abstraction and async tasking.

All board targets compile for:

- `thumbv7em-none-eabihf`

MCU families in use:

- `STM32G473RCTx`
- `STM32G4A1KETx`
- `STM32H723ZG`

## Board targets

| Workspace | MCU | Flash path in `just run` |
| --- | --- | --- |
| `fw-communication-board` | `STM32G473RCTx` | `probe-rs run` |
| `fw-engine-board` | `STM32G473RCTx` | `STM32_Programmer_CLI` + `probe-rs attach` |
| `fw-fuel-control-board` | `STM32G473RCTx` | `STM32_Programmer_CLI` + `probe-rs attach` |
| `fw-lox-valve-control-board` | `STM32G4A1KETx` | `probe-rs run` |
| `fw-oxidizer-control-board` | `STM32G473RCTx` | `STM32_Programmer_CLI` + `probe-rs attach` |
| `fw-power-board` | `STM32G473RCTx` | `probe-rs run` |
| `fw-recovery-board` | `STM32G473RCTx` | `probe-rs run` |
| `fw-sensor-carrier` | `STM32H723ZG` | `STM32_Programmer_CLI` + `probe-rs attach` |
| `fw-test-board` | `STM32G473RCTx` | `probe-rs run` |

## Repository layout

```text
asteria-firmware/
├── fw-*/                 # per-board firmware workspaces (each has app/ + justfile)
├── crates/               # shared embedded crates (drivers, filters, wire types, utils)
├── tools/                # host-side tooling workspace (asteria-tool)
├── just/                 # shared just recipes + scripts (flash, artifacts, logs)
├── shared/               # source snippets reused via include!(), not a crate
├── hermes-can/           # git submodule (old CAN message library)
├── data-definitions/     # git submodule (CAN message library)
├── .b2.env.example       # template for B2/S3 credentials
└── justfile              # root workspace orchestration
```

## Workspace organization

Important notes:

- There is no root Cargo workspace.
- `fw-*`, `crates/`, and `tools/` are independent Cargo workspaces.
- Use `just` as the top-level orchestrator instead of running Cargo from repo root.

At a high level:

- `fw-*` workspaces: one board firmware binary each (`app/`)
- `crates/`: shared embedded libraries and device drivers
- `tools/`: host-side CLI tooling (`asteria-tool`)
- `just/`: shared automation recipes and scripts used across workspaces
