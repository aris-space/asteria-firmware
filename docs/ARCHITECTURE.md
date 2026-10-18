# Architecture

There is no root Cargo workspace. `fw-*`, `crates/`, and `tools/` are separate workspaces. Use root `just` for cross-workspace commands.

## Firmware Shape

Most board crates follow this shape:

- `main.rs` - startup, peripheral initialization, task spawning
- `build.rs` / `build_info.rs` - build metadata used in logs
- `can_impl.rs` - board-specific CAN handling where present
- `drivers.rs`, `sensors.rs`, `actuators.rs`, `tasks/`, `resources/` - board hardware and async task code

`fw-sensor-carrier-v3` also has a `validation/` crate for sensor carrier v3 hardware bring-up.

## Board LEDs

- Green: fast blink (250/250 ms) during startup, slow blink (750 ms on, 250 ms off; 75% duty) while running
- Red: slow blink (1800 ms on, 200 ms off; 90% duty) on a clean release build (no uncommitted changes, `--release --no-default-features`), fast blink (125/125 ms) otherwise
- Yellow: board specific

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
