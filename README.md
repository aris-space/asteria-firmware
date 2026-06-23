# ASTERIA Firmware

[![CI](https://github.com/aris-space/asteria-firmware/actions/workflows/check.yml/badge.svg)](https://github.com/aris-space/asteria-firmware/actions/workflows/check.yml)
[![Rust](https://img.shields.io/badge/rust-nightly-orange?logo=rust)](https://www.rust-lang.org/tools/install)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue)](#license)

This repository contains the firmware for the ASTERIA rocket, the ARIS launch vehicle competing at EuRoC 2026. It includes firmware for the avionics and ground support electronics.

> [!NOTE]
> This software was developed by the ASTERIA Avionics team for ARIS. It is provided as-is, without guarantees, and external contributions are unlikely to be accepted. Most of the programs here require ASTERIA-specific or compatible hardware. Some required submodules are ARIS-private, so a public checkout may not contain everything needed to build or flash all targets. The code is published primarily for transparency, reference, and project continuity.
>
> Point of contact: Louis Schell, [louis.schell@aris-space.ch](mailto:louis.schell@aris-space.ch)

The firmware is written in embedded Rust and targets STM32 microcontrollers. We use the [Embassy](https://embassy.dev/) ecosystem for the HAL and for the async tasking/runtime functionality that would otherwise be provided by an RTOS.

Each `fw-*` directory is one board firmware workspace. Shared embedded crates live in `crates/`, host-side tooling lives in `tools/`, and message/datapoint definitions come from the `data-definitions` submodule.

## Provenance

This repository was hard-forked from [`hermes-firmware`](https://github.com/aris-space/hermes-firmware) for the ASTERIA project. Embedded Rust has been used within ARIS since project [NICOLLIER](https://github.com/aris-space/nicollier-core), through [HERMES](https://github.com/aris-space/hermes-firmware), and now in ASTERIA. Large parts of this repository are iterations on earlier HERMES patterns, tools, and embedded conventions, adapted for ASTERIA's hardware and mission needs.

## Board Overview

| Firmware                     | Common name             | MCU             |
| ---------------------------- | ----------------------- | --------------- |
| `fw-communication-board`     | Communication board     | `STM32G473RCTx` |
| `fw-engine-board`            | Engine board            | `STM32G473RCTx` |
| `fw-fuel-control-board`      | Fuel control board      | `STM32G473RCTx` |
| `fw-lox-valve-control-board` | LOX valve control board | `STM32G4A1KETx` |
| `fw-oxidizer-control-board`  | Oxidizer control board  | `STM32G473RCTx` |
| `fw-power-board`             | Power board             | `STM32G473RCTx` |
| `fw-recovery-board`          | Recovery board          | `STM32G473RCTx` |
| `fw-sensor-carrier`          | Sensor carrier          | `STM32H723ZG`   |
| `fw-sensor-carrier-v3`       | Sensor carrier          | `STM32H723ZG`   |
| `fw-test-board`              | Test board              | `STM32G473RCTx` |

See [Architecture and board targets](docs/ARCHITECTURE.md) for flashing paths and more repository layout details.

## Quick Start

Before working on this repository, complete the [setup guide](docs/SETUP.md).

Useful follow-up docs:

- [Development workflow](docs/DEVELOPMENT.md)
- [Architecture and board targets](docs/ARCHITECTURE.md)
- [Contributing](CONTRIBUTING.md)

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) before opening a PR. If you have not worked through [Setup](docs/SETUP.md), do that first.

> [!IMPORTANT]
> By contributing, you agree that your contribution will be licensed under MIT OR Apache-2.0, matching the source license of this repository.

## License

Source files are licensed under [`MIT`](LICENSE-MIT) OR [`Apache-2.0`](LICENSE-APACHE), at your option. Contributions are licensed under those same terms.

SPDX-License-Identifier: `MIT OR Apache-2.0`

See [NOTICE](NOTICE) for repository copyright attribution.

Note that the compiled firmware links against the GPL-licensed [`asteria-data-definitions`](https://github.com/aris-space/asteria-data-definitions), so distributed firmware binaries are combined works under GPL-3.0-or-later.
