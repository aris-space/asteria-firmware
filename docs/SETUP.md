# Setup Guide

First-time setup for ASTERIA firmware development. Complete this before working on the repository.

## Clone

```sh
git clone --recurse-submodules git@github.com:aris-space/asteria-firmware.git
cd asteria-firmware
```

If already cloned:

```sh
git submodule update --init --recursive
```

Submodules are required. `data-definitions` provides ASTERIA message/datapoint definitions; `hermes-can` is still used by older board firmware. Some required submodules are ARIS-private, so a public checkout may not contain everything needed to build or flash all targets.

## Rust

Install Rust with [`rustup`](https://rustup.rs/). The repo pins its nightly toolchain, components, and `thumbv7em-none-eabihf` target in [../rust-toolchain.toml](../rust-toolchain.toml), so first build should install them automatically.

## Tools

Required:

- `just` - task runner
- `probe-rs` - flashing and RTT output
- `flip-link` - embedded linker
- `pre-commit` - local hooks
- `taplo-cli` - TOML formatting
- `jq` - script JSON parsing
- `s5cmd` - artifact upload/download
- `defmt-print` - stored log decoding

Some boards also need:

- `STM32CubeProgrammer`
- `arm-none-eabi-objcopy`

Install `defmt-print` with:

```sh
cargo install defmt-print
```

## Hooks

```sh
pre-commit install
```

Before pushing:

```sh
pre-commit run --all-files
just fmt --check
just ci-checks
just test
```

## Artifact Storage

Flashed ELF files are cached locally and uploaded so stored `defmt` logs can be decoded with the exact matching binary.

```sh
cp .b2.env.example .b2.env
```

Fill in:

- `ASTERIA_B2_KEY_ID`
- `ASTERIA_B2_APPLICATION_KEY`

Credentials are in the ARIS password manager under `AV ASTERIA`.

## Validate

```sh
cd fw-communication-board
just build
```

With hardware connected:

```sh
just run
```
