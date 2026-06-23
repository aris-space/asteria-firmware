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

Required tools:

- [`just`](https://just.systems/) task runner used by this repo
- [`probe-rs`](https://probe.rs/docs/getting-started/installation/) for flashing and debugging
- [`flip-link`](https://github.com/knurling-rs/flip-link), a linker used by firmware builds
- [`s5cmd`](https://github.com/peak/s5cmd) for artifact uploads/downloads to object storage
- [`taplo-cli`](https://taplo.tamasfe.dev/cli/) for `.toml` formatting
- [`jq`](https://jqlang.org/) for parsing JSON output
- [`pre-commit`](https://pre-commit.com/) for local commit checks
- [`defmt-print`](https://crates.io/crates/defmt-print) for decoding stored `defmt` logs

Most of the above can be installed via your package manager or directly with `cargo install`, depending on your setup preferences.

Additional tools (required for some boards):

- [`STM32CubeProgrammer`](https://www.st.com/en/development-tools/stm32cubeprog.html)
- [`arm-none-eabi-objcopy`](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

These are required for boards that cannot be flashed with `probe-rs` and use the fallback flashing path.

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
