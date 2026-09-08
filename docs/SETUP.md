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

The embedded rust ecosystem uses a number of tools for building, flashing, and debugging. The following are used for development in this repository. You will rarely need to invoke all of them yourself, as they are used by the `just` task runner, but they must be installed for the tasks to work.

Tools:

- [`just`](https://just.systems/) task runner used by this repo
- [`probe-rs`](https://probe.rs/docs/getting-started/installation/) for flashing and debugging
- [`flip-link`](https://github.com/knurling-rs/flip-link), a linker used by firmware builds
- [`s5cmd`](https://github.com/peak/s5cmd) for optional artifact uploads/downloads to object storage
- `defmt-print` for fetching and decoding logs (`cargo install defmt-print`)
- [`taplo-cli`](https://taplo.tamasfe.dev/cli/) for `.toml` formatting
- [`jq`](https://jqlang.org/) for parsing JSON output
- [`pre-commit`](https://pre-commit.com/) for local commit checks

Building also needs a C/C++ toolchain and libclang; the scripts use Bash and Perl.

Most of the above can be installed via your package manager or directly with `cargo install`, depending on your setup preferences.

Some boards require additional tools:

- [`STM32CubeProgrammer`](https://www.st.com/en/development-tools/stm32cubeprog.html)
- [`arm-none-eabi-objcopy`](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

These are required for boards that cannot be flashed with `probe-rs` and use the fallback flashing path.

## Pre-commit Hooks

We use CI checks to enforce formatting and linting. To avoid wasting time on CI, we provide pre-commit hooks that check formatting and file syntax locally _before_ committing.
To install the hooks, run:

```sh
pre-commit install
```

You will need pre-commit installed. Run `just ci-checks` separately for linting.

## Artifact Storage

Flashed ELF files are cached locally and, if configured, uploaded to an S3 object storage, so that stored `defmt` logs can be decoded with the exact matching binary.

For remote uploads, copy the example environment file to `.b2.env`:
```sh
cp .b2.env.example .b2.env
```

Then, fill in the following environment variables with credentials from the ARIS password manager:

- `ASTERIA_B2_KEY_ID`
- `ASTERIA_B2_APPLICATION_KEY`

These credentials can be found under `AV ASTERIA`.

> [!IMPORTANT]
> Do not commit `.b2.env` to the repository. It contains sensitive credentials, and is gitignored for a reason.

## Validate

To validate your setup, you can build and run the test board firmware.

To build:
```sh
cd fw-test-board
just build
```

To flash and run on a connected test board, you'll need to connect the debug probe to the board and run:
```sh
just run
```
