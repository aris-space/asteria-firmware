# HERMES Firmware
This repository contains all firmware used in the HERMES launch vehicle.

Currently, there are 4 binary targets:
1. `communication_board`
2. `sensor_carrier`
3. `recovery_board`
4. `power_board`

These crates are all written using the [embassy async runtime](https://github.com/embassy-rs/embassy/) as a substitute for an RTOS. All firmware runs on the `stm32g473rc` microcontroller.

There are several additional library crates in the workspace:
1. `embedded-utils` (utilities for HERMES embedded)
2. `devices/ms5607` (pressure sensor driver)
3. `devices/lsm6dso32` (accelerometer and gyroscope IMU driver)

We also have a submodule for the `hermes-can` library, which is the central repository for all CAN messages used in the system. See the [hermes-can repository](https://github.com/aris-space/hermes-can) for more information.

## Prerequisites
We highly recommend using a Unix-like system for development, although everything should work on Windows as well.

The Rust MSRV for this repository is `1.85.0`.

The microcontrollers run on the ARM Cortex-M4 architecture, so you may need to install the `thumbv7em-none-eabihf` target for the Rust compiler. You should also install the `rustfmt` and `clippy` tools.
```sh
rustup target add thumbv7em-none-eabihf
rustup component add rustfmt clippy
```

For flashing the code and debugging, we use the [probe-rs](https://probe.rs/) toolchain. Simply visit their website and follow the installation instructions.

For testing, we use the `cargo-hack` tool. You can install it with the following command:
```sh
cargo install cargo-hack --locked
```

## Building
All targets are set up with runners so that they can be built very simply with Cargo. Make sure you are in the directory of the firmware you want to build.
```sh 
cd path/to/board/firmware
cargo build
```

To upload the firmware to the microcontroller, you can use the `cargo run` command. Make sure you are in the directory of the firmware you want to compile and flash.
```sh
cd path/to/board/firmware
cargo run
```

Each binary crate comes with several features.
- `defmt` enables logging via the defmt framework. `defmt` is enabled by default, but for production builds it should be disabled because defmt may result in blocking behavior if the logging buffer is full (see [defmt-documentation](https://docs.rs/defmt-rtt/latest/defmt_rtt/#blockingnon-blocking) [yes, we are working on a fix for this]).
- `debug` enables `panic-probe` and defmt for debugging. This feature is enabled by default. Panic-probe prints out the panic message over defmt when a panic occurs. Again, this feature should be disabled for production builds for the same reason.

The crates are all compiled in optimized mode by default. See the workspace `Cargo.toml` file for more information. To compile for production, please use the `--release` flag.

## Compiling for Production
To compile and run for production, you should disable the `defmt` and `debug` features.
```sh
cd path/to/board/firmware
cargo build --release --no-default-features
cargo run --release --no-default-features
```

## Checking Your Code Locally

We have a few tests in the repository that are automatically checked by CI. You can run them locally with the following commands.

### Cargo Check
Run a full code check (type-checking and basic analysis) with:
```sh
cargo hack check --workspace --feature-powerset --all-targets --target thumbv7em-none-eabihf
```
This command checks all targets and enumerates all possible feature combinations to ensure they are configured properly.

Note: This may take a while to run the first time. If you just want to check a single crate, you can simply run `cargo check` in the crate directory.

### Cargo Clippy
Catch potential issues and enforce linting rules by running:
```sh
cargo hack clippy --workspace --all-targets --feature-powerset --target thumbv7em-none-eabihf -- -D warnings
```
This command runs Clippy on all targets and enumerates all possible feature combinations to ensure they are configured properly.

Note: This may take a while to run the first time. If you just want to check a single crate, you can simply run `cargo clippy` in the crate directory.

## Committing and Pushing
As in all HERMES repositories, the main branch is protected. You’ll need to create a branch and submit a pull request. CI tests will run automatically to check your changes. If everything is green, request a review from someone, and they can merge your changes.

## Known Issues
Please refer to our GitHub issues for more details on known problems and bugs.

- **Logging framework blocks** – With `defmt-03`, the logging framework may block the system when the logging buffer becomes full. This issue is mitigated by using the `defmt` feature, which disables logging in production builds.
- **Feature unification** – Since every binary crate depends on the `hermes-can` library, compiling packages within the workspace unifies their features. This may inadvertently enable features that are not intended to be active. CI addresses this by using `cargo-hack` to build each package separately. While this is not a problem for local development, *please be aware of it when building the entire workspace.*
