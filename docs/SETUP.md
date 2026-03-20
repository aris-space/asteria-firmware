# SETUP

## 1) Clone the repository

```bash
git clone --recurse-submodules <repo-url>
cd asteria-firmware
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

## 2) Install required tooling

Required tools:

- [`just`](https://just.systems/) task runner used by this repo
- [`probe-rs`](https://probe.rs/docs/getting-started/installation/) for flashing and debugging
- [`flip-link`](https://github.com/knurling-rs/flip-link), a linker used by firmware builds
- [`s5cmd`](https://github.com/peak/s5cmd) for artifact uploads/downloads to object storage
- [`taplo-cli`](https://taplo.tamasfe.dev/cli/) for `.toml` formatting
- [`jq`](https://jqlang.org/) for parsing JSON output
- [`pre-commit`](https://pre-commit.com/) for local commit checks

Most of the above can be installed via your package manager or directly with `cargo install`, depending on your setup preferences.

Additional tools (required for some boards):

- [`STM32CubeProgrammer`](https://www.st.com/en/development-tools/stm32cubeprog.html)
- [`arm-none-eabi-objcopy`](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

These are required for boards that cannot be flashed with `probe-rs` and use the fallback flashing path.

## 3) Install Rust toolchain

The repository pins the required toolchain and target in `rust-toolchain.toml`.

Install Rust via [`rustup`](https://rustup.rs/). It will install the pinned toolchain/target automatically on first build.

## 4) Configure object storage credentials

We store every flashed firmware ELF in object storage and cache it locally in `.artifacts/`. This is mainly to make `defmt` log decoding reproducible later with the exact matching firmware image.

```bash
cp .b2.env.example .b2.env
```

Object storage is private, so you need credentials to access it. You can find them in the ARIS password manager under the `AV ASTERIA` collection. Copy the following fields into your `.b2.env` file:
* `ASTERIA_B2_KEY_ID`
* `ASTERIA_B2_APPLICATION_KEY`

## 5) Validate setup

From any board workspace:

```bash
cd fw-communication-board
just build
just run
```
