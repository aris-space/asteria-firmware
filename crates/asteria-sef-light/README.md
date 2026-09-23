# asteria-sef-light

Allocation-free, `no_std` vertical state estimation for Project ASTERIA.

SEF Light combines two independent IMU/AHRS and vertical-Kalman-filter chains with shared
barometer and dual-GNSS aiding. It exposes both chain estimates and a hysteretically selected
output.

The complete state, process and measurement derivations, delayed-fusion behavior, and redundancy
policy are documented in [the SEF Light design document](../../docs/sef-light.md).

## Example

A compile-checked example is available in [`examples/basic.rs`](examples/basic.rs):

```sh
cargo run -p asteria-sef-light --example basic
```

## Regenerate the symbolic model

```sh
uv sync --project tools/sef-light
uv run --project tools/sef-light python tools/sef-light/generate.py
```

Files under `src/generated` are generated and must not be edited by hand.
