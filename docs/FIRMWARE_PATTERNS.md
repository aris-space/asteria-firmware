# Firmware Architecture

This document describes the layered architecture used in the sensor carrier v3 firmware. The same patterns can be reused for other boards.

## Layers

```
main.rs                    Entry point: clock init, resource split, spawn
  |
startup.rs                 Wire resources to tasks, spawn on executors
  |
  +-- resources/           Raw peripheral setup (SPI, I2C, UART, GPIO)
  +-- tasks/               Application logic
  |     +-- readout/       Sensor readout tasks
  |     +-- blinky.rs      Simple LED task
  +-- storage.rs           Filesystem + defmt log sink
  +-- signals.rs           PubSubChannel + Watch arrays per sensor
  +-- measurements.rs      Sample types (Timestamped<T>, ImuSample, etc.)
  +-- sensors.rs           Sensor ID types (ImuId, BarometerId, etc.)
```

## Resources

Each file in `resources/` owns one peripheral type. Every resource follows the same pattern:

```rust
// Private config function
fn config() -> peripheral::Config { ... }

// Public setup consuming the assigned pins/DMA
impl MyPeripheral {
    pub fn setup(self) -> ConcreteType { ... }
}
```

`setup()` takes no arguments — the config is internal to the resource file. This keeps `startup.rs` clean:

```rust
let imu1 = resources.imu1.setup();
let bus1 = resources.bus1.setup();
```

Pin assignments live in `resources/mod.rs` via the `assign_resources!` macro. Adding a new peripheral means:
1. Add pin group in `mod.rs`
2. Create a new file with `fn config()` + `impl setup()`

### Resource files

| File | Returns | Notes |
|------|---------|-------|
| `sensors.rs` | `(SpiDevice, ExtiInput)` | Async SPI with DMA, CS pin, interrupt |
| `buses.rs` | `SharedI2cBus` | Async I2C wrapped in static Mutex for sharing |
| `uart.rs` | `Uart<'static, Async>` | Async UART with DMA |
| `flash.rs` | `&'static mut BoardFlash` | Blocking SPI, static allocation |
| `usb.rs` | `UsbDriver` | USB OTG HS |
| `can.rs` | `Can<'static>` | FDCAN (todo) |
| `leds.rs` | `Output<'static>` | Simple GPIO |

## Sensor Readout Tasks

All sensor tasks live in `tasks/readout/` and follow the same structure:

### Generic inner / concrete wrapper pattern

Embassy tasks can't be generic (macro limitation). So each task has:
1. Generic `Inactive<SPI, INT>` / `Active<SPI, INT>` structs with `embedded-hal-async` trait bounds
2. A generic `run_inner(...)` that orchestrates the state machine
3. A thin `#[embassy_executor::task]` wrapper with concrete types

```rust
async fn run_inner<SPI, INT>(spi: SPI, int1: INT, id: ImuId) -> !
where
    SPI: embedded_hal_async::spi::SpiDevice,
    INT: embedded_hal_async::digital::Wait,
{ ... }

#[embassy_executor::task(pool_size = 2)]
pub async fn task(spi: SpiDevice, int1: ExtiInput<'static, Async>, id: ImuId) -> ! {
    run_inner(spi, int1, id).await
}
```

### Inactive / Active state machine

Every sensor readout uses this pattern:

```
loop {
    Inactive::run() → tries init with exponential backoff → returns Active
    Active::run()   → measurement loop, error counting   → returns Inactive
}
```

- **Inactive**: retries initialization with `backoff(attempt)`. Shared backoff function in `tasks/mod.rs`.
- **Active**: reads sensor, publishes to signals. On `MAX_CONSECUTIVE_ERRORS` consecutive failures, transitions back to Inactive.
- The sensor driver is destroyed on transition to Inactive and reconstructed on the next init attempt.

### Sensor-specific notes

- **IMU** (`readout/imu.rs`): FIFO mode at 833Hz, batch reads accel/gyro pairs, interpolates per-sample timestamps across the FIFO window. Uses `submit_imu_samples()` for batch signal publish.
- **Barometer** (`readout/barometer.rs`): Timer-based polling with `Timer::at(next_sample)` for consistent intervals. Accepts timestamp delay parameter.
- **Magnetometer** (`readout/magnetometer.rs`): LSM303AGR in continuous mag mode. Initialisation enables offset cancellation, LPF, and built-in accelerometer.
- **GNSS** (`readout/gnss.rs`): UBX protocol parsing via `ublox` crate. Inactive waits for `NavStatus` with valid fix before transitioning to Active. Active parses `NavPvt` packets.

### Raw data convention

Readout tasks publish raw chip data — no axis corrections, no calibration, no frame rotation. A future processing task will apply per-sensor rotation matrices and calibration from a params system.

## Signals

`signals.rs` defines per-sensor-instance channels using the `define_signal!` macro:

```rust
define_signal!(
    IMU_CHANNELS, IMU_WATCHES, submit_imu_sample, submit_imu_samples, imu_watch:
    ImuSample, ImuId,
    cap = 64, subs = 4, pubs = 2,
    count = IMU_COUNT, watchers = 4
);
```

This creates:
- `IMU_CHANNELS`: array of `PubSubChannel`, one per sensor instance
- `IMU_WATCHES`: array of `Watch`, one per sensor instance
- `submit_imu_sample(sample)`: publishes to channel + watch, indexed by `sample.sensor_id`
- `submit_imu_samples(&[sample])`: batch publish, updates watch only with last sample

Consumers subscribe to the channel for streamed data or watch the Watch for latest value.

## Measurements

`measurements.rs` defines sample types. All measurements use `Timestamped<T>`:

```rust
pub struct Timestamped<T> {
    pub ts: Instant,
    pub value: T,
}
```

Constructors: `new(ts, value)`, `now(value)`, `now_with_delay(value, delay)`, `at(ts, value)`.

Each sample type wraps a sensor ID + timestamped data:

```rust
pub struct ImuSample {
    pub sensor_id: ImuId,
    pub data: Timestamped<ImuData>,
}
```

## Sensor IDs

`sensors.rs` defines typed sensor IDs via `define_sensor_family!`:

```rust
define_sensor_family! {
    pub struct ImuId;
    count: IMU_COUNT = 2;
    all: IMU_IDS = [IMU_0 = 0, IMU_1 = 1];
}
```

Each ID type has `COUNT`, `index()`, const instances, and an all-IDs array. Used to index into signal channel/watch arrays.

## Storage

`storage.rs` owns the flash filesystem (littlefs2 on W25Q256JV) and the defmt log consumer.

### RpcService for shared filesystem access

```rust
pub static FS: RpcService<CriticalSectionRawMutex, Fs, 256> = RpcService::new();
```

Other tasks access the filesystem via `storage::FS.call(|fs| ...)`. The storage task's main loop uses `select` to interleave RPC servicing and defmt drain:

```rust
loop {
    staging.drain(&fs, &defmt_path, &mut consumer);
    select {
        FS.run(&mut fs),           // service RPC jobs (cancel-safe)
        consumer.wait_for_log(),   // defmt frame arrived
    }
}
```

`FS.run()` is cancel-safe — when a defmt frame arrives, the run future is dropped, the frame is staged, and `run()` is called again next iteration, recovering any in-flight RPC job.

### Defmt logging

The storage task owns the `DefmtConsumer` from `defmt-brtt`. Frames are staged in a 512-byte buffer and flushed to `/log_N/defmt.bin` on the filesystem. No copies through the RpcService — the storage task writes directly with `&mut Filesystem`.

### Log sessions

Each boot creates a new session directory `/log_N/` containing:
- `defmt.bin` — binary defmt frames
- `build_info.txt` — firmware build metadata

The session index is stored in an `AtomicU32`. Other modules use `storage::workdir_path("filename")` to construct paths within the current session.

### Error handling

Mount failures are retried with format. If the filesystem is truly dead, the task enters degraded mode — defmt frames are drained and discarded, logs go to RTT only. The system continues operating. Callers should check `storage::is_available()` before `FS.call()`.

## Executor priorities

| Executor | Priority | Tasks |
|----------|----------|-------|
| `level_0` | Interrupt (P6, TIM2) | All sensor readouts, blinky |
| `level_t` | Thread (lowest) | Storage (blocking flash I/O) |

Sensor readouts are async (DMA-based) and run on the interrupt executor so blocking flash writes on thread mode can't starve them.

## Adding a new board

1. Create `resources/` with pin assignments and peripheral setup
2. Define sensor IDs in `sensors.rs`
3. Define sample types in `measurements.rs`
4. Define signals in `signals.rs`
5. Create readout tasks in `tasks/readout/` following the Inactive/Active pattern
6. Wire everything in `startup.rs`
7. Set up storage if the board has flash
