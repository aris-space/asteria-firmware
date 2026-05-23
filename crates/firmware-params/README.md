# firmware-params

Small `no_std` parameter layer for firmware that needs one abstraction for:

- typed firmware reads and writes
- USB/RPC reads and writes by key
- flash-backed configuration values
- volatile command-like parameters, such as `RUN_CAL = true`

`Param<T>` is just a typed storage cell. Persistence is declared in the static registry. Keys are 32-byte ASCII paths — recommended convention is a version prefix (`v1/...`) so schema changes can bump to `v2/...` without colliding with old flash entries.

```rust
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use firmware_params::{make_key, Param, ParamEntry, Registry};

type Mutex = CriticalSectionRawMutex;

static THRESHOLD: Param<Mutex, f32> = Param::new(make_key("v1/sensor/threshold"));
static RUN_CAL: Param<Mutex, bool, 1> = Param::new(make_key("v1/cmd/run_cal"));

static PARAMS: Registry = Registry::new(&[
    ParamEntry::volatile(&THRESHOLD),
    ParamEntry::volatile(&RUN_CAL),
]);
```

Firmware and RPC both go through `ParamAccess`:

```rust,no_run
# use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
# use firmware_params::{make_key, Key, LoadStatus, Param, ParamAccess, ParamEntry, ParamStorage, Registry};
# use sequential_storage::cache::NoCache;
# use sequential_storage::map::MapStorage;
# use embedded_storage_async::nor_flash::NorFlash;
# type Mutex = CriticalSectionRawMutex;
# static THRESHOLD: Param<Mutex, f32> = Param::new(make_key("v1/sensor/threshold"));
# static PARAMS: Registry = Registry::new(&[ParamEntry::volatile(&THRESHOLD)]);
# async fn example<S: NorFlash>(storage: MapStorage<Key, S, NoCache>) -> Result<(), firmware_params::ParamWriteError<S::Error>> {
let params: ParamAccess<Mutex, _, _> = ParamAccess::new(&PARAMS, storage);

// At boot: walk persistent entries and decide per key.
for entry in PARAMS.entries().filter(|e| e.storage() == ParamStorage::Persistent) {
    match params.load(entry).await.unwrap() {
        LoadStatus::Loaded => {}
        LoadStatus::Missing => { /* first boot; write a sensible default if you want */ }
        LoadStatus::BadDecode(_) => { /* schema drift; log + maybe wipe */ }
    }
}

params.set(&THRESHOLD, 42.0).await?;
let value: Option<f32> = THRESHOLD.get();
# let _ = value;
# Ok(())
# }
```

Persistent parameters are written to flash before the RAM watch is updated. Volatile parameters only update RAM. A command is just a volatile parameter with task code subscribed to it:

```rust,no_run
# use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
# use embedded_storage_async::nor_flash::NorFlash;
# use firmware_params::{make_key, Key, Param, ParamAccess};
# use sequential_storage::cache::KeyCacheImpl;
# type Mutex = CriticalSectionRawMutex;
# static RUN_CAL: Param<Mutex, bool, 1> = Param::new(make_key("v1/cmd/run_cal"));
async fn calibration_task<SM, S, C>(params: &ParamAccess<SM, S, C>)
where
    SM: RawMutex,
    S: NorFlash,
    C: KeyCacheImpl<Key>,
{
    let Some(mut requests) = RUN_CAL.subscribe() else {
        return;
    };

    loop {
        if requests.changed().await {
            // run calibration here
            let _ = params.set(&RUN_CAL, false).await;
        }
    }
}
```

Run the full host demo:

```sh
cargo run -p firmware-params --example command_trigger
```
