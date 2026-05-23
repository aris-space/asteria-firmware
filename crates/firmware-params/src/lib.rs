#![no_std]

//! A small firmware parameter layer.
//!
//! Firmware code interacts with typed [`Param`] cells. The USB/RPC and
//! flash layer interacts with [`ParamAccess`], which routes reads and
//! writes through a static [`Registry`]. Persistence is a property of a
//! registry entry, not a different parameter type.
//!
//! ```
//! use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
//! use firmware_params::{make_key, Param, ParamEntry, Registry};
//!
//! type Mutex = CriticalSectionRawMutex;
//!
//! static THRESHOLD: Param<Mutex, f32> = Param::new(make_key("v1/sensor/threshold"));
//! static RUN_CAL: Param<Mutex, bool, 1> = Param::new(make_key("v1/cmd/run_cal"));
//!
//! static PARAMS: Registry = Registry::new(&[
//!     ParamEntry::volatile(&THRESHOLD),
//!     ParamEntry::volatile(&RUN_CAL),
//! ]);
//! ```

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::{Receiver, Watch};
use embedded_storage_async::nor_flash::NorFlash;
use postcard::experimental::max_size::MaxSize;
use postcard::{from_bytes, to_slice};
use postcard_schema::{Schema, schema::NamedType};
use sequential_storage::cache::KeyCacheImpl;
use sequential_storage::map::MapStorage;
use serde::{Deserialize, Serialize};

/// Fixed-size wire/storage key for a parameter. ASCII path, zero-padded.
pub type Key = [u8; 32];

/// Default scratch-buffer size used for postcard payloads.
pub const DEFAULT_MAX_PARAM_BYTES: usize = 64;

/// View a [`Key`] as the original path string (everything before the first
/// zero byte). Returns `"<invalid utf-8>"` if the prefix isn't valid UTF-8.
#[must_use]
pub fn key_str(key: &Key) -> &str {
    let len = key.iter().position(|&b| b == 0).unwrap_or(key.len());
    core::str::from_utf8(&key[..len]).unwrap_or("<invalid utf-8>")
}

/// Build a [`Key`] from an ASCII path. Zero-pads to 32 bytes.
///
/// Recommended convention: version prefix, then subsystem, then name —
/// e.g. `"v1/imu/cal"`, `"v1/sensor/threshold"`, `"v1/cmd/run_cal"`.
/// Bumping the prefix (`v1` → `v2`) on a schema change is a clean way
/// to avoid decoding old bytes after a firmware update.
///
/// # Panics
/// At const-eval if `path.len() > 32`.
#[must_use]
pub const fn make_key(path: &str) -> Key {
    let bytes = path.as_bytes();
    assert!(bytes.len() <= 32, "key path exceeds 32 bytes");
    let mut out = [0u8; 32];
    let mut i = 0;
    while i < bytes.len() {
        out[i] = bytes[i];
        i += 1;
    }
    out
}

mod sealed {
    pub trait Sealed {}
}

/// A typed firmware-facing parameter cell.
///
/// `Param` is a key plus a watch channel — no default value baked in.
/// `get()` returns `None` until something is written or loaded from flash;
/// the caller decides what (if anything) goes there.
pub struct Param<M: RawMutex, T: Clone + MaxSize, const WATCHERS: usize = 0> {
    key: Key,
    watch: Watch<M, T, WATCHERS>,
}

impl<M: RawMutex, T: Clone + MaxSize, const WATCHERS: usize> Param<M, T, WATCHERS> {
    /// Create a parameter.
    #[must_use]
    pub const fn new(key: Key) -> Self {
        Self {
            key,
            watch: Watch::new(),
        }
    }

    /// Parameter key.
    #[must_use]
    pub const fn key(&self) -> Key {
        self.key
    }

    /// Current value, or `None` if nothing has been loaded or written yet.
    #[must_use]
    pub fn get(&self) -> Option<T> {
        self.watch.try_get()
    }

    /// Convenience: current value, or `fallback` if uninitialized.
    pub fn get_or(&self, fallback: T) -> T {
        self.watch.try_get().unwrap_or(fallback)
    }

    /// Subscribe to updates.
    ///
    /// Returns `None` if all watcher slots are already taken.
    pub fn subscribe(&self) -> Option<Receiver<'_, M, T, WATCHERS>> {
        self.watch.receiver()
    }

    fn publish(&self, value: T) {
        self.watch.sender().send(value);
    }
}

impl<M: RawMutex, T: Clone + MaxSize, const WATCHERS: usize> sealed::Sealed
    for Param<M, T, WATCHERS>
{
}

mod wire {
    use super::{Key, sealed::Sealed};
    use postcard_schema::schema::NamedType;

    /// Type-erased wire API. Kept inside a private module so external callers
    /// can't bring the trait into scope and call `apply` directly — every cache
    /// update has to go through [`super::ParamAccess`].
    pub trait WireParam: Sealed {
        fn key(&self) -> &Key;
        fn schema(&self) -> &'static NamedType;
        /// `Ok(None)` means the param has no value yet (never loaded, never written).
        fn encode(&self, buf: &mut [u8]) -> Result<Option<usize>, postcard::Error>;
        fn validate(&self, bytes: &[u8]) -> Result<(), postcard::Error>;
        fn apply(&self, bytes: &[u8]) -> Result<(), postcard::Error>;
    }
}

use wire::WireParam;

impl<
    M: RawMutex,
    T: Clone + MaxSize + Serialize + for<'de> Deserialize<'de> + Schema,
    const WATCHERS: usize,
> WireParam for Param<M, T, WATCHERS>
{
    fn key(&self) -> &Key {
        &self.key
    }

    fn schema(&self) -> &'static NamedType {
        T::SCHEMA
    }

    fn encode(&self, buf: &mut [u8]) -> Result<Option<usize>, postcard::Error> {
        match self.watch.try_get() {
            Some(value) => to_slice(&value, buf).map(|bytes| Some(bytes.len())),
            None => Ok(None),
        }
    }

    fn validate(&self, bytes: &[u8]) -> Result<(), postcard::Error> {
        let _: T = from_bytes(bytes)?;
        Ok(())
    }

    fn apply(&self, bytes: &[u8]) -> Result<(), postcard::Error> {
        self.publish(from_bytes(bytes)?);
        Ok(())
    }
}

/// Storage policy for a registered parameter.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum ParamStorage {
    /// Runtime-only value. Writes update RAM but never touch flash.
    Volatile,
    /// Configuration value. Writes persist to flash before updating RAM.
    Persistent,
}

/// Static registration for a parameter.
#[derive(Clone, Copy)]
pub struct ParamEntry {
    storage: ParamStorage,
    param: &'static (dyn WireParam + Sync),
}

impl ParamEntry {
    /// Register a runtime-only parameter.
    #[must_use]
    pub const fn volatile<M, T, const WATCHERS: usize>(
        param: &'static Param<M, T, WATCHERS>,
    ) -> Self
    where
        M: RawMutex,
        T: Clone + MaxSize + Serialize + for<'de> Deserialize<'de> + Schema,
        Param<M, T, WATCHERS>: Sync,
    {
        Self {
            storage: ParamStorage::Volatile,
            param,
        }
    }

    /// Register a flash-backed parameter.
    #[must_use]
    pub const fn persistent<M, T, const WATCHERS: usize>(
        param: &'static Param<M, T, WATCHERS>,
    ) -> Self
    where
        M: RawMutex,
        T: Clone + MaxSize + Serialize + for<'de> Deserialize<'de> + Schema,
        Param<M, T, WATCHERS>: Sync,
    {
        Self {
            storage: ParamStorage::Persistent,
            param,
        }
    }

    #[must_use]
    pub const fn storage(&self) -> ParamStorage {
        self.storage
    }

    #[must_use]
    pub fn key(&self) -> &Key {
        self.param.key()
    }

    #[must_use]
    pub fn schema(&self) -> &'static NamedType {
        self.param.schema()
    }
}

/// Static parameter table.
#[derive(Clone, Copy)]
pub struct Registry {
    entries: &'static [ParamEntry],
}

impl Registry {
    #[must_use]
    pub const fn new(entries: &'static [ParamEntry]) -> Self {
        Self { entries }
    }

    #[must_use]
    pub const fn len(&self) -> usize {
        self.entries.len()
    }

    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn entries(&self) -> impl Iterator<Item = &'static ParamEntry> + '_ {
        self.entries.iter()
    }

    fn find(&self, key: Key) -> Option<&'static ParamEntry> {
        self.entries.iter().find(|entry| *entry.key() == key)
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum ParamReadError {
    UnknownKey,
    /// Param is registered but has no value yet (never loaded, never written).
    NotInitialized,
    /// Encoding the cached value into the caller's buffer failed (typically
    /// because the buffer is smaller than the value's postcard size).
    Encode(postcard::Error),
}

impl From<postcard::Error> for ParamReadError {
    fn from(value: postcard::Error) -> Self {
        Self::Encode(value)
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum ParamWriteError<E> {
    UnknownKey,
    BadPayload(postcard::Error),
    Storage(sequential_storage::Error<E>),
}

impl<E> From<postcard::Error> for ParamWriteError<E> {
    fn from(value: postcard::Error) -> Self {
        Self::BadPayload(value)
    }
}

impl<E> From<sequential_storage::Error<E>> for ParamWriteError<E> {
    fn from(value: sequential_storage::Error<E>) -> Self {
        Self::Storage(value)
    }
}

/// Outcome of loading a single registered entry from flash.
#[derive(Debug)]
pub enum LoadStatus {
    /// Flash had bytes for this key and they decoded into the cache.
    Loaded,
    /// Flash had no entry for this key. Cache stays empty.
    Missing,
    /// Flash had bytes but they didn't decode (likely a schema drift).
    /// Cache stays empty.
    BadDecode(postcard::Error),
}

/// Shared read/write layer for firmware tasks and RPC handlers.
pub struct ParamAccess<
    SM: RawMutex,
    S: NorFlash,
    C: KeyCacheImpl<Key>,
    const MAX_BYTES: usize = DEFAULT_MAX_PARAM_BYTES,
> {
    registry: &'static Registry,
    storage: Mutex<SM, MapStorage<Key, S, C>>,
}

impl<SM: RawMutex, S: NorFlash, C: KeyCacheImpl<Key>, const MAX_BYTES: usize>
    ParamAccess<SM, S, C, MAX_BYTES>
{
    pub fn new(registry: &'static Registry, storage: MapStorage<Key, S, C>) -> Self {
        Self {
            registry,
            storage: Mutex::new(storage),
        }
    }

    /// Typed firmware write.
    ///
    /// Persistent params are written to flash before their RAM cache is
    /// updated. Volatile params update RAM only.
    ///
    /// # Errors
    /// - [`ParamWriteError::UnknownKey`] if `param` isn't in the registry.
    /// - [`ParamWriteError::BadPayload`] if encoding fails.
    /// - [`ParamWriteError::Storage`] if the flash write fails (persistent only).
    #[allow(clippy::future_not_send)]
    pub async fn set<PM, T, const WATCHERS: usize>(
        &self,
        param: &Param<PM, T, WATCHERS>,
        value: T,
    ) -> Result<(), ParamWriteError<S::Error>>
    where
        PM: RawMutex,
        T: Clone + MaxSize + Serialize + for<'de> Deserialize<'de> + Schema,
    {
        const {
            assert!(
                T::POSTCARD_MAX_SIZE <= MAX_BYTES,
                "parameter type's max postcard size exceeds ParamAccess MAX_BYTES",
            );
        }
        let mut bytes = [0u8; MAX_BYTES];
        let len = to_slice(&value, &mut bytes)?.len();
        self.write(param.key(), &bytes[..len]).await
    }

    /// Raw RPC read by key.
    ///
    /// # Errors
    /// - [`ParamReadError::UnknownKey`] if `key` isn't in the registry.
    /// - [`ParamReadError::NotInitialized`] if the param has no value yet.
    /// - [`ParamReadError::Encode`] if `buf` is too small.
    pub fn read(&self, key: Key, buf: &mut [u8]) -> Result<usize, ParamReadError> {
        let entry = self.registry.find(key).ok_or(ParamReadError::UnknownKey)?;
        entry
            .param
            .encode(buf)?
            .ok_or(ParamReadError::NotInitialized)
    }

    /// Raw RPC write by key.
    ///
    /// # Errors
    /// - [`ParamWriteError::UnknownKey`] if `key` isn't in the registry.
    /// - [`ParamWriteError::BadPayload`] if `bytes` don't decode.
    /// - [`ParamWriteError::Storage`] if the flash write fails (persistent only).
    #[allow(clippy::future_not_send)]
    pub async fn write(&self, key: Key, bytes: &[u8]) -> Result<(), ParamWriteError<S::Error>> {
        let entry = self.registry.find(key).ok_or(ParamWriteError::UnknownKey)?;

        entry.param.validate(bytes)?;

        if entry.storage == ParamStorage::Persistent {
            let mut storage = self.storage.lock().await;
            let mut scratch = [0u8; MAX_BYTES];
            storage.store_item(&mut scratch, &key, &bytes).await?;
        }

        Ok(entry.param.apply(bytes)?)
    }

    /// Bring one registered entry to its post-init state.
    ///
    /// - Persistent entries are read from flash and populated into the RAM
    ///   watch. [`LoadStatus::Loaded`] on success, [`LoadStatus::Missing`]
    ///   if flash has no entry for this key, [`LoadStatus::BadDecode`] if
    ///   the bytes are garbage (likely schema drift after a firmware update).
    /// - Volatile entries short-circuit to [`LoadStatus::Loaded`] — nothing
    ///   to load, but uniform with persistent entries so the caller can
    ///   iterate the whole registry without filtering.
    ///
    /// # Errors
    /// Returns the underlying [`sequential_storage::Error`] on flash I/O failure.
    /// Decode failures are reported as [`LoadStatus::BadDecode`], not as errors.
    #[allow(clippy::future_not_send)]
    pub async fn load(
        &self,
        entry: &ParamEntry,
    ) -> Result<LoadStatus, sequential_storage::Error<S::Error>> {
        if entry.storage != ParamStorage::Persistent {
            return Ok(LoadStatus::Loaded);
        }
        let mut scratch = [0u8; MAX_BYTES];
        let len = {
            let mut storage = self.storage.lock().await;
            storage
                .fetch_item(&mut scratch, entry.key())
                .await?
                .map(<[u8]>::len)
        };
        match len {
            None => Ok(LoadStatus::Missing),
            Some(len) => match entry.param.apply(&scratch[..len]) {
                Ok(()) => Ok(LoadStatus::Loaded),
                Err(error) => Ok(LoadStatus::BadDecode(error)),
            },
        }
    }
}
