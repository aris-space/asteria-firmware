//! Per-sensor calibration. Every sensor kind turns a raw sample into a
//! calibrated one in the same order:
//!
//! 1. time: subtract the stored latency from the readout timestamp,
//! 2. units: convert native counts to physical units,
//! 3. axes: remap the sensor frame onto the board frame,
//! 4. correction: apply the stored per-unit correction.
//!
//! Units and axes are fixed by the chip and the PCB, so they are code. Latency
//! and the correction are measured per board and stored in flash as one
//! [`StoredCal`] per sensor. [`load`] reads them once at boot; a value written
//! from the console applies after a reset.
//!
//! Each `calibration/<kind>.rs` has the same layout: hardware constants and
//! `sensor_to_board` (where the sensor has axes), its `Correction`, the `CAL`
//! table, `apply_calibration`, then the calibration routine if it has one.

use core::fmt;

use defmt::{info, warn};
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::sensors::{BaroId, DhtId, GnssId, ImuId, MagId, SensorId};
use crate::storage::{KEY_LEN, Key, Storage};

pub mod baro;
pub mod dht;
pub mod gnss;
pub mod imu;
pub mod mag;

/// Loads every sensor's calibration from flash. Call once at boot, before
/// any readout runs.
pub async fn load(storage: &Storage) {
    imu::CAL.load(storage).await;
    mag::CAL.load(storage).await;
    gnss::CAL.load(storage).await;
    baro::CAL.load(storage).await;
    dht::CAL.load(storage).await;
}

/// Stores the latency of the sensor named `sensor`, keeping its correction.
/// Returns `None` if no sensor has that name.
pub async fn store_latency(storage: &Storage, sensor: &str, latency_us: i32) -> Option<bool> {
    if let Some(id) = ImuId::from_name(sensor) {
        return Some(imu::CAL.store_latency(storage, id, latency_us).await);
    }
    if let Some(id) = MagId::from_name(sensor) {
        return Some(mag::CAL.store_latency(storage, id, latency_us).await);
    }
    if let Some(id) = GnssId::from_name(sensor) {
        return Some(gnss::CAL.store_latency(storage, id, latency_us).await);
    }
    if let Some(id) = BaroId::from_name(sensor) {
        return Some(baro::CAL.store_latency(storage, id, latency_us).await);
    }
    if let Some(id) = DhtId::from_name(sensor) {
        return Some(dht::CAL.store_latency(storage, id, latency_us).await);
    }
    None
}

/// A sensor kind's per-unit correction, stored inside [`StoredCal`].
pub trait Correction:
    Copy + PartialEq + Serialize + DeserializeOwned + fmt::Display + 'static
{
    const DEFAULT: Self;

    /// Rejects a stored record that decodes but cannot be applied.
    fn is_valid(&self) -> bool {
        true
    }
}

#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct StoredCal<C> {
    pub name: Name,
    /// Time from the physical measurement to the readout timestamp.
    pub latency_us: i32,
    pub correction: C,
}

impl<C: Correction> StoredCal<C> {
    pub const DEFAULT: Self = Self {
        name: Name::new("default"),
        latency_us: 0,
        correction: C::DEFAULT,
    };

    /// The physical measurement time of a sample the readout stamped `ts`.
    pub fn sample_time(&self, ts: Instant) -> Instant {
        let latency = Duration::from_micros(self.latency_us.unsigned_abs().into());
        if self.latency_us >= 0 {
            ts.checked_sub(latency).unwrap_or(ts)
        } else {
            ts + latency
        }
    }
}

impl<C: Correction> fmt::Display for StoredCal<C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if *self == Self::DEFAULT {
            return write!(
                f,
                "\"{}\" \x1b[31m(built-in default, not calibrated)\x1b[0m",
                self.name
            );
        }
        write!(
            f,
            "\"{}\"  latency {} us{}",
            self.name, self.latency_us, self.correction
        )
    }
}

/// The calibrations of one sensor family: written to flash by the console,
/// applied from the copy [`load`](Self::load) read at boot.
pub struct Calibrations<Id, C, const N: usize> {
    ids: [Id; N],
    applied: OnceLock<[StoredCal<C>; N]>,
}

impl<Id: SensorId, C: Correction, const N: usize> Calibrations<Id, C, N> {
    pub const fn new(ids: [Id; N]) -> Self {
        Self {
            ids,
            applied: OnceLock::new(),
        }
    }

    pub fn ids(&self) -> [Id; N] {
        self.ids
    }

    async fn load(&self, storage: &Storage) {
        let mut cals = [StoredCal::DEFAULT; N];
        for (cal, id) in cals.iter_mut().zip(self.ids) {
            match self.stored(storage, id).await {
                Some(stored) => {
                    info!("{}: cal \"{}\" loaded from flash", id, stored.name);
                    *cal = stored;
                }
                None => warn!("{}: no cal in flash, using the default", id),
            }
        }
        let _ = self.applied.init(cals);
    }

    /// The calibration in use since boot.
    pub fn applied(&self, id: Id) -> StoredCal<C> {
        self.applied
            .try_get()
            .map_or(StoredCal::DEFAULT, |cals| cals[id.index()])
    }

    /// The calibration in flash, which a reset would apply.
    pub async fn stored(&self, storage: &Storage, id: Id) -> Option<StoredCal<C>> {
        storage
            .load::<StoredCal<C>>(&key(id))
            .await
            .filter(|cal| cal.correction.is_valid())
    }

    /// Stores a new correction, keeping the sensor's latency.
    pub async fn store_correction(
        &self,
        storage: &Storage,
        id: Id,
        name: Name,
        correction: C,
    ) -> bool {
        let cal = StoredCal {
            name,
            correction,
            ..self.latest(storage, id).await
        };
        storage.store(&key(id), &cal).await
    }

    /// Stores a new latency, keeping the sensor's correction.
    pub async fn store_latency(&self, storage: &Storage, id: Id, latency_us: i32) -> bool {
        let cal = StoredCal {
            latency_us,
            ..self.latest(storage, id).await
        };
        storage.store(&key(id), &cal).await
    }

    async fn latest(&self, storage: &Storage, id: Id) -> StoredCal<C> {
        match self.stored(storage, id).await {
            Some(cal) => cal,
            None => self.applied(id),
        }
    }
}

/// Flash key of a sensor's [`StoredCal`], e.g. `cal:IMU_0`. The prefix keeps
/// records that older firmware stored in another layout from being decoded.
pub fn key(id: impl SensorId) -> Key {
    const PREFIX: &[u8] = b"cal:";
    let name = id.name().as_bytes();
    assert!(PREFIX.len() + name.len() <= KEY_LEN);
    let mut key = [0; KEY_LEN];
    key[..PREFIX.len()].copy_from_slice(PREFIX);
    key[PREFIX.len()..][..name.len()].copy_from_slice(name);
    key
}

/// Fixed-capacity, zero-padded user label persisted with a calibration.
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Name([u8; Name::CAP]);

impl Name {
    pub const CAP: usize = 16;

    pub const fn new(s: &str) -> Self {
        let b = s.as_bytes();
        let mut out = [0u8; Self::CAP];
        let mut i = 0;
        while i < b.len() && i < Self::CAP {
            out[i] = b[i];
            i += 1;
        }
        Self(out)
    }

    pub fn as_str(&self) -> &str {
        let len = self.0.iter().position(|&c| c == 0).unwrap_or(Self::CAP);
        core::str::from_utf8(&self.0[..len]).unwrap_or("?")
    }
}

impl fmt::Display for Name {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl defmt::Format for Name {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", self.as_str());
    }
}
