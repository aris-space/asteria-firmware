//! Per-sensor calibration. Every sensor kind turns a raw sample into a
//! calibrated one in the same order:
//!
//! 1. time: subtract the stored latency from the readout timestamp,
//! 2. units: convert native counts to physical units,
//! 3. axes: remap the sensor frame onto the board frame,
//! 4. correction: apply the stored per-unit correction.
//!
//! Units and axes are fixed by the chip and the PCB, so they are code. Latency
//! and the correction are fitted offline from SD logs and stored in flash as
//! one [`StoredCal`] per sensor, written with the console's `cal set` line in
//! the same `key=value` form `cal show` prints. [`load`] reads them once at
//! boot, so a new value applies after a reset.
//!
//! Each `calibration/<kind>.rs` has the same layout: hardware constants and
//! `sensor_to_board` (where the sensor has axes), its `Correction`, the `CAL`
//! table, then `apply_calibration`.

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

/// Parses and stores the calibration of the sensor named `sensor` from
/// `name=… latency_us=… <correction fields>`.
pub async fn set<'a>(
    storage: &Storage,
    sensor: &str,
    fields: impl IntoIterator<Item = &'a str>,
) -> Result<(), &'static str> {
    if let Some(id) = ImuId::from_name(sensor) {
        return imu::CAL.store(storage, id, fields).await;
    }
    if let Some(id) = MagId::from_name(sensor) {
        return mag::CAL.store(storage, id, fields).await;
    }
    if let Some(id) = GnssId::from_name(sensor) {
        return gnss::CAL.store(storage, id, fields).await;
    }
    if let Some(id) = BaroId::from_name(sensor) {
        return baro::CAL.store(storage, id, fields).await;
    }
    if let Some(id) = DhtId::from_name(sensor) {
        return dht::CAL.store(storage, id, fields).await;
    }
    Err("unknown sensor")
}

/// A sensor kind's per-unit correction, stored inside [`StoredCal`].
pub trait Correction:
    Copy + PartialEq + Serialize + DeserializeOwned + fmt::Display + 'static
{
    const DEFAULT: Self;
    /// The `key`s of [`set`](Self::set); a `cal set` line must give all of them.
    const FIELDS: &'static [&'static str];

    /// Sets one field from its console text; `false` if the value does not parse.
    fn set(&mut self, key: &str, value: &str) -> bool;

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

    /// Parses `name=… latency_us=…` and every correction field. A missing,
    /// unknown, or unparsable field rejects the whole line.
    pub fn parse<'a>(fields: impl IntoIterator<Item = &'a str>) -> Result<Self, &'static str> {
        let mut cal = Self::DEFAULT;
        let mut seen = 0u32;
        for field in fields {
            let (key, value) = field.split_once('=').ok_or("expected key=value")?;
            let bit = match key {
                "name" if value.len() <= Name::CAP => {
                    cal.name = Name::new(value);
                    0
                }
                "name" => return Err("name too long"),
                "latency_us" => {
                    cal.latency_us = value.parse().map_err(|_| "latency_us is not an integer")?;
                    1
                }
                _ => {
                    let index = C::FIELDS
                        .iter()
                        .position(|&known| known == key)
                        .ok_or("unknown field")?;
                    if !cal.correction.set(key, value) {
                        return Err("malformed value");
                    }
                    2 + index
                }
            };
            seen |= 1 << bit;
        }
        if seen != (1 << (2 + C::FIELDS.len())) - 1 {
            return Err("missing fields");
        }
        if !cal.correction.is_valid() {
            return Err("values out of range");
        }
        Ok(cal)
    }

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

/// The `cal set` fields, so `cal show` output can be pasted back.
impl<C: Correction> fmt::Display for StoredCal<C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "name={} latency_us={}{}",
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

    /// Stores the calibration parsed from a `cal set` line.
    pub async fn store<'a>(
        &self,
        storage: &Storage,
        id: Id,
        fields: impl IntoIterator<Item = &'a str>,
    ) -> Result<(), &'static str> {
        let cal = StoredCal::<C>::parse(fields)?;
        if storage.store(&key(id), &cal).await {
            Ok(())
        } else {
            Err("flash write failed")
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

/// Comma-separated floats, as `cal set` takes them.
pub struct Floats<'a>(pub &'a [f32]);

impl fmt::Display for Floats<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, value) in self.0.iter().enumerate() {
            if i > 0 {
                f.write_str(",")?;
            }
            write!(f, "{value}")?;
        }
        Ok(())
    }
}

/// Exactly `N` comma-separated floats.
pub fn parse_floats<const N: usize>(text: &str) -> Option<[f32; N]> {
    let mut values = [0.0; N];
    let mut parts = text.split(',');
    for value in &mut values {
        *value = parts.next()?.parse().ok()?;
    }
    parts.next().is_none().then_some(values)
}
