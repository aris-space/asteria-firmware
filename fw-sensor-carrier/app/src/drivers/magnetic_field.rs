#![allow(dead_code)]
use crate::sensors::SensorId;
use core::fmt;
use core::fmt::{Debug, Formatter};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_sync::pubsub::{ImmediatePublisher, PubSubChannel};
use embassy_sync::watch::{Sender, Watch};
use embassy_time::{Duration, Instant};

pub const CAP: usize = 10;
pub const PUB: usize = 0;
pub const SUB: usize = 2;
pub const WATCH: usize = 3;

/// Global watch for magnetic field estimates.
pub static MAGNETIC_FIELD_WATCH: Watch<ThreadModeRawMutex, MagMeasurement, WATCH> = Watch::new();
/// Global pubsub channel for magnetic field estimates.
pub static MAGNETIC_FIELD_PUBSUB: PubSubChannel<ThreadModeRawMutex, MagMeasurement, CAP, SUB, PUB> =
    PubSubChannel::new();

pub static MAGNETIC_FIELD_DRIVER: OnceLock<MagneticFieldDriver> = OnceLock::new();

pub type MagMeasurement = (Instant, MagneticField);

struct Shared {
    timeout_selector: TimeoutSelector,
}

/// Driver for the magnetometer data updates.
pub struct MagneticFieldDriver<'a> {
    publisher: ImmediatePublisher<'a, ThreadModeRawMutex, MagMeasurement, CAP, SUB, PUB>,
    watch: Sender<'a, ThreadModeRawMutex, MagMeasurement, WATCH>,

    shared: Mutex<CriticalSectionRawMutex, Shared>,
}

impl<'a> MagneticFieldDriver<'a> {
    pub fn new(
        publisher: ImmediatePublisher<'a, ThreadModeRawMutex, MagMeasurement, CAP, SUB, PUB>,
        watch: Sender<'a, ThreadModeRawMutex, MagMeasurement, WATCH>,
    ) -> Self {
        const MAG_TIMEOUT: Duration = Duration::from_millis(100);

        Self {
            publisher,
            watch,
            shared: Mutex::new(Shared {
                timeout_selector: TimeoutSelector::new(MAG_TIMEOUT),
            }),
        }
    }

    /// Update the magnetic field estimates.
    ///
    /// # Arguments
    ///
    /// * `x` - The magnetic field measurement on the X-axis (in microtesla).
    /// * `y` - The magnetic field measurement on the Y-axis (in microtesla).
    /// * `z` - The magnetic field measurement on the Z-axis (in microtesla).
    pub async fn update_magnetometer(&self, measurement: MagMeasurement, _sensor_id: SensorId) {
        let (ts, _mag) = measurement;

        // Only accept from the current primary or if the primary has timed out.
        let accept = {
            let mut shared = self.shared.lock().await;
            shared.timeout_selector.accept(_sensor_id, ts)
        };

        if !accept {
            return;
        }

        self.publisher.publish_immediate(measurement);
        self.watch.send(measurement);
    }
}

#[derive(Default, Clone, Copy, PartialEq)]
pub struct MagneticField {
    pub x: u16,
    pub y: u16,
    pub z: u16,
}

impl Debug for MagneticField {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "MagneticField {{ x: {}, y: {}, z: {} }}",
            self.x_nt(),
            self.y_nt(),
            self.z_nt()
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for MagneticField {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "MagneticField {{ x: {}, y: {}, z: {} }}",
            self.x_nt(),
            self.y_nt(),
            self.z_nt()
        );
    }
}

// This is just a reimplementation of the MagneticField struct from the LSM303AGR driver, because
// it does not support some basic operations.
impl MagneticField {
    const SCALING_FACTOR: i32 = 150;

    /// Creates a new `MagneticField` instance.
    #[inline]
    pub const fn new(x: u16, y: u16, z: u16) -> Self {
        Self { x, y, z }
    }

    /// Raw magnetic field in X-direction.
    #[inline]
    pub const fn x_raw(&self) -> u16 {
        self.x
    }

    /// Raw magnetic field in Y-direction.
    #[inline]
    pub const fn y_raw(&self) -> u16 {
        self.y
    }

    /// Raw magnetic field in Z-direction.
    #[inline]
    pub const fn z_raw(&self) -> u16 {
        self.z
    }

    /// Raw magnetic field in X-, Y- and Z-directions.
    #[inline]
    pub const fn xyz_raw(&self) -> (u16, u16, u16) {
        (self.x, self.y, self.z)
    }

    /// Unscaled magnetic field in X-direction.
    #[inline]
    pub const fn x_unscaled(&self) -> i16 {
        self.x as i16
    }

    /// Unscaled magnetic field in Y-direction.
    #[inline]
    pub const fn y_unscaled(&self) -> i16 {
        self.y as i16
    }

    /// Unscaled magnetic field in Z-direction.
    #[inline]
    pub const fn z_unscaled(&self) -> i16 {
        self.z as i16
    }

    /// Unscaled magnetic field in X-, Y- and Z-directions.
    #[inline]
    pub const fn xyz_unscaled(&self) -> (i16, i16, i16) {
        (self.x as i16, self.y as i16, self.z as i16)
    }

    /// Magnetic field in X-direction in nT (nano-Tesla).
    #[inline]
    pub const fn x_nt(&self) -> i32 {
        (self.x_unscaled() as i32) * Self::SCALING_FACTOR
    }

    /// Magnetic field in Y-direction in nT (nano-Tesla).
    #[inline]
    pub const fn y_nt(&self) -> i32 {
        (self.y_unscaled() as i32) * Self::SCALING_FACTOR
    }

    /// Magnetic field in Z-direction in nT (nano-Tesla).
    #[inline]
    pub const fn z_nt(&self) -> i32 {
        (self.z_unscaled() as i32) * Self::SCALING_FACTOR
    }

    /// Magnetic field in X-, Y- and Z-directions in nT (nano-Tesla).
    #[inline]
    pub const fn xyz_nt(&self) -> (i32, i32, i32) {
        (self.x_nt(), self.y_nt(), self.z_nt())
    }
}

/// Keep a primary source, but swap on timeout or if the current message comes from the primary.
struct TimeoutSelector {
    primary: SensorId,
    last_update: Instant,
    timeout: Duration,
}

impl TimeoutSelector {
    fn new(timeout: Duration) -> Self {
        Self {
            primary: SensorId::MagnetometerBus1, // will be overwritten by first accepted mag id
            last_update: Instant::now(),
            timeout,
        }
    }

    fn accept(&mut self, id: SensorId, ts: Instant) -> bool {
        if self.primary == id || self.last_update.elapsed() > self.timeout {
            self.primary = id;
            self.last_update = ts;
            true
        } else {
            false
        }
    }
}
