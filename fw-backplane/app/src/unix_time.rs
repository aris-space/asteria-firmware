#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_time::{Duration, Instant};

/// Reference point that ties a monotonic [`Instant`] to absolute Unix time.
///
/// * `base_micros` – microseconds since 1970-01-01 00:00:00 UTC measured
///   at `instant`.
/// * `instant`     – the monotonic instant captured at the same moment.
struct ClockBase {
    base_micros: u64,
    instant: Instant,
}

static CLOCK_INITIALISED_START: AtomicBool = AtomicBool::new(false);
static CLOCK_INITIALISED_FINISHED: AtomicBool = AtomicBool::new(false);
static mut CLOCK_BASE: ClockBase = ClockBase {
    base_micros: 0,
    instant: Instant::from_ticks(0),
};

/// Initialise the global UTC clock.
///
/// # Arguments
/// * `base_micros` – absolute time (µs since Unix epoch).
/// * `instant`- obtained at the same moment.
///
/// The routine is idempotent: subsequent calls are ignored.
/// Two atomics are used so that readers never observe a half-written
/// [`ClockBase`].
pub fn init_utc_clock(base_micros: u64, instant: Instant) {
    if CLOCK_INITIALISED_START.swap(true, Ordering::SeqCst) {
        return; // Clock already initialised.
    }
    unsafe {
        CLOCK_BASE.base_micros = base_micros;
        CLOCK_BASE.instant = instant;
    }
    CLOCK_INITIALISED_FINISHED.store(true, Ordering::SeqCst);
}

/// Microseconds since the Unix epoch.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
pub struct UnixTime {
    micros: u64,
}

impl UnixTime {
    /// Returns `true` if the clock has been initialised.
    #[inline]
    pub fn is_initialised() -> bool {
        CLOCK_INITIALISED_FINISHED.load(Ordering::SeqCst)
    }

    /// Current UTC time, or `None` if `init_utc_clock` has not run yet.
    pub fn now() -> Option<Self> {
        if !Self::is_initialised() {
            return None;
        }

        let now = Instant::now();
        // Safe because the writer set FINISHED first; SeqCst establishes
        // the necessary ordering.
        let elapsed_micros = now
            .duration_since(unsafe { CLOCK_BASE.instant })
            .as_micros();
        let base_micros = unsafe { CLOCK_BASE.base_micros };
        Some(UnixTime {
            micros: base_micros + elapsed_micros,
        })
    }

    /// Saturating difference between two timestamps.
    #[inline]
    pub const fn saturating_duration_since(&self, earlier: &UnixTime) -> Duration {
        Duration::from_micros(self.micros.saturating_sub(earlier.micros))
    }

    /// Raw microseconds since epoch.
    #[inline]
    pub const fn as_micros(&self) -> u64 {
        self.micros
    }

    /// Milliseconds since epoch.
    #[inline]
    pub const fn as_millis(&self) -> u64 {
        self.micros / 1_000
    }

    /// Seconds since epoch.
    #[inline]
    pub const fn as_secs(&self) -> u64 {
        self.micros / 1_000_000
    }

    /// Convert to [`Duration`].
    #[inline]
    pub const fn as_duration(&self) -> Duration {
        Duration::from_micros(self.micros)
    }

    /// True if `self` is strictly later than `other`.
    #[inline]
    pub const fn is_after(&self, other: &Self) -> bool {
        self.micros > other.micros
    }

    /// Time elapsed between the current time and `self`.
    pub fn elapsed(&self) -> Duration {
        UnixTime::now()
            .map(|now| now.saturating_duration_since(self))
            .unwrap_or(Duration::from_micros(0))
    }
}
