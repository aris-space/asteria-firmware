#![allow(unused_macros)]

// Adapted from embassy's `fmt.rs`, but each record goes to BOTH defmt (RTT) and log
// (USB-C), so format strings must suit defmt and core::fmt alike (no `{=type}`).

macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        ::log::trace!($s $(, $x)*);
        ::defmt::trace!($s $(, $x)*);
    }};
}

macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        ::log::debug!($s $(, $x)*);
        ::defmt::debug!($s $(, $x)*);
    }};
}

macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        ::log::info!($s $(, $x)*);
        ::defmt::info!($s $(, $x)*);
    }};
}

macro_rules! warn {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        ::log::warn!($s $(, $x)*);
        ::defmt::warn!($s $(, $x)*);
    }};
}

macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        ::log::error!($s $(, $x)*);
        ::defmt::error!($s $(, $x)*);
    }};
}

// No log "println"; map banner/verdict lines to log::info so they reach USB-C.
macro_rules! println {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        ::log::info!($s $(, $x)*);
        ::defmt::println!($s $(, $x)*);
    }};
}

/// A `Debug` value for both arms above, so one `{}` works: defmt via
/// `defmt::Debug2Format`, core::fmt via its own `Debug`.
pub struct Debug2Format<'a, T: core::fmt::Debug + ?Sized>(pub &'a T);

impl<T: core::fmt::Debug + ?Sized> core::fmt::Debug for Debug2Format<'_, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self.0, f)
    }
}

impl<T: core::fmt::Debug + ?Sized> core::fmt::Display for Debug2Format<'_, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self.0, f)
    }
}

impl<T: core::fmt::Debug + ?Sized> defmt::Format for Debug2Format<'_, T> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{}", defmt::Debug2Format(self.0))
    }
}
