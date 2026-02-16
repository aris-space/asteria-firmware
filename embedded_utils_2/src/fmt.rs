#![allow(unused)]
use core::sync::atomic::AtomicU32;
#[macro_export]
macro_rules! assert {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::assert!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::assert!($($x)*);
        }
    };
}

#[macro_export]
macro_rules! assert_eq {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::assert_eq!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::assert_eq!($($x)*);
        }
    };
}

#[macro_export]
macro_rules! assert_ne {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::assert_ne!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::assert_ne!($($x)*);
        }
    };
}

#[macro_export]
macro_rules! debug_assert {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::debug_assert!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::debug_assert!($($x)*);
        }
    };
}

#[macro_export]
macro_rules! debug_assert_eq {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::debug_assert_eq!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::debug_assert_eq!($($x)*);
        }
    };
}

#[macro_export]
macro_rules! debug_assert_ne {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::debug_assert_ne!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::debug_assert_ne!($($x)*);
        }
    };
}

#[macro_export]
macro_rules! todo {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::todo!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::todo!($($x)*);
        }
    };
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! unreachable {
    ($($x:tt)*) => {
        ::core::unreachable!($($x)*)
    };
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! unreachable {
    ($($x:tt)*) => {
        ::defmt::unreachable!($($x)*)
    };
}

#[macro_export]
macro_rules! panic {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::panic!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::panic!($($x)*);
        }
    };
}

#[macro_export]
macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::trace!($s $(, $x)*);
            #[cfg(not(feature="defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[macro_export]
macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::debug!($s $(, $x)*);
            #[cfg(not(feature="defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[macro_export]
macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::info!($s $(, $x)*);
            #[cfg(not(feature="defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[macro_export]
macro_rules! _warn {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::warn!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( &$x ),*);

            let _ = ::core::sync::atomic::AtomicU32::fetch_update(
                &$crate::WARN_COUNT,
                ::core::sync::atomic::Ordering::Relaxed,
                ::core::sync::atomic::Ordering::Relaxed,
                |val| val.checked_add(1),
            );
        }
    };
}

#[macro_export]
macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::error!($s $(, $x)*);
            #[cfg(not(feature="defmt"))]
            let _ = ($( & $x ),*);

            let _ = ::core::sync::atomic::AtomicU32::fetch_update(
                &$crate::ERROR_COUNT,
                ::core::sync::atomic::Ordering::Relaxed,
                ::core::sync::atomic::Ordering::Relaxed,
                |val| val.checked_add(1),
            );
        }
    };
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! unwrap {
    ($($x:tt)*) => {
        ::defmt::unwrap!($($x)*)
    };
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! unwrap {
    ($arg:expr) => {
        match $crate::fmt::Try::into_result($arg) {
            ::core::result::Result::Ok(t) => t,
            ::core::result::Result::Err(_) => {
                ::core::panic!();
            }
        }
    };
    ($arg:expr, $($msg:expr),+ $(,)? ) => {
        match $crate::fmt::Try::into_result($arg) {
            ::core::result::Result::Ok(t) => t,
            ::core::result::Result::Err(_) => {
                ::core::panic!();
            }
        }
    };
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct NoneError;

pub trait Try {
    type Ok;
    type Error;
    fn into_result(self) -> Result<Self::Ok, Self::Error>;
}

impl<T> Try for Option<T> {
    type Ok = T;
    type Error = NoneError;

    #[inline]
    fn into_result(self) -> Result<T, NoneError> {
        self.ok_or(NoneError)
    }
}

impl<T, E> Try for Result<T, E> {
    type Ok = T;
    type Error = E;

    #[inline]
    fn into_result(self) -> Self {
        self
    }
}

pub(crate) struct Bytes<'a>(pub &'a [u8]);

#[cfg(feature = "defmt")]
impl defmt::Format for Bytes<'_> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{:02x}", self.0)
    }
}

pub use _warn as warn;
pub use assert;
pub use assert_eq;
pub use assert_ne;
pub use debug;
pub use debug_assert;
pub use debug_assert_eq;
pub use debug_assert_ne;
pub use error;
pub use info;
pub use panic;
pub use todo;
pub use trace;
pub use unreachable;
pub use unwrap;
