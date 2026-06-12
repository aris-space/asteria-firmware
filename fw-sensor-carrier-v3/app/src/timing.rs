//! DWT cycle-counter timing primitives.
//!
//! Used to measure EKF predict/correct step durations, which are logged to the
//! SD card (`predict_us` / `correct_us`). The counter is enabled once at
//! startup via [`init_dwt`].

/// SYSCLK from `shared/stm32h723_clocks.rs` (PLL1_P = 240 MHz).
/// TODO: derive at compile time from a shared const if the clocks config
/// ever exposes one, instead of duplicating the literal.
pub const SYSCLK_HZ: u32 = 240_000_000;

/// Convert a cycle delta to microseconds (fractional).
#[inline]
pub fn cycles_to_us_f32(cycles: u32) -> f32 {
    cycles as f32 / (SYSCLK_HZ as f32 / 1_000_000.0)
}

/// Enable the cycle counter. Must be called once at startup.
pub fn init_dwt() {
    // Steal cortex-m peripherals — embassy_stm32::init only consumes the
    // device peripherals, not the cortex-m core peripherals.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
}

#[inline(always)]
pub fn cycle_count() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}
