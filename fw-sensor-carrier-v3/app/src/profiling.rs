//! On-device profiling helpers (timing + stack high-water).
//!
//! Compiled in only with `--features profiling`.
//!
//! - Timing: ARM `DWT.CYCCNT` cycle counter wrapped behind `CycleStats`.
//! - Stack: paints a region of MSP at startup with a sentinel, then scans for
//!   the deepest write to compute the high-water mark.

use core::sync::atomic::{AtomicU32, Ordering};

/// SYSCLK from `shared/stm32h723_clocks.rs` (PLL1_P = 240 MHz).
/// TODO: derive at compile time from a shared const if the clocks config
/// ever exposes one, instead of duplicating the literal.
pub const SYSCLK_HZ: u32 = 240_000_000;

#[inline]
pub fn cycles_to_us(cycles: u32) -> u32 {
    cycles / (SYSCLK_HZ / 1_000_000)
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

/// Min / avg / max over a rolling window of cycle counts.
pub struct CycleStats {
    min: AtomicU32,
    max: AtomicU32,
    sum: AtomicU32,
    count: AtomicU32,
}

impl CycleStats {
    pub const fn new() -> Self {
        Self {
            min: AtomicU32::new(u32::MAX),
            max: AtomicU32::new(0),
            sum: AtomicU32::new(0),
            count: AtomicU32::new(0),
        }
    }

    /// Record a single observation. `cycles` is a delta (not a wall time).
    pub fn observe(&self, cycles: u32) {
        let _ = self
            .min
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |m| {
                Some(m.min(cycles))
            });
        let _ = self
            .max
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |m| {
                Some(m.max(cycles))
            });
        // sum can wrap on long-running windows — TODO bump to u64 if we ever
        // want lifetime stats instead of rolling.
        self.sum.fetch_add(cycles, Ordering::Relaxed);
        self.count.fetch_add(1, Ordering::Relaxed);
    }

    /// Returns `(min, avg, max, count)` and clears the window.
    pub fn snapshot_and_reset(&self) -> (u32, u32, u32, u32) {
        let count = self.count.swap(0, Ordering::Relaxed);
        let sum = self.sum.swap(0, Ordering::Relaxed);
        let min = self.min.swap(u32::MAX, Ordering::Relaxed);
        let max = self.max.swap(0, Ordering::Relaxed);
        let avg = if count == 0 { 0 } else { sum / count };
        (min, avg, max, count)
    }
}

// ---------------------------------------------------------------------------
// Stack painting
// ---------------------------------------------------------------------------

const SENTINEL: u32 = 0xDEAD_BEEF;
/// Paint this many bytes of MSP starting just below the painter's SP. With
/// flip-link the stack lives at the bottom of RAM; this stays well within
/// the stack region for our 320 KiB AXISRAM.
pub const PAINT_DEPTH_BYTES: usize = 8 * 1024;
/// Don't paint within this distance of the painter's own SP — would corrupt
/// the painter's own stack frame.
const SAFETY_MARGIN_BYTES: usize = 512;

/// Lower bound of the painted region. Set by `paint_msp` so `msp_high_water`
/// can scan it later from a different call site (e.g. a periodic task).
static PAINTED_LOW: AtomicU32 = AtomicU32::new(0);
static PAINTED_HIGH: AtomicU32 = AtomicU32::new(0);

/// Paint MSP with a sentinel pattern. Call once, before any meaningful work.
///
/// SAFETY: writes a fixed-size region below the current MSP. The caller must
/// ensure that region is within the stack and not in use by any other
/// execution context. With cortex-m-rt + flip-link and a single-threaded
/// startup phase this holds.
pub unsafe fn paint_msp() {
    let sp = cortex_m::register::msp::read() as usize;
    // Round high boundary down to a word, leaving a safety margin below SP.
    let high = (sp - SAFETY_MARGIN_BYTES) & !3;
    let low = high.saturating_sub(PAINT_DEPTH_BYTES);

    let mut p = low as *mut u32;
    let end = high as *mut u32;
    while p < end {
        unsafe { core::ptr::write_volatile(p, SENTINEL) };
        p = unsafe { p.add(1) };
    }

    PAINTED_LOW.store(low as u32, Ordering::Relaxed);
    PAINTED_HIGH.store(high as u32, Ordering::Relaxed);
}

/// Scan the painted region from the low end and return the number of bytes
/// of stack that have been written since painting (i.e. high-water mark
/// within the painted window).
///
/// Returns `None` if `paint_msp` was never called.
/// Returns `Some(PAINT_DEPTH_BYTES)` if the entire window has been used —
/// the real peak may be deeper than what we painted.
pub fn msp_high_water() -> Option<usize> {
    let low = PAINTED_LOW.load(Ordering::Relaxed);
    let high = PAINTED_HIGH.load(Ordering::Relaxed);
    if low == 0 || high <= low {
        return None;
    }
    let mut p = low as *const u32;
    let end = high as *const u32;
    while p < end {
        // Stack grows down — first word from the low end that isn't the
        // sentinel marks where deeper-than-anywhere-else writes reached.
        let v = unsafe { core::ptr::read_volatile(p) };
        if v != SENTINEL {
            let used = (high as usize) - (p as usize);
            return Some(used);
        }
        p = unsafe { p.add(1) };
    }
    Some(0)
}
