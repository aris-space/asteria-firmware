#![allow(dead_code)]

use embassy_time::Timer;

#[derive(Debug, Clone, Copy)]
pub struct ExponentialBackoff {
    base_ms: u32,
    max_ms: u32,
}

impl ExponentialBackoff {
    pub fn new(base_ms: u32, max_ms: u32) -> Self {
        Self { base_ms, max_ms }
    }

    pub async fn wait(&self, attempt: u8) {
        let delay = self
            .base_ms
            .saturating_mul(2_u32.pow(u32::from(attempt)))
            .min(self.max_ms);
        Timer::after_millis(u64::from(delay)).await;
    }
}
