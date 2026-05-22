#![allow(dead_code)]

use core::ops::{Add, Div, Mul, Sub};

use num_traits::Zero;

/// Simple moving average over the last `N` samples.
pub struct MovingAverage<T, const N: usize> {
    buffer: [T; N],
    index: usize,
    sum: T,
    filled: bool,
}

impl<T, const N: usize> MovingAverage<T, N>
where
    T: Copy + Add<Output = T> + Sub<Output = T> + Div<f32, Output = T> + Zero,
{
    pub fn new() -> Self {
        Self {
            buffer: [T::zero(); N],
            index: 0,
            sum: T::zero(),
            filled: false,
        }
    }

    pub fn update(&mut self, sample: T) -> T {
        self.sum = self.sum - self.buffer[self.index];
        self.sum = self.sum + sample;
        self.buffer[self.index] = sample;
        self.index = (self.index + 1) % N;
        if self.index == 0 {
            self.filled = true;
        }
        if self.filled {
            self.sum / N as f32
        } else if self.index > 0 {
            self.sum / self.index as f32
        } else {
            sample
        }
    }
}

/// Exponential moving average with a configurable per-update alpha (for irregular sample rates).
pub struct Ema<T> {
    value: T,
    initialized: bool,
}

impl<T> Ema<T>
where
    T: Copy + Add<Output = T> + Mul<f32, Output = T> + Zero,
{
    pub fn new() -> Self {
        Self {
            value: T::zero(),
            initialized: false,
        }
    }

    pub fn current_value(&self) -> T {
        self.value
    }

    pub fn update_with_alpha(&mut self, sample: T, alpha: f32) -> T {
        if !self.initialized {
            self.value = sample;
            self.initialized = true;
            return sample;
        }
        let one_minus_alpha = 1.0 - alpha;
        self.value = (sample * alpha) + (self.value * one_minus_alpha);
        self.value
    }
}
