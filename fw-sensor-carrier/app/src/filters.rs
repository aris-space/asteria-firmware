#![allow(dead_code)]
use core::ops::{Add, Div, Mul, Sub};

/// Defines the core behavior for all filters.
pub trait Filter {
    /// The type of sample this filter processes.
    type Sample;

    /// Process a new sample and return the filtered output.
    fn update(&mut self, sample: Self::Sample) -> Self::Sample;
}

/// A simple moving average filter using a circular buffer.
///
/// Calculates the average of the N most recent samples.
#[derive(Debug, Clone)]
pub struct MovingAverage<T, const N: usize> {
    buffer: [T; N],
    index: usize,
    sum: T,
    filled: bool,
}

impl<T, const N: usize> MovingAverage<T, N>
where
    T: Copy + Add<Output = T> + Sub<Output = T> + Div<f32, Output = T> + num_traits::Zero,
{
    /// Creates a new `SimpleMovingAverage` filter with all samples initialized to zero.
    pub fn new() -> Self {
        Self {
            buffer: [T::zero(); N],
            index: 0,
            sum: T::zero(),
            filled: false,
        }
    }

    /// Creates a new `SimpleMovingAverage` filter with all samples initialized to the given value.
    pub fn with_initial_value(value: T, n_value: T) -> Self
    where
        T: Mul<Output = T>,
    {
        Self {
            buffer: [value; N],
            index: 0,
            sum: value * n_value,
            filled: true,
        }
    }

    /// Returns the current average value.
    pub fn current_value(&self) -> T {
        if self.filled {
            self.sum / N as f32
        } else if self.index > 0 {
            self.sum / self.index as f32
        } else {
            T::zero()
        }
    }
}

impl<T, const N: usize> Filter for MovingAverage<T, N>
where
    T: Copy + Add<Output = T> + Sub<Output = T> + Div<f32, Output = T>,
{
    type Sample = T;

    fn update(&mut self, sample: Self::Sample) -> Self::Sample {
        // Subtract the oldest sample from the sum
        self.sum = self.sum - self.buffer[self.index];

        // Add the new sample to the sum and store it in the buffer
        self.sum = self.sum + sample;
        self.buffer[self.index] = sample;

        // Update the index for the next update
        self.index = (self.index + 1) % N;

        // Update filled flag if we've gone around the buffer once
        if self.index == 0 {
            self.filled = true;
        }

        // Calculate the average
        if self.filled {
            self.sum / N as f32
        } else if self.index > 0 {
            self.sum / self.index as f32
        } else {
            sample
        }
    }
}

/// An exponential moving average filter.
///
/// Applies exponential weighting to new samples with configurable alpha.
#[derive(Debug, Clone)]
pub struct ExponentialMovingAverage<T> {
    alpha: f32,
    one_minus_alpha: f32,
    value: T,
    initialized: bool,
}

impl<T> ExponentialMovingAverage<T>
where
    T: Copy
        + Add<Output = T>
        + Mul<f32, Output = T>
        + Sub<Output = T>
        + num_traits::Zero
        + PartialOrd,
{
    /// Creates a new `ExponentialMovingAverage` filter with the given alpha value.
    ///
    /// Alpha should be between 0 and 1, where:
    /// - Values closer to 0 establish stronger smoothing effect (slower response)
    /// - Values closer to 1 track input changes more closely (faster response)
    pub fn new(alpha: f32) -> Self {
        debug_assert!(
            (0.0..=1.0).contains(&alpha),
            "Alpha must be between 0 and 1"
        );
        Self {
            alpha,
            one_minus_alpha: 1.0 - alpha,
            value: T::zero(),
            initialized: false,
        }
    }

    /// Creates a new `ExponentialMovingAverage` with an initial value.
    ///
    /// Alpha should be between 0 and 1, where:
    /// - Values closer to 0 establish stronger smoothing effect (slower response)
    /// - Values closer to 1 track input changes more closely (faster response)
    pub fn with_initial_value(alpha: f32, initial: T) -> Self {
        debug_assert!(
            (0.0..=1.0).contains(&alpha),
            "Alpha must be between 0 and 1"
        );
        Self {
            alpha,
            one_minus_alpha: 1.0 - alpha,
            value: initial,
            initialized: true,
        }
    }

    pub fn current_value(&self) -> T {
        self.value
    }

    /// Updates the filter with a new sample and a custom alpha.
    ///
    /// Use this if your update rate varies and you want to control smoothing dynamically.
    pub fn update_with_alpha(&mut self, sample: T, alpha: f32) -> T
    where
        T: Mul<f32, Output = T> + Add<Output = T>,
    {
        debug_assert!(
            (0.0..=1.0).contains(&alpha),
            "Alpha must be between 0 and 1"
        );

        if !self.initialized {
            self.value = sample;
            self.initialized = true;
            return sample;
        }

        // EMA formula: value = alpha * sample + (1 - alpha) * value
        let one_minus_alpha = 1.0 - alpha;
        self.value = (sample * alpha) + (self.value * one_minus_alpha);
        self.value
    }
}

impl<T> Filter for ExponentialMovingAverage<T>
where
    T: Copy + Add<Output = T> + Mul<f32, Output = T>,
{
    type Sample = T;

    fn update(&mut self, sample: Self::Sample) -> Self::Sample {
        if !self.initialized {
            self.value = sample;
            self.initialized = true;
            return sample;
        }

        // EMA formula: value = alpha * sample + (1 - alpha) * value
        self.value = (sample * self.alpha) + (self.value * self.one_minus_alpha);
        self.value
    }
}
