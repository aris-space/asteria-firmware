// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]

use libm::expf;

pub struct GaussianMovingAverage<const SIZE: usize> {
    window: [f32; SIZE],
    weights: [f32; SIZE],
    head: usize, // ring buffer write index
}

impl<const SIZE: usize> GaussianMovingAverage<SIZE> {
    /// Create Gaussian weights with variance sigma^2 centered at `mean` (in index units).
    ///
    /// Panics if `sigma <= 0` or `SIZE == 0`.
    pub fn new(sigma: f32, mean: f32) -> Self {
        assert!(SIZE > 0, "SIZE must be > 0");
        assert!(sigma > 0.0, "sigma must be > 0");

        let inv_2sigma2 = 1.0 / (2.0 * sigma * sigma);

        let weights = {
            let mut w = [0.0; SIZE];

            let mut sum = 0.0;
            for (i, item) in w.iter_mut().enumerate() {
                let diff = i as f32 - mean;
                *item = expf(-diff * diff * inv_2sigma2);
                sum += *item;
            }
            for item in w.iter_mut() {
                *item /= sum;
            }
            w
        };

        Self {
            window: [0.0; SIZE],
            weights,
            head: 0,
        }
    }

    /// Push a new sample and return the current weighted average.
    pub fn update(&mut self, value: f32) -> f32 {
        // ring buffer: write at head, then advance
        self.window[self.head] = value;
        self.head = (self.head + 1) % SIZE;

        // Compute the weighted average
        let mut weighted_sum = 0.0;
        for i in 0..SIZE {
            // logical oldest..newest mapped via head
            let j = (self.head + i) % SIZE;
            weighted_sum += self.window[j] * self.weights[i];
        }
        weighted_sum
    }
}
