#![no_std]

//! Selection policy shared by estimator banks.

mod gnss;

pub use gnss::{
    DualGnssSelector, GNSS_RECEIVER_COUNT, GnssSample, GnssSelectorConfig, GnssSolution,
    GnssSource, SelectedGnss,
};

/// Switching policy for a bank of scored estimator candidates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SelectorConfig {
    /// Required score advantage before changing the selected estimator.
    switch_hysteresis: f32,
    /// Required duration of a score advantage and minimum time between optional changes.
    ///
    /// An invalid selected estimator is replaced immediately.
    minimum_dwell_time_us: u64,
}

impl SelectorConfig {
    /// Creates a validated switching policy.
    ///
    /// Returns `None` when `switch_hysteresis` is negative or non-finite.
    #[must_use]
    pub fn new(switch_hysteresis: f32, minimum_dwell_time_us: u64) -> Option<Self> {
        (switch_hysteresis.is_finite() && switch_hysteresis >= 0.0).then_some(Self {
            switch_hysteresis,
            minimum_dwell_time_us,
        })
    }
}

/// One estimator considered by [`HysteresisSelector`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Candidate {
    /// Stable identity used to retrieve the estimator output.
    pub index: usize,
    /// Consistency score. Lower is better.
    pub score: f32,
    /// Whether the estimator is currently eligible for selection.
    pub valid: bool,
}

/// Chooses the lowest-scoring valid estimator without rapidly switching near ties.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HysteresisSelector {
    config: SelectorConfig,
    selected: usize,
    last_switch_time_us: Option<u64>,
    challenger: Option<(usize, u64)>,
}

impl HysteresisSelector {
    /// Creates a selector with an initial candidate.
    #[must_use]
    pub const fn new(config: SelectorConfig, initially_selected: usize) -> Self {
        Self {
            config,
            selected: initially_selected,
            last_switch_time_us: None,
            challenger: None,
        }
    }

    /// Returns the currently selected candidate index.
    #[must_use]
    pub const fn selected(&self) -> usize {
        self.selected
    }

    /// Reconsiders the selection and returns the selected candidate index.
    ///
    /// Invalid candidates and candidates with non-finite scores are ignored. If the selected
    /// candidate becomes invalid, the best valid candidate replaces it immediately. Otherwise a
    /// challenger must beat it by the configured hysteresis for the dwell time. The same dwell
    /// time also separates optional switches.
    pub fn select(
        &mut self,
        time_us: u64,
        candidates: impl IntoIterator<Item = Candidate>,
    ) -> usize {
        let mut best = None;
        let mut selected_score = None;

        for candidate in candidates {
            if !candidate.valid || !candidate.score.is_finite() {
                continue;
            }
            if candidate.index == self.selected {
                selected_score = Some(candidate.score);
            }
            if best
                .is_none_or(|current: Candidate| candidate.score.total_cmp(&current.score).is_lt())
            {
                best = Some(candidate);
            }
        }

        let Some(best) = best else {
            self.challenger = None;
            return self.selected;
        };
        if best.index == self.selected {
            self.challenger = None;
            return self.selected;
        }
        let Some(selected_score) = selected_score else {
            self.selected = best.index;
            self.last_switch_time_us = Some(time_us);
            self.challenger = None;
            return self.selected;
        };
        if best.score + self.config.switch_hysteresis >= selected_score {
            self.challenger = None;
            return self.selected;
        }
        let since = match self.challenger {
            Some((index, since)) if index == best.index => since,
            _ => time_us,
        };
        self.challenger = Some((best.index, since));
        let advantage_persisted =
            time_us.saturating_sub(since) >= self.config.minimum_dwell_time_us;
        let dwell_elapsed = self.last_switch_time_us.is_none_or(|last_switch_time_us| {
            time_us.saturating_sub(last_switch_time_us) >= self.config.minimum_dwell_time_us
        });
        if advantage_persisted && dwell_elapsed {
            self.selected = best.index;
            self.last_switch_time_us = Some(time_us);
            self.challenger = None;
        }
        self.selected
    }
}

#[cfg(test)]
mod tests {
    use super::{Candidate, HysteresisSelector, SelectorConfig};

    fn candidates(first_score: f32, second_score: f32) -> [Candidate; 2] {
        [
            Candidate {
                index: 0,
                score: first_score,
                valid: true,
            },
            Candidate {
                index: 1,
                score: second_score,
                valid: true,
            },
        ]
    }

    #[test]
    fn short_score_excursion_does_not_switch() {
        let config = SelectorConfig::new(0.1, 1_000_000).unwrap();
        let mut selector = HysteresisSelector::new(config, 0);

        assert_eq!(selector.select(0, candidates(1.0, 0.0)), 0);
        assert_eq!(selector.select(500_000, candidates(1.0, 0.0)), 0);
        assert_eq!(selector.select(750_000, candidates(0.0, 1.0)), 0);
        assert_eq!(selector.select(1_500_000, candidates(1.0, 0.0)), 0);
        assert_eq!(selector.select(2_400_000, candidates(1.0, 0.0)), 0);
        assert_eq!(selector.select(2_500_000, candidates(1.0, 0.0)), 1);
        assert_eq!(selector.select(2_600_000, candidates(0.0, 1.0)), 1);
        assert_eq!(selector.select(3_500_000, candidates(0.0, 1.0)), 1);
        assert_eq!(selector.select(3_600_000, candidates(0.0, 1.0)), 0);
    }

    #[test]
    fn invalid_selected_candidate_fails_over_immediately() {
        let config = SelectorConfig::new(0.1, 1_000_000).unwrap();
        let mut selector = HysteresisSelector::new(config, 0);
        let [mut first, second] = candidates(1.0, 0.0);
        first.valid = false;

        assert_eq!(selector.select(0, [first, second]), 1);
    }
}
