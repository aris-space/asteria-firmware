//! Selection policies for estimator chains and GNSS receivers.

/// Number of GNSS receivers handled by the selector.
pub const GNSS_RECEIVER_COUNT: usize = 2;

const RECEIVER_0: usize = 0;
const RECEIVER_1: usize = 1;

/// Validation required before considering a receiver's GNSS solution.
pub trait GnssSolution: Copy {
    /// Returns whether the measurement and its uncertainty are usable.
    fn is_valid(&self) -> bool;
}

/// One receiver's GNSS solution and quality metadata.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssSample<M> {
    /// Position and velocity measurement with uncertainty.
    pub measurement: M,
    /// Fix quality tier where `3` conventionally means a usable 3D fix.
    pub fix_tier: u8,
    /// Position dilution of precision scaled by 100. Zero means unavailable.
    pub pdop_centi: u16,
}

/// Origin of a selected GNSS solution.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GnssSource {
    /// Receiver zero.
    Receiver0,
    /// Receiver one.
    Receiver1,
}

/// GNSS solution selected for fusion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SelectedGnss<M> {
    /// Observation to fuse.
    pub measurement: M,
    /// Receiver decision used to produce it.
    pub source: GnssSource,
}

/// Quality and switching policy for two GNSS receivers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssSelectorConfig {
    /// Minimum accepted fix tier.
    minimum_fix_tier: u8,
    /// Minimum measurement-time interval before switching between valid receivers.
    minimum_switch_dwell_us: u64,
}

impl GnssSelectorConfig {
    /// Creates a dual-receiver selection policy.
    #[must_use]
    pub const fn new(minimum_fix_tier: u8, minimum_switch_dwell_us: u64) -> Self {
        Self {
            minimum_fix_tier,
            minimum_switch_dwell_us,
        }
    }
}

/// Chooses one valid GNSS receiver by fix tier, then PDOP, with switch dwell.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DualGnssSelector {
    config: GnssSelectorConfig,
    primary: usize,
    last_switch_time_us: Option<u64>,
}

impl DualGnssSelector {
    /// Creates a selector preferring receiver zero when quality is otherwise equal.
    #[must_use]
    pub const fn new(config: GnssSelectorConfig) -> Self {
        Self {
            config,
            primary: RECEIVER_0,
            last_switch_time_us: None,
        }
    }

    /// Selects one unmodified solution from two receiver candidates for one measurement time.
    ///
    /// `timestamp_us` is the compensated measurement time and is used for switch dwell.
    /// The caller is responsible for pairing candidates that represent the same physical time.
    #[must_use]
    pub fn select<M: GnssSolution>(
        &mut self,
        timestamp_us: u64,
        samples: [Option<GnssSample<M>>; GNSS_RECEIVER_COUNT],
    ) -> Option<SelectedGnss<M>> {
        let valid = samples.map(|sample| sample.filter(|value| self.sample_is_valid(value)));
        match valid {
            [None, None] => None,
            [Some(sample), None] => Some(self.select_receiver(timestamp_us, RECEIVER_0, sample)),
            [None, Some(sample)] => Some(self.select_receiver(timestamp_us, RECEIVER_1, sample)),
            [Some(first), Some(second)] => {
                let best = better_sample(&first, &second);
                let dwell_active = self.last_switch_time_us.is_some_and(|last_switch_time_us| {
                    timestamp_us.saturating_sub(last_switch_time_us)
                        < self.config.minimum_switch_dwell_us
                });
                let selected = if best != self.primary && dwell_active {
                    self.primary
                } else {
                    best
                };
                let sample = if selected == RECEIVER_0 {
                    first
                } else {
                    second
                };
                Some(self.select_receiver(timestamp_us, selected, sample))
            }
        }
    }

    fn sample_is_valid<M: GnssSolution>(&self, sample: &GnssSample<M>) -> bool {
        sample.fix_tier >= self.config.minimum_fix_tier && sample.measurement.is_valid()
    }

    fn select_receiver<M>(
        &mut self,
        timestamp_us: u64,
        receiver: usize,
        sample: GnssSample<M>,
    ) -> SelectedGnss<M> {
        if self.last_switch_time_us.is_none() || receiver != self.primary {
            self.primary = receiver;
            self.last_switch_time_us = Some(timestamp_us);
        }
        SelectedGnss {
            measurement: sample.measurement,
            source: match receiver {
                RECEIVER_0 => GnssSource::Receiver0,
                RECEIVER_1 => GnssSource::Receiver1,
                _ => unreachable!("GNSS selector contains an invalid receiver index"),
            },
        }
    }
}

fn better_sample<M>(first: &GnssSample<M>, second: &GnssSample<M>) -> usize {
    if first.fix_tier != second.fix_tier {
        return if first.fix_tier > second.fix_tier {
            RECEIVER_0
        } else {
            RECEIVER_1
        };
    }
    let first_pdop = if first.pdop_centi == 0 {
        u16::MAX
    } else {
        first.pdop_centi
    };
    let second_pdop = if second.pdop_centi == 0 {
        u16::MAX
    } else {
        second.pdop_centi
    };
    if first_pdop <= second_pdop {
        RECEIVER_0
    } else {
        RECEIVER_1
    }
}

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
    use super::{
        Candidate, DualGnssSelector, GnssSample, GnssSelectorConfig, GnssSolution, GnssSource,
        HysteresisSelector, SelectorConfig,
    };

    impl GnssSolution for f32 {
        fn is_valid(&self) -> bool {
            self.is_finite()
        }
    }

    fn sample(measurement: f32, fix_tier: u8, pdop_centi: u16) -> GnssSample<f32> {
        GnssSample {
            measurement,
            fix_tier,
            pdop_centi,
        }
    }

    #[test]
    fn selects_one_receiver_without_changing_its_measurement() {
        let mut selector = DualGnssSelector::new(GnssSelectorConfig::new(3, 0));
        let selected = selector
            .select(
                1_000,
                [Some(sample(10.0, 3, 150)), Some(sample(20.0, 3, 80))],
            )
            .unwrap();
        assert_eq!(selected.source, GnssSource::Receiver1);
        assert_eq!(selected.measurement, 20.0);
    }

    #[test]
    fn dwell_applies_only_while_both_receivers_are_valid() {
        let mut selector = DualGnssSelector::new(GnssSelectorConfig::new(3, 100));
        let first = sample(10.0, 3, 150);
        let better = sample(20.0, 4, 80);

        assert_eq!(
            selector.select(10, [Some(first), None]).unwrap().source,
            GnssSource::Receiver0
        );
        assert_eq!(
            selector
                .select(50, [Some(first), Some(better)])
                .unwrap()
                .source,
            GnssSource::Receiver0
        );
        assert_eq!(
            selector
                .select(110, [Some(first), Some(better)])
                .unwrap()
                .source,
            GnssSource::Receiver1
        );
        assert_eq!(
            selector
                .select(111, [Some(first), Some(sample(f32::NAN, 4, 80))])
                .unwrap()
                .source,
            GnssSource::Receiver0
        );
    }

    #[test]
    fn first_epoch_uses_best_quality_without_dwell() {
        let mut selector = DualGnssSelector::new(GnssSelectorConfig::new(3, 500_000));
        let selected = selector
            .select(10, [Some(sample(10.0, 3, 150)), Some(sample(20.0, 4, 80))])
            .unwrap();
        assert_eq!(selected.source, GnssSource::Receiver1);
    }

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
