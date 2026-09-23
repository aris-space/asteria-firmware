//! Receiver-independent dual-GNSS arbitration.

/// Number of GNSS receivers handled by the selector.
pub const GNSS_RECEIVER_COUNT: usize = 2;

const RECEIVER_0: usize = 0;
const RECEIVER_1: usize = 1;

/// Measurement operations required by dual-receiver arbitration.
pub trait GnssSolution: Copy {
    /// Returns whether the measurement and its uncertainty are usable.
    fn is_valid(&self) -> bool;

    /// Returns whether both solutions agree within the supplied sigma gate.
    fn is_consistent_with(&self, other: &Self, gate_sigma: f32) -> bool;

    /// Inverse-variance blends two consistent solutions.
    #[must_use]
    fn blend_with(&self, other: &Self) -> Self;
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
    /// Consistent receiver solutions blended by inverse variance.
    Blended,
}

/// GNSS solution selected for fusion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SelectedGnss<M> {
    /// Observation to fuse.
    pub measurement: M,
    /// Receiver decision used to produce it.
    pub source: GnssSource,
}

/// Dual-GNSS selection and blending policy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssSelectorConfig {
    /// Minimum accepted fix tier.
    minimum_fix_tier: u8,
    /// Maximum inter-receiver discrepancy in combined standard deviations.
    consistency_gate_sigma: f32,
    /// Minimum fusion-time interval before switching between inconsistent receivers.
    minimum_switch_dwell_us: u64,
}

impl GnssSelectorConfig {
    /// Creates a validated dual-receiver selection policy.
    ///
    /// Returns `None` when `consistency_gate_sigma` is non-positive or non-finite.
    #[must_use]
    pub fn new(
        minimum_fix_tier: u8,
        consistency_gate_sigma: f32,
        minimum_switch_dwell_us: u64,
    ) -> Option<Self> {
        (consistency_gate_sigma.is_finite() && consistency_gate_sigma > 0.0).then_some(Self {
            minimum_fix_tier,
            consistency_gate_sigma,
            minimum_switch_dwell_us,
        })
    }
}

/// Stateful quality selector and conservative blender for two GNSS receivers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DualGnssSelector {
    config: GnssSelectorConfig,
    primary: usize,
    last_switch_time_us: u64,
}

impl DualGnssSelector {
    /// Creates a selector preferring receiver zero when quality is otherwise equal.
    #[must_use]
    pub const fn new(config: GnssSelectorConfig) -> Self {
        Self {
            config,
            primary: RECEIVER_0,
            last_switch_time_us: 0,
        }
    }

    /// Selects or blends the available samples from one completed receiver epoch.
    ///
    /// `fusion_time_us` is used only for receiver-switch dwell. The caller is responsible for
    /// assigning both samples to the same compensated fusion epoch.
    #[must_use]
    pub fn select<M: GnssSolution>(
        &mut self,
        fusion_time_us: u64,
        samples: [Option<GnssSample<M>>; GNSS_RECEIVER_COUNT],
    ) -> Option<SelectedGnss<M>> {
        let valid = samples.map(|sample| sample.filter(|value| self.sample_is_valid(value)));
        match valid {
            [None, None] => None,
            [Some(sample), None] => Some(self.select_receiver(fusion_time_us, RECEIVER_0, sample)),
            [None, Some(sample)] => Some(self.select_receiver(fusion_time_us, RECEIVER_1, sample)),
            [Some(first), Some(second)]
                if first.measurement.is_consistent_with(
                    &second.measurement,
                    self.config.consistency_gate_sigma,
                ) =>
            {
                Some(SelectedGnss {
                    measurement: first.measurement.blend_with(&second.measurement),
                    source: GnssSource::Blended,
                })
            }
            [Some(first), Some(second)] => {
                let best = better_sample(&first, &second);
                let selected = if best != self.primary
                    && fusion_time_us.saturating_sub(self.last_switch_time_us)
                        < self.config.minimum_switch_dwell_us
                {
                    self.primary
                } else {
                    best
                };
                let sample = if selected == RECEIVER_0 {
                    first
                } else {
                    second
                };
                Some(self.select_receiver(fusion_time_us, selected, sample))
            }
        }
    }

    fn sample_is_valid<M: GnssSolution>(&self, sample: &GnssSample<M>) -> bool {
        sample.fix_tier >= self.config.minimum_fix_tier && sample.measurement.is_valid()
    }

    fn select_receiver<M>(
        &mut self,
        fusion_time_us: u64,
        receiver: usize,
        sample: GnssSample<M>,
    ) -> SelectedGnss<M> {
        if receiver != self.primary {
            self.primary = receiver;
            self.last_switch_time_us = fusion_time_us;
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
