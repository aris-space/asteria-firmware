use crate::error::EstimatorError;
use crate::filter::{
    MeasurementUpdate, PressureMeasurement, VerticalFilter, VerticalFilterConfig,
    VerticalGnssMeasurement, VerticalGnssUpdate, VerticalState, VerticalUncertainty,
};
use crate::imu::{ImuAttitudeConfig, ImuAttitudeStatus, ImuMeasurement, ImuVerticalizer};
use crate::sensor_id::{BarometerId, IMU_0, IMU_1, IMU_COUNT, ImuId};
use asteria_state_estimation::{
    BarometerAided, BarometerInput, Candidate, GNSS_RECEIVER_COUNT, GnssAided, GnssInput,
    GnssSolution, HysteresisSelector, ImuAided, ImuInput, MagnetometerAided, MagnetometerInput,
    NavigationState, SelectorConfig, StateEstimator, UpdateError,
};
use heapless::Deque;

const MAX_MAGNETOMETER_AGE_US: u64 = 250_000;

#[derive(Clone, Copy)]
struct ImuSample {
    time_us: u64,
    measurement: ImuMeasurement,
}

/// North and east components of the newest fused GNSS fix, held until the next one. SEF Light
/// neither filters nor propagates them.
#[derive(Clone, Copy)]
struct HorizontalFix {
    position_ne_m: [f32; 2],
    position_std_ne_m: [f32; 2],
    velocity_ne_mps: [f32; 2],
    velocity_std_ne_mps: [f32; 2],
}

impl HorizontalFix {
    const UNKNOWN: Self = Self {
        position_ne_m: [0.0; 2],
        position_std_ne_m: [f32::INFINITY; 2],
        velocity_ne_mps: [0.0; 2],
        velocity_std_ne_mps: [f32::INFINITY; 2],
    };

    fn from_input(input: &GnssInput) -> Option<Self> {
        let [north_m, east_m, _] = input.position_ned_m;
        let [north_std_m, east_std_m, _] = input.position_std_m;
        let [north_mps, east_mps, _] = input.velocity_ned_mps;
        let [north_std_mps, east_std_mps, _] = input.velocity_std_mps;
        let fix = Self {
            position_ne_m: [north_m, east_m],
            position_std_ne_m: [north_std_m, east_std_m],
            velocity_ne_mps: [north_mps, east_mps],
            velocity_std_ne_mps: [north_std_mps, east_std_mps],
        };
        let values = [fix.position_ne_m, fix.velocity_ne_mps]
            .into_iter()
            .flatten();
        let deviations = [fix.position_std_ne_m, fix.velocity_std_ne_mps]
            .into_iter()
            .flatten();
        (values.chain(deviations.clone()).all(f32::is_finite)
            && deviations.into_iter().all(|std| std >= 0.0))
        .then_some(fix)
    }
}

#[derive(Clone, Copy)]
enum FilterEvent {
    Prediction {
        sample_time_us: u64,
        imu: usize,
        acceleration_up_mps2: f32,
        degraded: bool,
        dt_s: f32,
    },
    Pressure {
        sample_time_us: u64,
        barometer: BarometerId,
        measurement: PressureMeasurement,
    },
    Gnss {
        sample_time_us: u64,
        measurement: VerticalGnssMeasurement,
    },
}

#[derive(Clone, Copy)]
enum FilterEventResult {
    Prediction,
    Pressure([MeasurementUpdate; IMU_COUNT]),
    Gnss([VerticalGnssUpdate; IMU_COUNT]),
}

impl FilterEvent {
    const fn sample_time_us(self) -> u64 {
        match self {
            Self::Prediction { sample_time_us, .. }
            | Self::Pressure { sample_time_us, .. }
            | Self::Gnss { sample_time_us, .. } => sample_time_us,
        }
    }

    const fn order(self) -> u8 {
        match self {
            Self::Prediction { .. } => 0,
            Self::Pressure { .. } => 1,
            Self::Gnss { .. } => 2,
        }
    }

    const fn sort_key(self) -> (u64, u8) {
        (self.sample_time_us(), self.order())
    }

    fn apply(
        self,
        filters: &mut [VerticalFilter; IMU_COUNT],
    ) -> Result<FilterEventResult, EstimatorError> {
        match self {
            Self::Prediction {
                imu,
                acceleration_up_mps2,
                degraded,
                dt_s,
                ..
            } => {
                if degraded {
                    filters[imu].predict_degraded(acceleration_up_mps2, dt_s)?;
                } else {
                    filters[imu].predict(acceleration_up_mps2, dt_s)?;
                }
                Ok(FilterEventResult::Prediction)
            }
            Self::Pressure {
                barometer,
                measurement,
                ..
            } => Ok(FilterEventResult::Pressure([
                filters[0].update_pressure(barometer, measurement)?,
                filters[1].update_pressure(barometer, measurement)?,
            ])),
            Self::Gnss { measurement, .. } => Ok(FilterEventResult::Gnss([
                filters[0].update_gnss(measurement)?,
                filters[1].update_gnss(measurement)?,
            ])),
        }
    }
}

/// Health-score and switching policy for the two vertical estimator chains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerticalEstimatorSelectorConfig {
    /// Weight retained from the previous normalized-innovation score.
    score_memory: f32,
    /// Maximum normalized innovation contribution from one measurement update.
    maximum_nis_contribution: f32,
    /// Score added while an IMU's attitude estimator reports degraded acceleration.
    degraded_score_penalty: f32,
    /// Maximum allowed age of an IMU sample before its chain becomes ineligible.
    maximum_imu_age_us: u64,
    /// Shared hysteresis and dwell-time policy.
    selection: SelectorConfig,
}

impl VerticalEstimatorSelectorConfig {
    /// Creates a validated vertical-estimator selection policy.
    ///
    /// # Errors
    ///
    /// Returns an error when a score parameter is outside its supported range or the maximum IMU
    /// age is zero.
    pub fn new(
        score_memory: f32,
        maximum_nis_contribution: f32,
        degraded_score_penalty: f32,
        maximum_imu_age_us: u64,
        selection: SelectorConfig,
    ) -> Result<Self, EstimatorError> {
        if !score_memory.is_finite()
            || !(0.0..1.0).contains(&score_memory)
            || !maximum_nis_contribution.is_finite()
            || maximum_nis_contribution <= 0.0
            || !degraded_score_penalty.is_finite()
            || degraded_score_penalty < 0.0
            || maximum_imu_age_us == 0
        {
            return Err(EstimatorError::OutOfRangeInput);
        }
        Ok(Self {
            score_memory,
            maximum_nis_contribution,
            degraded_score_penalty,
            maximum_imu_age_us,
            selection,
        })
    }
}

/// Signed difference between the IMU-zero and IMU-one estimator chains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerticalDisagreement {
    /// `imu0.height_m - imu1.height_m`.
    pub height_difference_m: f32,
    /// `imu0.velocity_mps - imu1.velocity_mps`.
    pub velocity_difference_mps: f32,
}

/// Two independent IMU and vertical-filter chains with shared aiding measurements.
///
/// Each IMU advances only its matching filter. Every barometer or GNSS observation is applied to
/// both filters. Normalized innovations, IMU freshness, and attitude health drive the selected
/// output while both underlying estimates remain available for diagnostics. The const generic
/// sets the maximum number of retained prediction and aiding events.
pub struct DualVerticalEstimator<const FILTER_HISTORY_CAPACITY: usize> {
    imu_verticalizers: [ImuVerticalizer; IMU_COUNT],
    magnetic_field: [Option<(u64, [f32; 3])>; IMU_COUNT],
    filters: [VerticalFilter; IMU_COUNT],
    history_base_filters: [VerticalFilter; IMU_COUNT],
    history_base_scores: [f32; IMU_COUNT],
    filter_history: Deque<FilterEvent, FILTER_HISTORY_CAPACITY>,
    history_floor_key: Option<(u64, u8)>,
    maximum_aiding_delay_us: u64,
    last_imu: [Option<ImuSample>; IMU_COUNT],
    horizontal: HorizontalFix,
    imu_has_predicted: [bool; IMU_COUNT],
    consistency_scores: [f32; IMU_COUNT],
    selector: HysteresisSelector,
    selector_config: VerticalEstimatorSelectorConfig,
}

impl<const FILTER_HISTORY_CAPACITY: usize> DualVerticalEstimator<FILTER_HISTORY_CAPACITY> {
    /// Creates two independent chains with a common vertical-filter configuration.
    ///
    /// Separate attitude configurations allow for different IMU ranges or update rates without
    /// coupling their state. Selection policy is explicit because its timing and score thresholds
    /// depend on the vehicle and sensor suite. `maximum_aiding_delay_us` controls how much filter
    /// history is retained for delayed barometer and GNSS updates.
    ///
    /// # Errors
    ///
    /// Returns an error if the delay window is zero. A zero history capacity does
    /// not compile.
    pub fn new(
        filter_config: VerticalFilterConfig,
        attitude_configs: [ImuAttitudeConfig; IMU_COUNT],
        selector_config: VerticalEstimatorSelectorConfig,
        maximum_aiding_delay_us: u64,
    ) -> Result<Self, EstimatorError> {
        if maximum_aiding_delay_us == 0 {
            return Err(EstimatorError::OutOfRangeInput);
        }
        let selector = HysteresisSelector::new(selector_config.selection, 0);
        let filters = [
            VerticalFilter::new(filter_config),
            VerticalFilter::new(filter_config),
        ];
        Ok(Self {
            imu_verticalizers: [
                ImuVerticalizer::new(attitude_configs[0]),
                ImuVerticalizer::new(attitude_configs[1]),
            ],
            magnetic_field: [None; IMU_COUNT],
            filters,
            history_base_filters: filters,
            history_base_scores: [0.0; IMU_COUNT],
            filter_history: Deque::new(),
            history_floor_key: None,
            maximum_aiding_delay_us,
            last_imu: [None; IMU_COUNT],
            horizontal: HorizontalFix::UNKNOWN,
            imu_has_predicted: [false; IMU_COUNT],
            consistency_scores: [0.0; IMU_COUNT],
            selector,
            selector_config,
        })
    }

    /// Processes one timestamped IMU sample and advances only that IMU's filter.
    ///
    /// The first sample for each IMU establishes its time origin and its initial attitude from
    /// the measured gravity, and returns `false`. Every later sample derives its time step from
    /// that IMU's preceding sample, advances the chain, and returns `true`.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid IMU input or a non-increasing timestamp.
    pub fn update_imu(
        &mut self,
        imu: ImuId,
        sample_time_us: u64,
        measurement: ImuMeasurement,
    ) -> Result<bool, EstimatorError> {
        let index = imu.index();
        ImuVerticalizer::validate_measurement(measurement)?;
        if self
            .history_floor_key
            .is_some_and(|history_floor_key| (sample_time_us, 0) < history_floor_key)
        {
            return Err(EstimatorError::MeasurementTooOld);
        }
        let Some(ImuSample {
            time_us: previous_time_us,
            ..
        }) = self.last_imu[index]
        else {
            self.imu_verticalizers[index].align_to_gravity(measurement);
            self.last_imu[index] = Some(ImuSample {
                time_us: sample_time_us,
                measurement,
            });
            return Ok(false);
        };
        let elapsed_us = sample_time_us
            .checked_sub(previous_time_us)
            .filter(|elapsed_us| *elapsed_us > 0)
            .ok_or(EstimatorError::NonMonotonicImuTimestamp)?;
        let dt_s = core::time::Duration::from_micros(elapsed_us).as_secs_f32();
        let magnetic_field = self.magnetic_field[index].and_then(|(time_us, field)| {
            sample_time_us
                .checked_sub(time_us)
                .filter(|age| *age <= MAX_MAGNETOMETER_AGE_US)
                .map(|_| field)
        });
        let acceleration_up_mps2 = self.imu_verticalizers[index].update_with_magnetometer(
            measurement,
            magnetic_field,
            dt_s,
        )?;
        let degraded = self.imu_verticalizers[index]
            .status()
            .acceleration_degraded();
        self.apply_filter_event(FilterEvent::Prediction {
            sample_time_us,
            imu: index,
            acceleration_up_mps2,
            degraded,
            dt_s,
        })?;
        self.last_imu[index] = Some(ImuSample {
            time_us: sample_time_us,
            measurement,
        });
        self.imu_has_predicted[index] = true;
        self.select_best();
        Ok(true)
    }

    /// Supplies a calibrated body-frame magnetic field to one IMU attitude chain.
    /// The field is held for at most 250 ms, then AHRS continues without it.
    /// Magnetic input changes attitude only; it does not insert a vertical-filter event.
    pub fn update_magnetometer(
        &mut self,
        imu: ImuId,
        sample_time_us: u64,
        field: [f32; 3],
    ) -> Result<(), EstimatorError> {
        crate::error::validate_finite(&field)?;
        if field.iter().all(|component| *component == 0.0) {
            return Err(EstimatorError::OutOfRangeInput);
        }
        self.magnetic_field[imu.index()] = Some((sample_time_us, field));
        Ok(())
    }

    /// Applies one timestamped barometer observation to both estimator chains.
    ///
    /// A delayed observation is fused at `sample_time_us`, after which retained predictions and
    /// aiding updates are replayed to recover the current state.
    ///
    /// # Errors
    ///
    /// Returns an error if the pressure observation is invalid or older than retained history.
    pub fn update_pressure(
        &mut self,
        sample_time_us: u64,
        barometer: BarometerId,
        measurement: PressureMeasurement,
    ) -> Result<[MeasurementUpdate; IMU_COUNT], EstimatorError> {
        let FilterEventResult::Pressure(updates) =
            self.apply_filter_event(FilterEvent::Pressure {
                sample_time_us,
                barometer,
                measurement,
            })?
        else {
            unreachable!("pressure event returned a non-pressure result")
        };
        self.select_best();
        Ok(updates)
    }

    /// Applies one barometer observation using the shared sensor-update naming.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Self::update_pressure`].
    pub fn update_barometer(
        &mut self,
        barometer: BarometerId,
        sample_time_us: u64,
        measurement: PressureMeasurement,
    ) -> Result<[MeasurementUpdate; IMU_COUNT], EstimatorError> {
        self.update_pressure(sample_time_us, barometer, measurement)
    }

    /// Applies one GNSS fix already selected by the caller to both estimator chains.
    /// `sample_time_us` is its calibrated physical measurement time on the estimator clock.
    ///
    /// # Errors
    ///
    /// Returns an error if the fix is invalid or older than retained history.
    pub fn fuse_gnss(
        &mut self,
        sample_time_us: u64,
        measurement: VerticalGnssMeasurement,
    ) -> Result<[VerticalGnssUpdate; IMU_COUNT], EstimatorError> {
        let FilterEventResult::Gnss(updates) = self.apply_filter_event(FilterEvent::Gnss {
            sample_time_us,
            measurement,
        })?
        else {
            unreachable!("GNSS event returned a non-GNSS result")
        };
        self.select_best();
        Ok(updates)
    }

    /// Changes one barometer bias random walk in both estimator chains.
    ///
    /// Setting this to zero freezes the process-noise growth for that bias. This is intended to be
    /// controlled by external flight-phase logic.
    ///
    /// # Errors
    ///
    /// Returns an error for a negative or non-finite standard deviation.
    pub fn set_barometer_bias_walk_std(
        &mut self,
        barometer: BarometerId,
        standard_deviation_m_per_sqrt_s: f32,
    ) -> Result<(), EstimatorError> {
        self.filters[0].set_barometer_bias_walk_std(barometer, standard_deviation_m_per_sqrt_s)?;
        self.filters[1].set_barometer_bias_walk_std(barometer, standard_deviation_m_per_sqrt_s)?;
        let latest_history_key = self.filter_history.back().map(|event| event.sort_key());
        let latest_imu_key = self.newest_imu_time_us().map(|time_us| (time_us, 0));
        self.history_floor_key = [self.history_floor_key, latest_history_key, latest_imu_key]
            .into_iter()
            .flatten()
            .max();
        self.history_base_filters = self.filters;
        self.history_base_scores = self.consistency_scores;
        self.filter_history.clear();
        Ok(())
    }

    /// Returns both vertical states in IMU ID order.
    #[must_use]
    pub fn states(&self) -> [VerticalState; IMU_COUNT] {
        [self.filters[0].state(), self.filters[1].state()]
    }

    /// Returns one inner filter for read-only inspection.
    #[must_use]
    pub fn filter(&self, imu: ImuId) -> &VerticalFilter {
        &self.filters[imu.index()]
    }

    /// Returns one vertical state.
    #[must_use]
    pub fn state(&self, imu: ImuId) -> VerticalState {
        self.filter(imu).state()
    }

    /// Returns the IMU chain currently driving the combined output.
    #[must_use]
    pub fn selected_imu(&self) -> ImuId {
        match self.selector.selected() {
            0 => IMU_0,
            1 => IMU_1,
            _ => unreachable!("vertical selector returned an invalid IMU index"),
        }
    }

    /// Returns the state that should be consumed by downstream flight logic.
    #[must_use]
    pub fn selected_state(&self) -> VerticalState {
        self.state(self.selected_imu())
    }

    /// Returns the uncertainty associated with the selected state.
    #[must_use]
    pub fn selected_uncertainty(&self) -> VerticalUncertainty {
        self.uncertainty(self.selected_imu())
    }

    /// Returns the innovation-only consistency score for each chain.
    #[must_use]
    pub const fn consistency_scores(&self) -> [f32; IMU_COUNT] {
        self.consistency_scores
    }

    /// Returns the externally relevant covariance terms for one estimator chain.
    #[must_use]
    pub fn uncertainty(&self, imu: ImuId) -> VerticalUncertainty {
        self.filter(imu).uncertainty()
    }

    /// Returns attitude-estimator diagnostics for one IMU.
    #[must_use]
    pub fn imu_status(&self, imu: ImuId) -> ImuAttitudeStatus {
        self.imu_verticalizers[imu.index()].status()
    }

    /// Returns one IMU's body-to-NED orientation quaternion in `[w, x, y, z]` order.
    #[must_use]
    pub fn orientation_body_to_ned_wxyz(&self, imu: ImuId) -> [f32; 4] {
        self.imu_verticalizers[imu.index()].orientation_body_to_ned_wxyz()
    }

    /// Returns the latest sample timestamp seen for one IMU.
    #[must_use]
    pub fn last_imu_sample_time_us(&self, imu: ImuId) -> Option<u64> {
        self.last_imu[imu.index()].map(|sample| sample.time_us)
    }

    /// Returns whether one IMU chain has predicted and completed attitude initialization.
    #[must_use]
    pub fn imu_ready(&self, imu: ImuId) -> bool {
        let index = imu.index();
        self.imu_has_predicted[index]
            && !self.imu_verticalizers[index].status().flags.initialising()
    }

    /// Returns whether both IMU chains are ready.
    ///
    /// Check this before interpreting [`Self::disagreement`] as a redundancy-health signal.
    #[must_use]
    pub fn redundancy_ready(&self) -> bool {
        self.imu_has_predicted
            .iter()
            .zip(self.imu_verticalizers.iter())
            .all(|(has_predicted, verticalizer)| {
                *has_predicted && !verticalizer.status().flags.initialising()
            })
    }

    /// Returns the signed difference between the two vertical estimates.
    ///
    /// This is numerically available from construction because both filters have explicit prior
    /// states. It becomes meaningful as a redundancy-health signal once
    /// [`Self::redundancy_ready`] returns `true`.
    #[must_use]
    pub fn disagreement(&self) -> VerticalDisagreement {
        let states = self.states();
        VerticalDisagreement {
            height_difference_m: states[0].height_m - states[1].height_m,
            velocity_difference_mps: states[0].velocity_mps - states[1].velocity_mps,
        }
    }

    fn record_scores(
        scores: &mut [f32; IMU_COUNT],
        result: &FilterEventResult,
        config: VerticalEstimatorSelectorConfig,
    ) {
        let normalized_innovations = match result {
            FilterEventResult::Prediction => return,
            FilterEventResult::Pressure(updates) => {
                updates.map(|update| update.normalized_innovation_squared)
            }
            FilterEventResult::Gnss(updates) => updates.map(|update| {
                f32::midpoint(
                    update.height.normalized_innovation_squared,
                    update.velocity.normalized_innovation_squared,
                )
            }),
        };
        for (score, normalized_innovation_squared) in scores.iter_mut().zip(normalized_innovations)
        {
            let contribution = normalized_innovation_squared.min(config.maximum_nis_contribution);
            *score = config.score_memory * *score + (1.0 - config.score_memory) * contribution;
        }
    }

    fn apply_filter_event(
        &mut self,
        event: FilterEvent,
    ) -> Result<FilterEventResult, EstimatorError> {
        if self
            .history_floor_key
            .is_some_and(|history_floor_key| event.sort_key() < history_floor_key)
        {
            return Err(EstimatorError::MeasurementTooOld);
        }

        let event_is_in_order = self
            .filter_history
            .back()
            .is_none_or(|latest| latest.sort_key() <= event.sort_key());
        if event_is_in_order {
            let result = event.apply(&mut self.filters)?;
            Self::record_scores(&mut self.consistency_scores, &result, self.selector_config);
            let history_cutoff_time_us = event
                .sample_time_us()
                .saturating_sub(self.maximum_aiding_delay_us);
            while self
                .filter_history
                .front()
                .is_some_and(|oldest| oldest.sample_time_us() < history_cutoff_time_us)
            {
                let evicted = self
                    .filter_history
                    .pop_front()
                    .expect("history is not empty");
                let result = evicted.apply(&mut self.history_base_filters)?;
                Self::record_scores(&mut self.history_base_scores, &result, self.selector_config);
                self.history_floor_key = Some(evicted.sort_key());
            }
            if self.filter_history.is_full() {
                let evicted = self
                    .filter_history
                    .pop_front()
                    .expect("history is not empty");
                let result = evicted.apply(&mut self.history_base_filters)?;
                Self::record_scores(&mut self.history_base_scores, &result, self.selector_config);
                self.history_floor_key = Some(evicted.sort_key());
            }
            self.filter_history
                .push_back(event)
                .unwrap_or_else(|_| unreachable!("history capacity was made available"));
            return Ok(result);
        }

        let mut validation_filters = self.filters;
        event.apply(&mut validation_filters)?;

        if self.filter_history.is_full() {
            let oldest = *self.filter_history.front().expect("history is full");
            if event.sort_key() < oldest.sort_key() {
                return Err(EstimatorError::MeasurementTooOld);
            }
            let evicted = self.filter_history.pop_front().expect("history is full");
            let result = evicted.apply(&mut self.history_base_filters)?;
            Self::record_scores(&mut self.history_base_scores, &result, self.selector_config);
            self.history_floor_key = Some(evicted.sort_key());
        }

        let insertion_index = self
            .filter_history
            .iter()
            .position(|stored| stored.sort_key() > event.sort_key())
            .unwrap_or(self.filter_history.len());
        // A ring buffer cannot insert in the middle, so a late event rebuilds
        // the history; late events are rare and replay the history anyway.
        let mut reordered = Deque::new();
        for (index, stored) in self.filter_history.iter().copied().enumerate() {
            if index == insertion_index {
                let _ = reordered.push_back(event);
            }
            let _ = reordered.push_back(stored);
        }
        if insertion_index == self.filter_history.len() {
            let _ = reordered.push_back(event);
        }
        self.filter_history = reordered;

        let mut replayed_filters = self.history_base_filters;
        let mut replayed_scores = self.history_base_scores;
        let mut inserted_result = None;
        for (index, stored) in self.filter_history.iter().copied().enumerate() {
            let result = stored.apply(&mut replayed_filters)?;
            Self::record_scores(&mut replayed_scores, &result, self.selector_config);
            if index == insertion_index {
                inserted_result = Some(result);
            }
        }
        self.filters = replayed_filters;
        self.consistency_scores = replayed_scores;
        Ok(inserted_result.expect("inserted history event must be replayed"))
    }

    fn newest_imu_time_us(&self) -> Option<u64> {
        self.last_imu
            .into_iter()
            .flatten()
            .map(|sample| sample.time_us)
            .max()
    }

    fn select_best(&mut self) {
        let newest_timestamp_us = self.newest_imu_time_us().unwrap_or(0);
        let candidates = core::array::from_fn::<_, IMU_COUNT, _>(|imu| Candidate {
            index: imu,
            score: self.consistency_scores[imu]
                + if self.imu_verticalizers[imu].status().acceleration_degraded() {
                    self.selector_config.degraded_score_penalty
                } else {
                    0.0
                },
            valid: self.imu_has_predicted[imu]
                && self.last_imu[imu].is_some_and(|sample| {
                    newest_timestamp_us.saturating_sub(sample.time_us)
                        <= self.selector_config.maximum_imu_age_us
                }),
        });
        self.selector.select(newest_timestamp_us, candidates);
    }
}

impl<const FILTER_HISTORY_CAPACITY: usize> ImuAided
    for DualVerticalEstimator<FILTER_HISTORY_CAPACITY>
{
    fn update_imu(&mut self, input: ImuInput) -> Result<(), UpdateError> {
        let imu = ImuId::from_index(input.sensor_index).ok_or(UpdateError::InvalidSensor)?;
        let measurement = ImuMeasurement {
            acceleration_body_mps2: input.acceleration_body_mps2,
            angular_rate_body_rad_s: input.angular_rate_body_rad_s,
        };
        DualVerticalEstimator::update_imu(self, imu, input.timestamp_us, measurement)
            .map(|_| ())
            .map_err(map_update_error)
    }
}

impl<const FILTER_HISTORY_CAPACITY: usize> BarometerAided
    for DualVerticalEstimator<FILTER_HISTORY_CAPACITY>
{
    fn update_barometer(&mut self, input: BarometerInput) -> Result<(), UpdateError> {
        let barometer =
            BarometerId::from_index(input.sensor_index).ok_or(UpdateError::InvalidSensor)?;
        let measurement = PressureMeasurement {
            height_m: input.height_m,
            height_std_m: input.height_std_m,
        };
        DualVerticalEstimator::update_barometer(self, barometer, input.timestamp_us, measurement)
            .map(|_| ())
            .map_err(map_update_error)
    }
}

impl<const FILTER_HISTORY_CAPACITY: usize> GnssAided
    for DualVerticalEstimator<FILTER_HISTORY_CAPACITY>
{
    fn update_gnss(&mut self, input: GnssInput) -> Result<(), UpdateError> {
        if input.sensor_index >= GNSS_RECEIVER_COUNT {
            return Err(UpdateError::InvalidSensor);
        }
        let measurement = VerticalGnssMeasurement {
            height_m: input.height_m,
            velocity_mps: input.vertical_velocity_mps,
            height_std_m: input.height_std_m,
            velocity_std_mps: input.vertical_velocity_std_mps,
        };
        self.fuse_gnss(input.timestamp_us, measurement)
            .map_err(map_update_error)?;
        if let Some(fix) = HorizontalFix::from_input(&input) {
            self.horizontal = fix;
        }
        Ok(())
    }
}

impl<const FILTER_HISTORY_CAPACITY: usize> StateEstimator
    for DualVerticalEstimator<FILTER_HISTORY_CAPACITY>
{
    /// Height and vertical velocity come from the selected chain's filter, attitude from its AHRS,
    /// and north and east from the newest fused GNSS fix. SEF Light estimates no IMU biases, so
    /// the rates and specific force are the selected IMU's newest calibrated sample.
    fn navigation_state(&self) -> NavigationState {
        let imu = self.selected_imu();
        let vertical = self.state(imu);
        let uncertainty = self.uncertainty(imu);
        let horizontal = self.horizontal;
        let sample = self.last_imu[imu.index()];
        NavigationState {
            time_us: sample.map_or(0, |sample| sample.time_us),
            position_ned_m: [
                horizontal.position_ne_m[0],
                horizontal.position_ne_m[1],
                -vertical.height_m,
            ],
            position_std_ned_m: [
                horizontal.position_std_ne_m[0],
                horizontal.position_std_ne_m[1],
                libm::sqrtf(uncertainty.height_variance_m2),
            ],
            velocity_ned_mps: [
                horizontal.velocity_ne_mps[0],
                horizontal.velocity_ne_mps[1],
                -vertical.velocity_mps,
            ],
            velocity_std_ned_mps: [
                horizontal.velocity_std_ne_mps[0],
                horizontal.velocity_std_ne_mps[1],
                libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
            ],
            orientation_body_to_ned_wxyz: self.orientation_body_to_ned_wxyz(imu),
            angular_rate_body_rad_s: sample.map_or([0.0; 3], |sample| {
                sample.measurement.angular_rate_body_rad_s
            }),
            specific_force_body_mps2: sample
                .map_or([0.0; 3], |sample| sample.measurement.acceleration_body_mps2),
        }
    }
}

impl<const FILTER_HISTORY_CAPACITY: usize> MagnetometerAided
    for DualVerticalEstimator<FILTER_HISTORY_CAPACITY>
{
    fn update_magnetometer(&mut self, input: MagnetometerInput) -> Result<(), UpdateError> {
        let imu = ImuId::from_index(input.sensor_index).ok_or(UpdateError::InvalidSensor)?;
        DualVerticalEstimator::update_magnetometer(self, imu, input.timestamp_us, input.field_body)
            .map_err(map_update_error)
    }
}

fn map_update_error(error: EstimatorError) -> UpdateError {
    match error {
        EstimatorError::NonFiniteInput
        | EstimatorError::NonPositiveInput
        | EstimatorError::NegativeStandardDeviation
        | EstimatorError::OutOfRangeInput => UpdateError::InvalidMeasurement,
        EstimatorError::NonMonotonicImuTimestamp => UpdateError::NonMonotonicImuTimestamp,
        EstimatorError::MeasurementTooOld => UpdateError::MeasurementTooOld,
    }
}

impl GnssSolution for VerticalGnssMeasurement {
    fn is_valid(&self) -> bool {
        [
            self.height_m,
            self.velocity_mps,
            self.height_std_m,
            self.velocity_std_mps,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.height_std_m > 0.0
            && self.velocity_std_mps > 0.0
    }
}
