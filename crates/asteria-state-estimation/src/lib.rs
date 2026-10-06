#![no_std]

//! Shared inputs, outputs, selection policies, and estimator API for estimator banks.

mod estimator;
mod horizon;
mod input;
mod reference;
mod selector;
mod state;

pub use estimator::{
    BarometerAided, GnssAided, ImuAided, MagnetometerAided, StateEstimator, UpdateError,
};
pub use input::{BarometerInput, GnssInput, ImuInput, MagnetometerInput};
pub use reference::{GeodeticPosition, GeodeticReference};
pub use selector::{
    Candidate, DualGnssSelector, GNSS_RECEIVER_COUNT, GnssSample, GnssSelectorConfig, GnssSolution,
    GnssSource, HysteresisSelector, SelectedGnss, SelectorConfig,
};
pub use state::NavigationState;
