#![allow(dead_code)]
use hermes_can::messages::event_messages::{ThrustCurveConfig, ThrustPoint};

pub const DUMMY_THRUST_CURVE: ThrustCurveConfig = ThrustCurveConfig {
    thrust_curve: [
        ThrustPoint {
            thrust: 182,
            time: 0,
        },
        ThrustPoint {
            thrust: 255,
            time: 10,
        },
        ThrustPoint {
            thrust: 242,
            time: 12,
        },
        ThrustPoint {
            thrust: 229,
            time: 15,
        },
        ThrustPoint {
            thrust: 216,
            time: 17,
        },
        ThrustPoint {
            thrust: 203,
            time: 19,
        },
        ThrustPoint {
            thrust: 189,
            time: 21,
        },
        ThrustPoint {
            thrust: 176,
            time: 23,
        },
        ThrustPoint {
            thrust: 163,
            time: 25,
        },
        ThrustPoint {
            thrust: 150,
            time: 27,
        },
        ThrustPoint {
            thrust: 137,
            time: 29,
        },
        ThrustPoint {
            thrust: 124,
            time: 31,
        },
        ThrustPoint {
            thrust: 111,
            time: 33,
        },
        ThrustPoint {
            thrust: 98,
            time: 35,
        },
        ThrustPoint {
            thrust: 85,
            time: 37,
        },
        ThrustPoint {
            thrust: 71,
            time: 40,
        },
        ThrustPoint {
            thrust: 58,
            time: 42,
        },
        ThrustPoint {
            thrust: 45,
            time: 44,
        },
        ThrustPoint {
            thrust: 32,
            time: 46,
        },
        ThrustPoint {
            thrust: 19,
            time: 48,
        },
        ThrustPoint {
            thrust: 6,
            time: 50,
        },
        ThrustPoint {
            thrust: 61,
            time: 98,
        },
        ThrustPoint {
            thrust: 73,
            time: 156,
        },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
    ],
    thrust_curve_length: 23,
};
