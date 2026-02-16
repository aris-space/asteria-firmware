use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embedded_utils::info;
use hermes_can::messages::event_messages::{ThrustCurveConfig, ThrustPoint};

pub mod abort_sequence;
pub mod actions;
// mod cold_flow_sequence;
mod cold_flow_sequence;
mod controller;
pub mod firing_sequence;
pub mod runner;

pub static THRUST_CURVE: OnceLock<Mutex<ThreadModeRawMutex, ThrustCurve>> = OnceLock::new();
pub static THRUST_CURVE_HASH: Mutex<ThreadModeRawMutex, u64> = Mutex::new(0);
pub const FSS_INJ_GAIN: f32 = 1.1; //ToDo: experimentally determine
pub const OSS_INJ_GAIN: f32 = 1.1; //ToDo: experimentally determine
pub const _FSS_GAIN: f32 = 1.2;
pub const _OSS_GAIN: f32 = 1.2;

#[derive(Clone, Copy, Debug)]
pub struct ThrustCurve {
    pub points: [PressurePoint; 25],
    pub length: u8,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PressurePoint {
    pub oxidizer_p: f32,
    pub fuel_p: f32,
    pub absolute_time_ms: u32,
}

pub async fn initiate_thrust_curve(config: ThrustCurveConfig) -> Result<u64, &'static str> {
    if config.thrust_curve_length > 25 {
        return Err("Thrust curve length exceeds maximum of 25 points");
    }

    let mut points = [PressurePoint {
        oxidizer_p: 0.0,
        fuel_p: 0.0,
        absolute_time_ms: 0,
    }; 25];

    for (idx, thrust_point) in config.thrust_curve.iter().enumerate() {
        let eng_p = thrust_point.thrust as f32 * (10.0 - 1.0) / 255.0 + 1.0; // Map 0-255 to 1-10 barg ToDo: change back for firing
        let absolute_time_ms = thrust_point.time as u32 * 100; // Given in 100ms increments

        points[idx] = PressurePoint {
            oxidizer_p: eng_p * OSS_INJ_GAIN,
            fuel_p: eng_p * FSS_INJ_GAIN,
            absolute_time_ms,
        };
    }

    let curve = ThrustCurve {
        points,
        length: config.thrust_curve_length,
    };

    if THRUST_CURVE.is_set() {
        *THRUST_CURVE.get().await.lock().await = curve;
    } else {
        THRUST_CURVE
            .init(Mutex::new(curve))
            .map_err(|_| "Thrust curve initialization failed")?;
    }

    info!("THRUST_CURVE INITIATED");
    let length = curve.length as usize;
    info!("{}", &curve.points[0..length]);

    let hash = config.calculate_unique_hash();

    Ok(hash)
}

pub const _COLD_FLOW_THRUST_CURVE: ThrustCurveConfig = ThrustCurveConfig {
    thrust_curve: [
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint {
            thrust: 0,
            time: 20,
        },
        ThrustPoint {
            thrust: 255,
            time: 21,
        },
        ThrustPoint {
            thrust: 255,
            time: 50,
        },
        ThrustPoint {
            thrust: 0,
            time: 51,
        },
        ThrustPoint {
            thrust: 0,
            time: 80,
        },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
        ThrustPoint { thrust: 0, time: 0 },
    ],
    thrust_curve_length: 6,
};
