use defmt::trace;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant};
use nalgebra::{Matrix3, Vector3};

use crate::measurements::{MagSample, RawMagSample};
use crate::sensors::{MAGNETOMETER_0, MAGNETOMETER_1, MagnetometerId};
use crate::signals;

/// LSM303AGR raw count -> nT (datasheet sensitivity).
const NT_PER_COUNT: f32 = 150.0;

/// Keep using the current primary; only switch on timeout.
const MAG_TIMEOUT: Duration = Duration::from_millis(100);

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut sub0 = signals::MAG_CHANNELS[MAGNETOMETER_0.index()]
        .subscriber()
        .expect("mag: subscribe 0");
    let mut sub1 = signals::MAG_CHANNELS[MAGNETOMETER_1.index()]
        .subscriber()
        .expect("mag: subscribe 1");

    let mut selector = TimeoutSelector::new(MAG_TIMEOUT);
    let sender = signals::MAG_FIELD_WATCH.sender();

    loop {
        let sample: RawMagSample =
            match select(sub0.next_message_pure(), sub1.next_message_pure()).await {
                Either::First(s) | Either::Second(s) => s,
            };

        if !selector.accept(sample.src, sample.ts) {
            continue;
        }

        let (hard_iron, soft_iron) = calibration_for(sample.src);
        let raw_nt = Vector3::new(
            sample.x as f32 * NT_PER_COUNT,
            sample.y as f32 * NT_PER_COUNT,
            sample.z as f32 * NT_PER_COUNT,
        );
        let calibrated = soft_iron * (raw_nt - hard_iron);

        let out = MagSample {
            src: sample.src,
            ts: sample.ts,
            x: calibrated.x,
            y: calibrated.y,
            z: calibrated.z,
        };
        sender.send(out);
        trace!("mag: nt x={} y={} z={}", out.x, out.y, out.z);
    }
}

#[allow(clippy::excessive_precision)]
fn calibration_for(id: MagnetometerId) -> (Vector3<f32>, Matrix3<f32>) {
    if id == MAGNETOMETER_0 {
        let hard_iron = Vector3::new(
            -5548.220872821729_f32,
            3827.5558491680104_f32,
            1103.1746334467373_f32,
        );
        let soft_iron = Matrix3::new(
            0.9933533874801035_f32,
            0.02660683424766733_f32,
            -0.012040160633120614_f32,
            0.0266068342476673_f32,
            1.0749237129791611_f32,
            -0.004425871227641531_f32,
            -0.01204016063312053_f32,
            -0.0044258712276415224_f32,
            0.9896712863098899_f32,
        );
        (hard_iron, soft_iron)
    } else {
        let hard_iron = Vector3::new(
            1877.0587380839947_f32,
            7553.287522291296_f32,
            6455.773615317315_f32,
        );
        let soft_iron = Matrix3::new(
            0.9947437765775108_f32,
            0.02142849359925576_f32,
            0.013753388737255394_f32,
            0.021428493599255673_f32,
            1.056768106384783_f32,
            -0.003894936674428707_f32,
            0.013753388737255299_f32,
            -0.003894936674428721_f32,
            0.999343144534608_f32,
        );
        (hard_iron, soft_iron)
    }
}

struct TimeoutSelector {
    primary: MagnetometerId,
    last_update: Instant,
    timeout: Duration,
}

impl TimeoutSelector {
    fn new(timeout: Duration) -> Self {
        Self {
            primary: MAGNETOMETER_0,
            last_update: Instant::now(),
            timeout,
        }
    }

    fn accept(&mut self, id: MagnetometerId, ts: Instant) -> bool {
        if self.primary == id || self.last_update.elapsed() > self.timeout {
            self.primary = id;
            self.last_update = ts;
            true
        } else {
            false
        }
    }
}
