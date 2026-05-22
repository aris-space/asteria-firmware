use crate::{filters::ExponentialMovingAverage, sensors::dht};
use datatypes::units::{Celsius, HPa};
use dp_sensor_carrier::EnvironmentalData;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    mutex::Mutex,
    once_lock::OnceLock,
    pubsub::{ImmediatePublisher, PubSubChannel},
    watch::{Sender, Watch},
};
use embassy_time::Instant;
use embedded_utils::ExtendTime;
use nalgebra::Vector2;

pub const CAP: usize = 10;
pub const PUB: usize = 0;
pub const SUB: usize = 2;
pub const WATCH: usize = 3;

const PRESSURE_TAU_S: f32 = 120.0;
const TH_TAU_S: f32 = 20.0;

pub static ENVIRONMENTAL_DRIVER_WATCH: Watch<ThreadModeRawMutex, EnvironmentalData, WATCH> =
    Watch::new();
pub static ENVIRONMENTAL_DRIVER_PUBSUB: PubSubChannel<
    ThreadModeRawMutex,
    EnvironmentalData,
    CAP,
    SUB,
    PUB,
> = PubSubChannel::new();
pub static ENVIRONMENTAL_DRIVER: OnceLock<EnvironmentalDriver> = OnceLock::new();

pub struct EnvironmentalDriver<'a> {
    publisher: ImmediatePublisher<'a, ThreadModeRawMutex, EnvironmentalData, CAP, SUB, PUB>,
    watch: Sender<'a, ThreadModeRawMutex, EnvironmentalData, WATCH>,

    th_filter: Mutex<CriticalSectionRawMutex, ExponentialMovingAverage<Vector2<f32>>>,
    pressure_filter: Mutex<CriticalSectionRawMutex, ExponentialMovingAverage<f32>>,

    last_pressure_ts: Mutex<CriticalSectionRawMutex, Option<Instant>>,
    last_th_ts: Mutex<CriticalSectionRawMutex, Option<Instant>>,
}

impl<'a> EnvironmentalDriver<'a> {
    pub fn new(
        publisher: ImmediatePublisher<'a, ThreadModeRawMutex, EnvironmentalData, CAP, SUB, PUB>,
        watch: Sender<'a, ThreadModeRawMutex, EnvironmentalData, WATCH>,
    ) -> Self {
        let dt = 1.0 / dht::SAMPLE_FREQUENCY_HZ as f32;
        let pressure_alpha = dt / (PRESSURE_TAU_S + dt);
        let th_alpha = dt / (TH_TAU_S + dt);

        Self {
            publisher,
            watch,
            th_filter: Mutex::new(ExponentialMovingAverage::new(th_alpha)),
            pressure_filter: Mutex::new(ExponentialMovingAverage::new(pressure_alpha)),
            last_pressure_ts: Mutex::new(None),
            last_th_ts: Mutex::new(None),
        }
    }

    pub async fn update_pressure_data(&self, pressure_mbar: f32, ts: Instant) {
        let dt_s = {
            let mut guard = self.last_pressure_ts.lock().await;
            let dt = if let Some(prev) = *guard {
                ts.duration_since(prev).as_secs_f32()
            } else {
                1.0 / dht::SAMPLE_FREQUENCY_HZ as f32
            };
            *guard = Some(ts);
            dt
        };

        let alpha = dt_s / (PRESSURE_TAU_S + dt_s);

        let filtered_p = {
            let mut g = self.pressure_filter.lock().await;
            g.update_with_alpha(pressure_mbar, alpha)
        };

        let th = self.th_filter.lock().await.current_value();
        self.publish(th, filtered_p);
    }

    pub async fn update_temp_hum_data(&self, temperature_c: f32, humidity_rh: f32, ts: Instant) {
        let dt_s = {
            let mut guard = self.last_th_ts.lock().await;
            let dt = if let Some(prev) = *guard {
                ts.duration_since(prev).as_secs_f32()
            } else {
                1.0 / dht::SAMPLE_FREQUENCY_HZ as f32
            };
            *guard = Some(ts);
            dt
        };

        let alpha = dt_s / (TH_TAU_S + dt_s);

        let filtered_th = {
            let mut g = self.th_filter.lock().await;
            g.update_with_alpha(Vector2::new(temperature_c, humidity_rh), alpha)
        };

        let pressure = self.pressure_filter.lock().await.current_value();
        self.publish(filtered_th, pressure);
    }

    fn publish(&self, th: Vector2<f32>, pressure: f32) {
        let fused = EnvironmentalData {
            temperature: Celsius(th.x),
            humidity: th.y,
            pressure: HPa(pressure),
        };
        self.publisher.publish_immediate(fused.clone());
        self.watch.send(fused);
    }
}
