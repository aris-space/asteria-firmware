use crate::globals::STATE;
use ads1x1x::{Ads1x1x, DataRate12Bit, FullScaleRange, TargetAddr, channel};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Ticker, Timer};
use embedded_utils::fmt::*;

const SOLENOID_CURRENT_ACQ_PERIOD: Duration = Duration::from_millis(100);
const MAX_READ_ATTEMPTS: usize = 3;
const FULL_SCALE_MILLIVOLTS: f32 = 4096.0;
const ADS1015_COUNTS: f32 = 2048.0;

// Replace with the actual transfer function of the solenoid current-sense circuit.
const CURRENT_SENSE_ZERO_MILLIVOLTS: f32 = 0.0;
const CURRENT_SENSE_MILLIVOLTS_PER_AMP: f32 = 500.0;

type Ads1015Device = Ads1x1x<
    I2c<'static, Async, i2c::mode::Master>,
    ads1x1x::ic::Ads1015,
    ads1x1x::ic::Resolution12Bit,
    ads1x1x::mode::OneShot,
>;

#[derive(Clone, Copy)]
struct AdcSample {
    raw_counts: u16,
    millivolts: f32,
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
pub struct SolenoidCurrentMeasurement {
    pub raw_counts: u16,
    pub millivolts: f32,
    pub amps: f32,
}

impl SolenoidCurrentMeasurement {
    fn from_sample(sample: AdcSample) -> Self {
        Self {
            raw_counts: sample.raw_counts,
            millivolts: sample.millivolts,
            amps: (sample.millivolts - CURRENT_SENSE_ZERO_MILLIVOLTS)
                / CURRENT_SENSE_MILLIVOLTS_PER_AMP,
        }
    }
}

#[derive(Clone, Copy)]
pub struct SolenoidCurrentMeasurements {
    pub dpr: SolenoidCurrentMeasurement,
    pub pressurization_vent: SolenoidCurrentMeasurement,
    pub fuel_vent: SolenoidCurrentMeasurement,
}

#[embassy_executor::task]
pub async fn solenoid_current_task(i2c: I2c<'static, Async, i2c::mode::Master>) -> ! {
    let mut ads1015 = Ads1x1x::new_ads1015(i2c, TargetAddr::default());
    ads1015
        .set_full_scale_range(FullScaleRange::Within4_096V)
        .ok();
    ads1015.set_data_rate(DataRate12Bit::Sps1600).ok();

    let sender = STATE.solenoid_currents.sender();
    let mut ticker = Ticker::every(SOLENOID_CURRENT_ACQ_PERIOD);
    let mut last_currents: Option<SolenoidCurrentMeasurements> = None;

    loop {
        let dpr = read_current_channel(&mut ads1015, CurrentChannel::A1).await;
        let pressurization_vent = read_current_channel(&mut ads1015, CurrentChannel::A3).await;
        let fuel_vent = read_current_channel(&mut ads1015, CurrentChannel::A2).await;

        match (dpr, pressurization_vent, fuel_vent, last_currents) {
            (Some(dpr), Some(pressurization_vent), Some(fuel_vent), _) => {
                let currents = SolenoidCurrentMeasurements {
                    dpr: SolenoidCurrentMeasurement::from_sample(dpr),
                    pressurization_vent: SolenoidCurrentMeasurement::from_sample(
                        pressurization_vent,
                    ),
                    fuel_vent: SolenoidCurrentMeasurement::from_sample(fuel_vent),
                };

                sender.send(currents);
                info!(
                    "[ADS1015] Solenoid currents: DPR={} A, PRZ vent={} A, fuel vent={} A",
                    currents.dpr.amps, currents.pressurization_vent.amps, currents.fuel_vent.amps
                );
                last_currents = Some(currents);
            }
            (dpr, pressurization_vent, fuel_vent, Some(mut currents)) => {
                if let Some(dpr) = dpr {
                    currents.dpr = SolenoidCurrentMeasurement::from_sample(dpr);
                }
                if let Some(pressurization_vent) = pressurization_vent {
                    currents.pressurization_vent =
                        SolenoidCurrentMeasurement::from_sample(pressurization_vent);
                }
                if let Some(fuel_vent) = fuel_vent {
                    currents.fuel_vent = SolenoidCurrentMeasurement::from_sample(fuel_vent);
                }

                sender.send(currents);
                last_currents = Some(currents);
                warn!("[ADS1015] Reused last good value for one or more solenoid current channels");
            }
            _ => {
                error!("[ADS1015] Failed to read solenoid currents");
            }
        }

        ticker.next().await;
    }
}

#[derive(Clone, Copy)]
enum CurrentChannel {
    A1,
    A2,
    A3,
}

async fn read_current_channel(
    ads1015: &mut Ads1015Device,
    channel: CurrentChannel,
) -> Option<AdcSample> {
    for _ in 0..MAX_READ_ATTEMPTS {
        let result = match channel {
            CurrentChannel::A1 => ads1015.read(channel::SingleA1),
            CurrentChannel::A2 => ads1015.read(channel::SingleA2),
            CurrentChannel::A3 => ads1015.read(channel::SingleA3),
        };

        match result {
            Ok(raw_counts) => {
                return Some(AdcSample {
                    raw_counts: raw_counts as u16,
                    millivolts: raw_counts as f32 * FULL_SCALE_MILLIVOLTS / ADS1015_COUNTS,
                });
            }
            Err(nb::Error::WouldBlock) => Timer::after_millis(1).await,
            Err(nb::Error::Other(_)) => Timer::after_millis(1).await,
        }
    }

    None
}
