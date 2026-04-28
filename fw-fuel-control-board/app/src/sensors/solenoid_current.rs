use crate::drivers::ads1015::{AdcSample, Ads1015, SingleEndedChannel};
use crate::globals::STATE;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Ticker};
use embedded_utils::fmt::*;

const SOLENOID_CURRENT_ACQ_PERIOD: Duration = Duration::from_millis(100);

// Replace with the actual transfer function of the solenoid current-sense circuit.
const CURRENT_SENSE_ZERO_MILLIVOLTS: f32 = 0.0;
const CURRENT_SENSE_MILLIVOLTS_PER_AMP: f32 = 500.0;

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
pub async fn solenoid_current_task(ads1015: Ads1015<I2c<'static, Async, i2c::mode::Master>>) -> ! {
    let mut ads1015 = ads1015;
    let sender = STATE.solenoid_currents.sender();
    let mut ticker = Ticker::every(SOLENOID_CURRENT_ACQ_PERIOD);

    loop {
        let dpr = ads1015.read_single_ended(SingleEndedChannel::A0).await;
        let pressurization_vent = ads1015.read_single_ended(SingleEndedChannel::A1).await;
        let fuel_vent = ads1015.read_single_ended(SingleEndedChannel::A2).await;

        match (dpr, pressurization_vent, fuel_vent) {
            (Ok(dpr), Ok(pressurization_vent), Ok(fuel_vent)) => {
                let currents = SolenoidCurrentMeasurements {
                    dpr: SolenoidCurrentMeasurement::from_sample(dpr),
                    pressurization_vent: SolenoidCurrentMeasurement::from_sample(
                        pressurization_vent,
                    ),
                    fuel_vent: SolenoidCurrentMeasurement::from_sample(fuel_vent),
                };

                sender.send(currents);
                trace!(
                    "[ADS1015] Solenoid currents: DPR={} A, PRZ vent={} A, fuel vent={} A",
                    currents.dpr.amps, currents.pressurization_vent.amps, currents.fuel_vent.amps
                );
            }
            _ => {
                error!("[ADS1015] Failed to read one or more solenoid current channels");
            }
        }

        ticker.next().await;
    }
}
