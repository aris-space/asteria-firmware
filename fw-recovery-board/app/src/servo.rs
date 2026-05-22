/* Second try, construct a Servo struct that can enable, disable, detect actuator presence and set
   specific servo angles

   Inspired by code from Domenic Nebiker (meaning that I copied most of it)
*/
use cortex_m::prelude::_embedded_hal_Pwm;
use dp_recovery_board::ActuatorStatus;
use embassy_stm32::PeripheralType;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::{Channel, GeneralInstance4Channel};
use embedded_utils::fmt::*;
// idk if this is possible to do it nicer. Ill have to see

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ServoError {
    InvalidAngle,
    NotEnabled,
}

/// struct for handling the entirety of separation / deployment in one instance. Uses Servo struct defined in the same file.
/// needed to control both servo channels at the same time and should make code neater
pub struct RecoveryActuator<
    A: PeripheralType + GeneralInstance4Channel,
    B: PeripheralType + GeneralInstance4Channel,
> {
    act1: Servo<A>,
    act2: Servo<B>,
    act_pwr: Output<'static>,
    active: bool,
}
impl<A: PeripheralType + GeneralInstance4Channel, B: PeripheralType + GeneralInstance4Channel>
    RecoveryActuator<A, B>
{
    pub fn new(act1: Servo<A>, act2: Servo<B>, act_pwr: Output<'static>) -> Self {
        Self {
            act1,
            act2,
            act_pwr,
            active: false,
        }
    }

    /// Give power to and enable the redundant servo channels on the REC board
    pub fn activate_servo(&mut self) {
        self.act_pwr.set_high();
        self.act1.enable();
        self.act2.enable();
        self.active = true;
    }

    /// Remove power to and deactivate the redundant servo channels on the REC board
    pub fn deactivate_servo(&mut self) {
        self.act_pwr.set_low();
        self.act1.disable();
        self.act2.disable();
        self.active = false;
    }

    /// set the specified angle to both servos
    pub fn set_angle(&mut self, angle: f32) -> Result<(), ServoError> {
        match self.act1.set_angle(angle) {
            Ok(()) => {}
            Err(e) => {
                error!("error on act1: {}", e);

                // still attempt to set act2 to angle
                match self.act2.set_angle(angle) {
                    Ok(()) => {}
                    Err(e) => {
                        error!("error on act2: {}", e);
                        return Err(e);
                    }
                }
                return Err(e);
            }
        }
        match self.act2.set_angle(angle) {
            Ok(()) => {}
            Err(e) => {
                error!("error on act2: {}", e);
                return Err(e);
            }
        }
        Ok(())
    }

    pub fn get_actuator_status(&mut self) -> [ActuatorStatus; 2] {
        if self.active {
            return [ActuatorStatus::PowerOn, ActuatorStatus::PowerOn];
        }
        [
            self.act1.get_actuator_status(),
            self.act2.get_actuator_status(),
        ]
    }
}

/// Servo struct for handling the implementation of a single servo channel as it is implemented
/// on the REC board.
pub struct Servo<TIM: PeripheralType + GeneralInstance4Channel> {
    handle: SimplePwm<'static, TIM>,
    max_duty: u32,
    channel: Channel,
    actuator_presence: Input<'static>,
    active: bool,
}

impl<TIM: PeripheralType + GeneralInstance4Channel> Servo<TIM> {
    ///construct a new Servo instance with an actuator_presence detection pin
    pub fn new(
        handle: SimplePwm<'static, TIM>,
        channel: Channel,
        actuator_presence: Input<'static>,
    ) -> Self {
        let max_duty = handle.max_duty_cycle();
        Self {
            handle,
            max_duty,
            channel,
            actuator_presence,
            active: false,
        }
    }

    ///set servo to a specifc angle
    pub fn set_angle(&mut self, angle: f32) -> Result<(), ServoError> {
        if !self.active {
            return Err(ServoError::NotEnabled);
        }
        if angle > 180.0 {
            return Err(ServoError::InvalidAngle);
        }
        // For 333Hz servo with 500us (0 deg) to 2500us (180 deg) pulse width
        let min_pulse = 0.5; // in milliseconds
        let max_pulse = 2.5; // in milliseconds
        let freq_hz = 333.0; // Updated servo PWM frequency

        // Calculate the duty cycle range
        let period_ms = 1000.0 / freq_hz; // Convert frequency to period in milliseconds
        let min_duty = (min_pulse / period_ms) * self.max_duty as f32;
        let max_duty = (max_pulse / period_ms) * self.max_duty as f32;

        // Interpolate the duty cycle based on the angle
        let duty = min_duty + (angle / 180.0) * (max_duty - min_duty);

        // Set the PWM duty cycle

        self.handle.set_duty(self.channel, duty as u32);

        Ok(())
    }

    ///enable Servo
    pub fn enable(&mut self) {
        self.handle.enable(self.channel);
        self.active = true;
    }

    ///disable servo
    pub fn disable(&mut self) {
        self.handle.disable(self.channel);
        self.active = false;
    }

    ///returns true if an actuator is detected, false otherwise
    /// (Actuator is detected when ~1mA is pulled from the servo connector)
    pub fn is_connected(&self) -> bool {
        //maybe needs to be replaced by !self.actuator_presence.is_high(), as state might be undefined and as such not low and not high->might work,
        //must be tested ASAP
        self.actuator_presence.is_high()
    }

    ///return actuator status of a servo that has an actuator presence detection (only works when armed but not powered on)
    pub fn get_actuator_status(&self) -> ActuatorStatus {
        match self.is_connected() {
            true => ActuatorStatus::Connected,
            false => ActuatorStatus::NotConnected,
        }
    }
}
