use crate::controls::THRUST_CURVE;
use crate::controls::actions::ActionCompleteness;
use crate::controls::actions::detect::SENSOR_TIMEOUT;
use crate::controls::controller::{CONTROL_TIME_STEP_MS, PIDController, PIDGains};
use crate::drivers::WATCH;
use crate::drivers::digital_pressure::DIGITAL_PRESSURE_WATCH;
use crate::valves::{EXTERNAL_VALVE_CONTROL, ExternalValve};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Instant, Timer, with_timeout};
use embedded_utils::{error, info};
use hermes_can::messages::event_messages::{DprState, FiringAbortInitiation};

const FSS_GAINS: PIDGains = PIDGains {
    kp: 1.216,
    ki: 14.716,
    kd: 0.0,
};
const OSS_GAINS: PIDGains = PIDGains {
    kp: 1.533,
    ki: 15.257,
    kd: 0.0,
};

pub async fn follow_thrust_curve<'a>(
    abort_initiation_receiver: &mut Receiver<
        'a,
        CriticalSectionRawMutex,
        FiringAbortInitiation,
        WATCH,
    >,
) -> ActionCompleteness {
    // Get the valve control channel
    // We use an immediate publisher here to make sure even if the queue is full we update the valve state
    // and proceed with calculating the next state
    let valve_channel = EXTERNAL_VALVE_CONTROL.immediate_publisher();

    // Get the thrust curve points
    let guard = match THRUST_CURVE.try_get() {
        Some(guard) => guard,
        None => {
            error!("Thrust curve not initialized, cannot follow thrust curve");
            return ActionCompleteness::Failed;
        }
    };

    let curve = { *guard.lock().await };
    let mut points = curve.points[..curve.length as usize].iter();

    // Ensure there are at least two points to interpolate between
    if points.len() < 2 {
        error!("Thrust curve has less than 2 points, cannot follow thrust curve");
        return ActionCompleteness::Failed;
    }

    // Unwrap is safe here because we checked length above
    let mut current_point = points.next().unwrap();
    let mut next_point = points.next().unwrap();

    // Initialize PID controllers for FSS and OSS
    let mut fss_controller = PIDController::new(FSS_GAINS);
    let mut oss_controller = PIDController::new(OSS_GAINS);

    // Get Injector pressures
    let mut injector_pressures = DIGITAL_PRESSURE_WATCH.receiver().unwrap();

    let start = Instant::now();
    loop {
        // Check for abort signal
        if abort_initiation_receiver.try_changed().is_some() {
            return ActionCompleteness::Failed;
        }

        // Calculate elapsed time
        let elapsed = start.elapsed().as_millis() as u32;

        // Calculate the interpolated setpoints pressures
        let total_time = next_point.absolute_time_ms - current_point.absolute_time_ms;
        let time_into_segment = elapsed - current_point.absolute_time_ms;

        let fuel_pressure_setpoint = current_point.fuel_p
            + (next_point.fuel_p - current_point.fuel_p)
                * (time_into_segment as f32 / total_time as f32);
        let oxidizer_pressure_setpoint = current_point.oxidizer_p
            + (next_point.oxidizer_p - current_point.oxidizer_p)
                * (time_into_segment as f32 / total_time as f32);

        // Get current injector pressures with a timeout
        let (fuel_pressure_reference, oxidizer_pressure_reference) =
            if let Ok(data) = with_timeout(SENSOR_TIMEOUT, injector_pressures.get()).await {
                (data.fue_inj_p, data.oxd_inj_p)
            } else {
                return ActionCompleteness::Failed;
            };
        //info!("[FIRING] Injector Setpoint: FSS: {}barg, OSS: {}barg", fuel_pressure_setpoint, oxidizer_pressure_setpoint);

        // Update tank setpoints using PID control
        let fss_tnk_p = fss_controller.step(fuel_pressure_setpoint, fuel_pressure_reference);
        let oss_tnk_p =
            oss_controller.step(oxidizer_pressure_setpoint, oxidizer_pressure_reference);

        // Update the pressure setpoints
        valve_channel.publish_immediate(ExternalValve::FuelDpr(DprState::Enabled(fss_tnk_p)));
        valve_channel.publish_immediate(ExternalValve::OxidizerDpr(DprState::Enabled(oss_tnk_p)));
        info!(
            "[FIRING] Updated Tank Setpoints: FSS: {}barg, OSS: {}barg",
            fss_tnk_p, oss_tnk_p
        );

        // Move to the next segment if we've reached or passed the next point's time
        if elapsed >= next_point.absolute_time_ms {
            current_point = next_point;
            next_point = if let Some(p) = points.next() {
                info!("[FIRING] Moving to next thrust curve point: {:?}", p);
                p
            } else {
                // Reached end of curve
                return ActionCompleteness::Successful;
            };
        }
        Timer::after_millis(CONTROL_TIME_STEP_MS as u64).await;
    }
}
