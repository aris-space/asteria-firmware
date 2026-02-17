use crate::Db2F;
use crate::modbus_server::{DeviceStatus, clear_status, set_status};
use core::fmt::Debug;
use core::future::Future;
use core::sync::atomic::{AtomicI32, Ordering};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, TimeoutError, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use embedded_utils::{error, info};
use epos4::{Epos4, Epos4Error, EposStatus};
use static_cell::StaticCell;

const ENCODER_COUNTS_PER_REV: i32 = 4096;
const GEAR_RATIO: i32 = 156;
const COUNTS_PER_OUTPUT_REV: i32 = ENCODER_COUNTS_PER_REV * GEAR_RATIO;
const QUARTER_REVOLUTION_COUNTS: i32 = COUNTS_PER_OUTPUT_REV / 4;
const VALVE_OPEN_POSITION: i32 = QUARTER_REVOLUTION_COUNTS;
#[allow(dead_code)]
const VALVE_CLOSED_POSITION: i32 = 0;
const TOLERANCE: f32 = 0.00;
const VALVE_OPEN_LIMIT: i32 = ((1.0 + TOLERANCE) * VALVE_OPEN_POSITION as f32) as i32;
const VALVE_CLOSED_LIMIT: i32 = (-TOLERANCE * VALVE_OPEN_POSITION as f32) as i32;

pub static HOME_POSITION_OFFSET: AtomicI32 = AtomicI32::new(0);

#[inline]
fn home_offset() -> i32 {
    HOME_POSITION_OFFSET.load(Ordering::SeqCst)
}

#[inline]
fn rel_to_abs(rel_counts: i32) -> i32 {
    rel_counts + home_offset()
}

#[inline]
fn abs_to_rel(abs_counts: i32) -> i32 {
    abs_counts - home_offset()
}

pub type SharedMotor = Mutex<
    ThreadModeRawMutex,
    Epos4<usart::RingBufferedUartRx<'static>, usart::UartTx<'static, Async>>,
>;

static UART_RX_BUF: StaticCell<[u8; 512]> = StaticCell::new();
pub static LOX_VALVE_MOTOR: OnceLock<SharedMotor> = OnceLock::new();

const MAXON_MOTOR_TIMEOUT: Duration = Duration::from_millis(500);

/// Helper: on any EPOS error, set appropriate status + log.
fn record_epos_error<E: Debug>(ctx: &str, e: &Epos4Error<E>) {
    match e {
        Epos4Error::DeviceInFaultState => set_status(DeviceStatus::MOTOR_DEVICE_FAULT),
        _ => set_status(DeviceStatus::MOTOR_COMM_ERROR),
    }
    error!("{} failed: {:?}", ctx, Db2F(e));
}

/// Helper: on success clear comm error.
fn record_comm_ok() {
    clear_status(DeviceStatus::MOTOR_COMM_ERROR);
}

/// Helper: run an EPOS future with standard timeout + status handling.
/// - `ctx` appears in logs.
/// - On timeout: sets MOTOR_COMM_ERROR and returns `Epos4Error::Timeout`.
async fn epos_op_with_timeout<T, F, E>(ctx: &str, fut: F) -> Result<T, Epos4Error<E>>
where
    F: Future<Output = Result<T, Epos4Error<E>>>,
    E: Debug,
{
    match with_timeout(MAXON_MOTOR_TIMEOUT, fut).await {
        Ok(Ok(v)) => {
            record_comm_ok();
            Ok(v)
        }
        Ok(Err(e)) => {
            record_epos_error(ctx, &e);
            Err(e)
        }
        Err(TimeoutError) => {
            set_status(DeviceStatus::MOTOR_COMM_ERROR);
            error!("{} timed out", ctx);
            Err(Epos4Error::Timeout)
        }
    }
}

pub async fn init_lox_valve_motor(
    uart: Uart<'static, Async>,
    mut endstop: ExtiInput<'_>,
) -> Result<(), Epos4Error<usart::Error>> {
    info!("Initializing valve motor");
    let (tx, rx) = uart.split();
    let rx_buf = UART_RX_BUF.init([0; 512]);
    let mut rx_buffered = rx.into_ring_buffered(rx_buf);
    rx_buffered.start_uart();

    let mut motor = Epos4::new(rx_buffered, tx, 0x1);

    if let Err(e) = wait_for_motor_ready(&mut motor).await {
        set_status(DeviceStatus::MOTOR_INIT_FAILED);
        error!("Failed to initialize motor: {:?}", Db2F(&e));
        return Err(e);
    }

    info!("now setting motor parameters...");

    // Set PPM profile with standardized timeout handling.
    epos_op_with_timeout("Set PPM profile", motor.set_ppm_profile(7142, 5000, 5000)).await?;

    // Switch to PPM mode.
    epos_op_with_timeout("Set PPM mode", motor.set_ppm_mode()).await?;
    // Enable power stage.
    epos_op_with_timeout("Enable motor", motor.enable()).await?;

    // Find and calibrate home
    find_and_calibrate_home(&mut motor, &mut endstop).await?;

    LOX_VALVE_MOTOR
        .init(Mutex::new(motor))
        .ok()
        .expect("init_lox_valve_motor must only be called once");

    info!("LOX Valve Motor initialized successfully.");
    Ok(())
}

/// Finds and sets the home position of the motor with the help of a limit switch
/// Moves the motor anti-clockwise until the limit switch is hit.
async fn find_and_calibrate_home<R, W, E>(
    motor: &mut Epos4<R, W>,
    limit_switch: &mut ExtiInput<'_>,
) -> Result<(), Epos4Error<E>>
where
    R: Read<Error = E>,
    W: Write<Error = E>,
    E: Debug,
{
    info!("Starting home calibration");

    loop {
        if limit_switch.is_low() {
            break;
        }
        let cur_abs = motor.get_current_position().await?;
        let target_abs = cur_abs - 750;
        motor.move_to_position(target_abs, true).await?;
    }

    // get current position and stop
    let cur_abs = motor.get_current_position().await?;
    motor.move_to_position(cur_abs, true).await?;

    HOME_POSITION_OFFSET.store(cur_abs, Ordering::SeqCst);
    info!(
        "Homing complete, home position offset (abs counts): {}",
        cur_abs
    );

    Ok(())
}

async fn wait_for_motor_ready<R, W, E>(motor: &mut Epos4<R, W>) -> Result<(), Epos4Error<E>>
where
    R: Read<Error = E>,
    W: Write<Error = E>,
    E: Debug,
{
    info!("Checking motor state...");

    for attempt in 1u32.. {
        match epos_op_with_timeout("Get status", motor.get_status()).await {
            Ok(status) => {
                info!("[Attempt {}] Motor status: {:?}", attempt, status);
                match status {
                    EposStatus::SwitchOnDisabled
                    | EposStatus::ReadyToSwitchOn
                    | EposStatus::SwitchedOn
                    | EposStatus::OperationEnabled => {
                        info!("Motor is ready. Status: {:?}", status);
                        clear_status(DeviceStatus::MOTOR_DEVICE_FAULT);
                        return Ok(());
                    }
                    EposStatus::Fault => {
                        set_status(DeviceStatus::MOTOR_DEVICE_FAULT);
                        info!("Motor in Fault state. Attempting to clear...");
                        // Reset node if possible; errors/timeouts are logged by helper.
                        let _ = epos_op_with_timeout("Reset node", motor.reset_node()).await;
                    }
                    _ => {
                        info!("Motor not yet ready, waiting...");
                    }
                }
            }
            Err(e) => {
                // Error already logged and status set by helper; attempt reset.
                info!(
                    "[Attempt {}] get_status error: {:?}. Attempting node reset.",
                    attempt,
                    Db2F(&e)
                );
                let _ = epos_op_with_timeout("Reset node", motor.reset_node()).await;
                Timer::after(Duration::from_millis(500)).await;
            }
        }

        Timer::after(Duration::from_millis(200)).await;
    }

    // Unreachable due to infinite loop, but keep signature happy.
    #[allow(unreachable_code)]
    Ok(())
}

pub async fn move_to_position_percent<R, W, E>(
    motor: &mut Epos4<R, W>,
    ratio: f32,
    immediate: bool,
) -> Result<(), Epos4Error<E>>
where
    R: Read<Error = E>,
    W: Write<Error = E>,
    E: Debug,
{
    // Compute desired *relative* position from percentage.
    let ratio = ratio.clamp(0.0, 1.0);
    let rel_range = (VALVE_OPEN_LIMIT - VALVE_CLOSED_LIMIT) as f32;
    let rel_position = (VALVE_CLOSED_LIMIT as f32 + rel_range * ratio) as i32;

    // Convert to *absolute* counts using home offset and command the drive.
    let abs_position = rel_to_abs(rel_position);

    epos_op_with_timeout(
        "Move to position",
        motor.move_to_position(abs_position, immediate),
    )
    .await
}

pub async fn get_position_percent<R, W, E>(motor: &mut Epos4<R, W>) -> Result<f32, Epos4Error<E>>
where
    R: Read<Error = E>,
    W: Write<Error = E>,
    E: Debug,
{
    // Read *absolute* position from the drive.
    let abs_pos =
        epos_op_with_timeout("Get current position", motor.get_current_position()).await?;

    // Convert to *relative* position w.r.t. home.
    let rel_pos = abs_to_rel(abs_pos);

    // Map to [0,1] using relative limits.
    let rel_range = (VALVE_OPEN_LIMIT - VALVE_CLOSED_LIMIT) as f32;
    if rel_range.abs() < 1e-6 {
        return Ok(0.0);
    }
    let ratio = (rel_pos as f32 - VALVE_CLOSED_LIMIT as f32) / rel_range;
    Ok(ratio.clamp(0.0, 1.0))
}
