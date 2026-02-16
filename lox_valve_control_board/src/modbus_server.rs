use bitflags::bitflags;
use core::sync::atomic::{AtomicU16, Ordering};
use embedded_utils::fmt::{error, info, trace, warn};

use crate::valve::{get_position_percent, move_to_position_percent, LOX_VALVE_MOTOR};
use crate::Db2F;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Error as UsartError, Uart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};
use embedded_io_async::Write;
use rmodbus::server::context::ModbusContext;
use rmodbus::server::storage::ModbusStorage;
use rmodbus::server::{Changes, ModbusFrame};
use rmodbus::{ModbusFrameBuf, ModbusProto};

const MODBUS_UNIT_ID: u8 = 101;

// Use 3 input registers and 2 holding registers
pub static MODBUS_CONTEXT: Mutex<ThreadModeRawMutex, ModbusStorage<0, 0, 3, 2>> =
    Mutex::new(ModbusStorage::<0, 0, 3, 2>::new());

pub mod registers {
    // Holding Registers

    /// Target position (read-write), f32, 0.0–1.0, where
    /// 0.0 is fully closed and 1.0 is fully open.
    pub const REG_MOTOR_TARGET_POSITION_RATIO: u16 = 0;

    // Input Registers

    /// Current position of the motor (read-only), f32, 0.0–1.0, where
    /// 0.0 is fully closed and 1.0 is fully open.
    pub const REG_CURRENT_MOTOR_POSITION_RATIO: u16 = 0;

    /// Device status (read-only), u16, bitmask:
    pub const REG_DEVICE_STATUS: u16 = 2;
}

static MOTOR_POSITION_WATCH: Watch<ThreadModeRawMutex, f32, 1> = Watch::new();

bitflags! {
    pub struct DeviceStatus: u16 {
        /// System is booting up and initializing peripherals.
        const INITIALIZING       = 1 << 0;
        /// System has initialized successfully and is running.
        const READY              = 1 << 1;
        /// A fatal, unrecoverable error occurred during motor initialization.
        const MOTOR_INIT_FAILED  = 1 << 2;
        /// A recoverable communication error with the motor controller occurred.
        /// This flag is cleared upon the next successful communication.
        const MOTOR_COMM_ERROR   = 1 << 3;
        /// The motor controller itself has reported an internal fault state.
        /// This flag is cleared after a successful fault clear command.
        const MOTOR_DEVICE_FAULT = 1 << 4;
    }
}

pub static DEVICE_STATUS: AtomicU16 = AtomicU16::new(0);

pub fn set_status(flags: DeviceStatus) {
    DEVICE_STATUS.fetch_or(flags.bits(), Ordering::Relaxed);
}

pub fn clear_status(flags: DeviceStatus) {
    DEVICE_STATUS.fetch_and(!flags.bits(), Ordering::Relaxed);
}

#[embassy_executor::task]
pub async fn modbus_server_task(usart: Uart<'static, Async>) {
    info!("Modbus server task started");
    let (mut tx, mut rx) = usart.split();
    let mut buf: ModbusFrameBuf = [0; 256];

    loop {
        match rx.read_until_idle(&mut buf).await {
            Ok(len) if len > 0 => {
                let mut response = heapless::Vec::<u8, 256>::new();
                response.resize_default(response.capacity()).unwrap();

                let mut frame =
                    ModbusFrame::new(MODBUS_UNIT_ID, &buf[..len], ModbusProto::Rtu, &mut response);

                if frame.parse().is_err() {
                    error!("Modbus frame parse error. Raw frame: {:?}", &buf[..len]);
                    continue;
                }

                if !frame.processing_required {
                    trace!("Modbus frame does not require processing.");
                    continue;
                }

                // There is NO timing issue for modbus here, since the MODBUS_CONTEXT is only held
                // elsewhere only very shortly for an update.
                let mut context_guard = MODBUS_CONTEXT.lock().await;

                let current_status = DEVICE_STATUS.load(Ordering::Relaxed);
                context_guard
                    .set_input(registers::REG_DEVICE_STATUS, current_status)
                    .expect("Failed to set Modbus input register REG_DEVICE_STATUS");

                if frame.readonly {
                    if let Err(e) = frame.process_read(&*context_guard) {
                        error!("Modbus read processing error: {}", Db2F(&e));
                    }
                } else if let Err(e) = frame.process_write(&mut *context_guard) {
                    error!("Modbus write processing error: {}", Db2F(&e));
                } else if let Some(Changes::Holdings { reg, count }) = frame.changes() {
                    let range = reg..(reg + count);
                    if range.contains(&registers::REG_MOTOR_TARGET_POSITION_RATIO) {
                        trace!("Modbus write to REG_MOTOR_POSITION_RATIO");
                        if let Ok(updated) = context_guard
                            .get_holdings_as_f32(registers::REG_MOTOR_TARGET_POSITION_RATIO)
                        {
                            MOTOR_POSITION_WATCH.sender().send(updated);
                        }
                    }
                }

                drop(context_guard);

                if frame.response_required {
                    if frame.finalize_response().is_ok() {
                        if let Err(e) = tx.write_all(&response).await {
                            warn!("Modbus response write error: {:?}", e);
                        }
                    } else {
                        error!("Failed to finalize Modbus error response.");
                    }
                }
                drop(response);
            }
            Ok(_) => {}
            Err(UsartError::BufferTooLong) => {
                warn!("Modbus read buffer too small for incoming frame.");
            }
            Err(e) => {
                warn!("Modbus UART read error: {:?}", e);
            }
        }
    }
}

#[embassy_executor::task]
pub async fn lox_valve_motor_controller_task() -> ! {
    let mut receiver = MOTOR_POSITION_WATCH
        .receiver()
        .expect("Failed to get receiver for motor position watch");

    let motor = LOX_VALVE_MOTOR.get().await;
    info!("Current motor controller task started");

    loop {
        let mut target_position = receiver.changed().await;
        {
            let mut motor = motor.lock().await;

            // Maybe the motor position was updated while we waited on the motor lock.
            // This way, we get the most frequent target position, even if we wait a long time
            if let Some(recent) = receiver.try_changed() {
                target_position = recent;
            }

            let err = move_to_position_percent(&mut motor, target_position, true).await;
            drop(motor);

            if let Err(e) = err {
                error!("Failed to move motor to target position: {:?}", e);
            } else {
                trace!("Motor moved to target position: {}", target_position);
            }
        }
    }
}

#[embassy_executor::task]
pub async fn lox_valve_motor_position_updater_task() -> ! {
    info!("Current motor position updater task started");
    let motor = LOX_VALVE_MOTOR.get().await;
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        let position_result = {
            let mut motor_guard = motor.lock().await;
            get_position_percent(&mut *motor_guard).await
        };

        match position_result {
            Ok(p) => {
                let mut context_guard = MODBUS_CONTEXT.lock().await;
                if context_guard
                    .set_inputs_from_f32(registers::REG_CURRENT_MOTOR_POSITION_RATIO, p)
                    .is_err()
                {
                    drop(context_guard);
                    error!(
                        "Failed to set Modbus input registers at addr {}: storage out of bounds?",
                        registers::REG_CURRENT_MOTOR_POSITION_RATIO
                    );
                }
            }
            Err(e) => {
                error!("Failed to read motor position for Modbus context: {:?}", e);
            }
        }

        ticker.next().await;
    }
}
