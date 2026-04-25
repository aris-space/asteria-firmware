use crate::board::{CAMERA_SETTINGS_CHANGED, LIVESTREAM_CAMERA, RECORDING_CAMERA};
use crate::can_impl::CanReceiver;
use crate::can_impl::{CanError, CanTransmitter};
use crate::reset_now;
use crate::sensor_readout::{RAIL_5V_LAST, RAIL_24V_LAST};
use crate::unix_time::init_utc_clock;
use core::sync::atomic::Ordering;
use embassy_futures::select::select;
use embassy_stm32::can::{CanRx, CanTx};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use embedded_utils::ERROR_COUNT;
use embedded_utils::fmt::{error, info, trace};
use hermes_can::messages::board_status::{
    CameraPowerStatus, PowerBoardStatus, StatusCommonMessage,
};
use hermes_can::messages::{BoardId, Message};

const CAN_SEND_TIMEOUT: Duration = Duration::from_millis(100);
const CAN_ERROR_RETRY_DELAY: Duration = Duration::from_millis(100);

pub async fn spawn_can_tasks(
    spawner: &embassy_executor::Spawner,
    can_rx: CanRx<'static>,
    can_tx: CanTx<'static>,
) {
    static CAN_TX: OnceLock<Mutex<ThreadModeRawMutex, CanTx<'static>>> = OnceLock::new();
    CAN_TX
        .init(Mutex::new(can_tx))
        .ok()
        .expect("Failed to set CAN TX mutex");
    let can_tx = CAN_TX.get().await;

    // RX task
    spawner.spawn(can_rx_task(can_rx).expect("Failed to spawn CAN RX task"));

    // TX tasks
    spawner.spawn(can_5v_task(can_tx).expect("Failed to spawn CAN 5V task"));

    spawner.spawn(can_24v_task(can_tx).expect("Failed to spawn CAN 24V task"));

    spawner.spawn(board_status(can_tx).expect("Failed to spawn board status task"));

    spawner.spawn(camera_status(can_tx).expect("Failed to spawn camera status task"));

    spawner.spawn(build_information(can_tx).expect("Failed to spawn build information task"));
}
const CAN_TX_TIMEOUT: Duration = Duration::from_millis(100);

#[embassy_executor::task]
async fn can_24v_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    info!("Starting CAN 24V can task");
    let can_target_hz = 10.0;
    let alpha = 0.2;
    let min_period = Duration::from_millis((1000.0 / (can_target_hz * (1.0 + alpha))) as u64);
    let mut last_sent = Instant::now().saturating_sub(min_period); // Allow immediate send on first iteration

    let mut rail_status = RAIL_24V_LAST
        .receiver()
        .expect("failed to get pressure watch");

    loop {
        let rail_status = loop {
            if let Ok(p) = with_timeout(Duration::from_secs(10), rail_status.changed()).await {
                break p;
            }
            error!("Timeout waiting for 24V rail data");
        };

        let now = Instant::now();
        let can_data = rail_status;

        if now - last_sent >= min_period {
            let mut tx = can_tx.lock().await;
            match with_timeout(CAN_TX_TIMEOUT, tx.transmit(can_data.clone())).await {
                Ok(Ok(())) => {
                    trace!("Sent 24V rail data: {:?}", can_data);
                    last_sent = now;
                }
                Ok(Err(err)) => {
                    error!("CAN TX error: {:?}", err);
                }
                Err(_) => {
                    error!("CAN TX timed out after {} ms", CAN_TX_TIMEOUT.as_millis());
                }
            }
        } else {
            trace!(
                "Discarding 24V rail data: {:?}, because now={}ms - last_sent={}ms < min_period={}ms",
                can_data,
                now.as_millis(),
                last_sent.as_millis(),
                min_period.as_millis()
            );
        }
    }
}

#[embassy_executor::task]
async fn can_5v_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    info!("Starting CAN 5V can task");
    let can_target_hz = 10.0;
    let alpha = 0.2;
    let min_period = Duration::from_millis((1000.0 / (can_target_hz * (1.0 + alpha))) as u64);
    let mut last_sent = Instant::now().saturating_sub(min_period); // Allow immediate send on first iteration

    let mut voltage_watch = RAIL_5V_LAST
        .receiver()
        .expect("failed to get voltage watch");

    loop {
        let rail_status = loop {
            if let Ok(p) = with_timeout(Duration::from_secs(10), voltage_watch.changed()).await {
                break p;
            }
            error!("Timeout waiting for 5V rail data");
        };

        let now = Instant::now();
        let can_data = rail_status;

        if now - last_sent >= min_period {
            let mut tx = can_tx.lock().await;
            match with_timeout(CAN_TX_TIMEOUT, tx.transmit(can_data.clone())).await {
                Ok(Ok(())) => {
                    trace!("Sent 5V rail data: {:?}", can_data);
                    last_sent = now;
                }
                Ok(Err(err)) => {
                    error!("CAN TX error: {:?}", err);
                }
                Err(_) => {
                    error!("CAN TX timed out after {} ms", CAN_TX_TIMEOUT.as_millis());
                }
            }
        } else {
            trace!(
                "Discarding 5V rail data: {:?}, because now={}ms - last_sent={}ms < min_period={}ms",
                can_data,
                now.as_millis(),
                last_sent.as_millis(),
                min_period.as_millis()
            );
        }
    }
}

#[embassy_executor::task]
async fn camera_status(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    // pin handles
    let recording_camera = RECORDING_CAMERA.get().await;
    let live_camera = LIVESTREAM_CAMERA.get().await;

    // sent at 5 hz
    let mut ticker = Ticker::every(Duration::from_millis(200));

    loop {
        let rec_enabled = recording_camera.lock().await.is_set_high();
        let live_enabled = live_camera.lock().await.is_set_high();

        let camera_status = CameraPowerStatus {
            recording_pwr_enabled: rec_enabled,
            live_pwr_enabled: live_enabled,
        };

        // this scope is necessary to ensure the lock is released – do not remove it!
        {
            let mut tx = can_tx.lock().await;
            match with_timeout(CAN_SEND_TIMEOUT, tx.transmit(camera_status.clone())).await {
                Ok(Ok(())) => trace!("Sent CameraPowerStatus: {:?}", camera_status),
                Ok(Err(err)) => error!("Error sending CameraPowerStatus: {:?}", err),
                Err(_) => error!(
                    "Timeout sending CameraPowerStatus after {} ms",
                    CAN_SEND_TIMEOUT.as_millis()
                ),
            }
        }

        select(CAMERA_SETTINGS_CHANGED.wait(), ticker.next()).await;
    }
}

#[embassy_executor::task]
async fn board_status(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    // send once per second
    let mut ticker = Ticker::every(Duration::from_millis(1000));

    loop {
        let board_state = PowerBoardStatus {
            common: StatusCommonMessage {
                errors: ERROR_COUNT.load(Ordering::Relaxed),
                micros_since_restart: Instant::now().as_micros(),
            },
        };

        // this scope is necessary to ensure the lock is released – do not remove it!
        {
            let mut tx = can_tx.lock().await;
            match with_timeout(CAN_SEND_TIMEOUT, tx.transmit(board_state.clone())).await {
                Ok(Ok(())) => trace!("Sent PowerBoardStatus: {:?}", board_state),
                Ok(Err(err)) => error!("Error sending PowerBoardStatus: {:?}", err),
                Err(_) => error!(
                    "Timeout sending PowerBoardStatus after {} ms",
                    CAN_SEND_TIMEOUT.as_millis()
                ),
            }
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn build_information(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    let build_info = crate::build_info::BUILD_INFO.get();
    let build_info_msg = hermes_can::messages::debug_info::PowerBoardBuildInfo {
        data: build_info.clone(),
    };

    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        // this scope is necessary to ensure the lock is released – do not remove it!
        {
            let mut tx = can_tx.lock().await;
            match with_timeout(CAN_SEND_TIMEOUT, tx.transmit(build_info_msg.clone())).await {
                Ok(Ok(())) => trace!("Sent PowerBoardBuildInfo: {:?}", build_info_msg),
                Ok(Err(err)) => error!("Error sending PowerBoardBuildInfo: {:?}", err),
                Err(_) => error!(
                    "Timeout sending PowerBoardBuildInfo after {} ms",
                    CAN_SEND_TIMEOUT.as_millis()
                ),
            }
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn can_rx_task(mut rx: CanRx<'static>) -> ! {
    let rec_pin = RECORDING_CAMERA.get().await;
    let live_pin = LIVESTREAM_CAMERA.get().await;

    loop {
        match rx.recv().await {
            Ok((msg, ts)) => {
                trace!("Received can message: {:?}", msg);

                match msg {
                    Message::ResetSpecific(cmd) => {
                        if cmd.board_id == BoardId::PowerBoard {
                            info!("PowerBoard reset command received, rebooting now");
                            reset_now();
                        }
                    }
                    Message::ResetAll(_) => {
                        info!("Reset-All broadcast received, restarting board");
                        reset_now();
                    }
                    Message::UTCTimeUpdate(time) => {
                        init_utc_clock(time.timestamp_micros, ts);
                    }
                    Message::CameraPowerControl(cmd) => {
                        {
                            let mut rec_camera = rec_pin.lock().await;
                            rec_camera.set_level(cmd.recording_pwr_enabled.into());
                        }
                        {
                            let mut live_camera = live_pin.lock().await;
                            live_camera.set_level(cmd.live_pwr_enabled.into());
                        }
                        CAMERA_SETTINGS_CHANGED.signal(());
                    }
                    #[allow(unreachable_patterns)]
                    _ => {
                        error!("Unhandled message variant: {}", msg);
                    }
                }
            }
            Err(err) => {
                error!("CAN RX error: {:?}", err);
                if matches!(err, CanError::Bus(_) | CanError::Other) {
                    Timer::after(CAN_ERROR_RETRY_DELAY).await;
                }
            }
        }
    }
}
