//! CAN bus: the RX task handles resets, one TX task per `dp-sensor-carrier`
//! message sends it at its rate.

use data_core::can::hal::CanDecode as _;
use datatypes::status::BoardId;
use embassy_executor::SendSpawner;
use embassy_stm32::can::{Can, RxFdBuf, TxFdBuf};
use embedded_can::StandardId;
use static_cell::StaticCell;

pub mod rx;
pub mod tx;

pub const THIS_BOARD_ID: BoardId = BoardId::SensorCarrier;
// One frame of every message, with room to spare.
const TX_BUFFER_LEN: usize = 16;
const RX_BUFFER_LEN: usize = 4;

// The only messages the Sensor Carrier receives.
data_core::can::sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
    }
}

/// The IDs of [`ReceivedMessage`], the only ones the hardware filter passes.
pub const RECEIVED_IDS: &[StandardId] = ReceivedMessage::SUPPORTED_IDS;

/// Switches `can` to buffered mode and spawns the RX task and every TX task,
/// each with its own handle to the TX buffer.
pub fn spawn(can: Can<'static>, spawner: SendSpawner) {
    static TX_BUFFER: StaticCell<TxFdBuf<TX_BUFFER_LEN>> = StaticCell::new();
    static RX_BUFFER: StaticCell<RxFdBuf<RX_BUFFER_LEN>> = StaticCell::new();
    let can = can.buffered_fd(
        TX_BUFFER.init(TxFdBuf::new()),
        RX_BUFFER.init(RxFdBuf::new()),
    );
    let tx = can.writer();
    spawner.spawn(rx::task(can.reader()).expect("Failed to spawn CAN RX task"));
    spawner.spawn(tx::orientation(tx.clone()).expect("Failed to spawn CAN orientation task"));
    spawner.spawn(tx::imu(tx.clone()).expect("Failed to spawn CAN IMU task"));
    spawner.spawn(tx::pressure(tx.clone()).expect("Failed to spawn CAN pressure task"));
    spawner.spawn(tx::position(tx.clone()).expect("Failed to spawn CAN position task"));
    spawner.spawn(tx::velocity(tx.clone()).expect("Failed to spawn CAN velocity task"));
    spawner.spawn(tx::magnetometer(tx.clone()).expect("Failed to spawn CAN magnetometer task"));
    spawner.spawn(tx::environmental(tx.clone()).expect("Failed to spawn CAN environmental task"));
    spawner.spawn(tx::status(tx.clone()).expect("Failed to spawn CAN status task"));
    spawner.spawn(tx::build_info(tx).expect("Failed to spawn CAN build info task"));
}
