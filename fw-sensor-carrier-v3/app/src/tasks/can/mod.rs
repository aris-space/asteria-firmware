//! CAN bus through `can-utils`: the publishers in [`tx`] fill [`OUTPUTS`],
//! whose broadcast tasks send each value at its message's rate; the RX task
//! handles resets.

use can_utils::broadcast::Broadcast as _;
use data_core::can::hal::CanDecode as _;
use datatypes::status::{BoardId, BuildInformationCommon};
use datatypes::units::HPa;
use dp_sensor_carrier::{
    EnvironmentalData, ImuData, MagnetometerData, Message, OrientationData, PositionData,
    SensorCarrierStatus, VelocityData,
};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::can::{Can, CanTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embedded_can::StandardId;

pub mod rx;
pub mod tx;

pub const THIS_BOARD_ID: BoardId = BoardId::SensorCarrier;

// The only messages the Sensor Carrier receives.
data_core::can::sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
    }
}

/// The IDs of [`ReceivedMessage`], the only ones the hardware filter passes.
pub const RECEIVED_IDS: &[StandardId] = ReceivedMessage::SUPPORTED_IDS;

// max_freq_hz is 1.2 times the message's specified rate, so jitter never
// throttles it; min_freq_hz resends the last value when nothing new arrives.
#[derive(can_utils::broadcast::Broadcast)]
#[broadcast(loop_type = "can_utils::broadcast::ResponsiveLoop")]
pub struct Outputs {
    #[broadcast(
        map = "Message::OrientationData(#value)",
        min_freq_hz = 0.1,
        max_freq_hz = 48.0
    )]
    pub orientation: Watch<CriticalSectionRawMutex, OrientationData, 1>,

    #[broadcast(
        map = "Message::ImuData(#value)",
        min_freq_hz = 0.1,
        max_freq_hz = 48.0
    )]
    pub inertial: Watch<CriticalSectionRawMutex, ImuData, 1>,

    #[broadcast(
        map = "Message::PressureData(#value)",
        min_freq_hz = 0.1,
        max_freq_hz = 48.0
    )]
    pub pressure: Watch<CriticalSectionRawMutex, HPa, 1>,

    #[broadcast(
        map = "Message::PositionData(#value)",
        min_freq_hz = 0.1,
        max_freq_hz = 24.0
    )]
    pub position: Watch<CriticalSectionRawMutex, PositionData, 1>,

    #[broadcast(
        map = "Message::VelocityData(#value)",
        min_freq_hz = 0.1,
        max_freq_hz = 24.0
    )]
    pub velocity: Watch<CriticalSectionRawMutex, VelocityData, 1>,

    #[broadcast(
        map = "Message::MagnetometerData(#value)",
        min_freq_hz = 0.1,
        max_freq_hz = 12.0
    )]
    pub magnetic_field: Watch<CriticalSectionRawMutex, MagnetometerData, 1>,

    #[broadcast(
        map = "Message::EnvironmentalData(#value)",
        min_freq_hz = 0.1,
        max_freq_hz = 1.2
    )]
    pub environmental: Watch<CriticalSectionRawMutex, EnvironmentalData, 1>,

    #[broadcast(
        map = "Message::BoardStatus(#value)",
        min_freq_hz = 1.0,
        max_freq_hz = 1.2
    )]
    pub status: Watch<CriticalSectionRawMutex, SensorCarrierStatus, 1>,

    #[broadcast(
        map = "Message::BuildInfo(#value)",
        min_freq_hz = 0.2,
        max_freq_hz = 0.24
    )]
    pub build_info: Watch<CriticalSectionRawMutex, BuildInformationCommon, 1>,
}

pub static OUTPUTS: Outputs = Outputs {
    orientation: Watch::new(),
    inertial: Watch::new(),
    pressure: Watch::new(),
    position: Watch::new(),
    velocity: Watch::new(),
    magnetic_field: Watch::new(),
    environmental: Watch::new(),
    status: Watch::new(),
    build_info: Watch::new(),
};

pub fn spawn(can: Can<'static>, spawner: SendSpawner) {
    let (can_tx, can_rx, _properties) = can.split();
    spawner.spawn(rx::task(can_rx).expect("Failed to spawn CAN RX task"));
    spawner.spawn(start_broadcasting(can_tx).expect("Failed to spawn CAN broadcast start task"));
    spawner.spawn(tx::navigation().expect("Failed to spawn CAN navigation publisher"));
    spawner.spawn(tx::pressure().expect("Failed to spawn CAN pressure publisher"));
    spawner.spawn(tx::magnetic_field().expect("Failed to spawn CAN magnetic field publisher"));
    spawner.spawn(tx::environmental().expect("Failed to spawn CAN environmental publisher"));
    spawner.spawn(tx::status().expect("Failed to spawn CAN status publisher"));
    spawner.spawn(tx::build_info().expect("Failed to spawn CAN build info publisher"));
}

/// Starts the broadcast tasks on this executor; `start_broadcasting` takes a
/// `Spawner`, which only a task running on the executor can get.
#[embassy_executor::task]
async fn start_broadcasting(can_tx: CanTx<'static>) {
    // SAFETY: this runs in an embassy task, polled with the executor's own
    // context; embassy names an InterruptExecutor's Spawner as this call's use.
    let spawner = unsafe { Spawner::for_current_executor() }.await;
    OUTPUTS
        .start_broadcasting(spawner, can_utils::setup::make_multiplexable(can_tx))
        .expect("Failed to start CAN broadcasting");
}
