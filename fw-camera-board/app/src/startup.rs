use embassy_executor::Spawner;
use embassy_stm32::can::Can;
use embassy_stm32::gpio::Output;

use crate::{resources, tasks};

pub struct PreparedBoard {
    pub debug_led: Output<'static>,
    pub camera_power: Output<'static>,
    pub can: Can<'static>,
    pub address: u8,
}

pub async fn prepare(resources: resources::AssignedResources) -> PreparedBoard {
    let debug_led = resources.debug_led.setup();
    let camera_power = resources.camera_power.setup();
    let can = resources.can_bus.setup();
    let address = resources.address.read().await;

    PreparedBoard {
        debug_led,
        camera_power,
        can,
        address,
    }
}

pub fn spawn_tasks(board: PreparedBoard, thread_spawner: Spawner) {
    defmt::info!("camera board address {}", board.address);

    thread_spawner.spawn(tasks::leds::task(board.debug_led).expect("Failed to spawn LED task"));
    thread_spawner.spawn(
        tasks::camera::task(board.camera_power, board.address)
            .expect("Failed to spawn camera task"),
    );

    let (_, can_rx, _) = board.can.split();
    thread_spawner.spawn(tasks::can::rx(can_rx).expect("Failed to spawn CAN RX task"));
}
