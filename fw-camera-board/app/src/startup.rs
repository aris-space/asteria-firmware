use embassy_executor::Spawner;
use embassy_stm32::can::Can;
use embassy_stm32::gpio::Output;

use crate::runcam::RunCam;
use crate::{resources, tasks};

pub struct PreparedBoard {
    pub debug_led: Output<'static>,
    pub camera: RunCam,
    pub can: Can<'static>,
    pub address: u8,
}

pub fn prepare(resources: resources::AssignedResources) -> PreparedBoard {
    let debug_led = resources.debug_led.setup();
    let camera = resources.camera.setup();
    let can = resources.can_bus.setup();
    let address = resources.address.setup();

    PreparedBoard {
        debug_led,
        camera,
        can,
        address,
    }
}

pub fn spawn_tasks(board: PreparedBoard, thread_spawner: Spawner) {
    defmt::info!("camera board address {}", board.address);

    // only one led on this board, so we just show the build status
    thread_spawner.spawn(
        embedded_utils::led::build_status(
            board.debug_led,
            crate::build_info::BUILD_INFO.get(),
            false,
        )
        .expect("Failed to spawn LED task"),
    );
    thread_spawner.spawn(
        tasks::camera::task(board.camera, board.address).expect("Failed to spawn camera task"),
    );

    let (_, can_rx, _) = board.can.split();
    thread_spawner.spawn(tasks::can::rx(can_rx).expect("Failed to spawn CAN RX task"));
}
