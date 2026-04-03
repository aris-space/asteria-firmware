use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;

use crate::resources::buses::SharedI2cBus;
use crate::resources::sensors::SpiDevice;
use crate::{resources, tasks};

pub struct PreparedBoard {
    pub services: ServiceResources,
    pub sensors: SensorResources,
}

pub struct ServiceResources {
    pub green_led: Output<'static>,
    pub yellow_led: Output<'static>,
    pub red_led: Output<'static>,
}

pub struct SensorResources {
    pub gps1_rx: UartRx<'static, Async>,
    pub gps2_rx: UartRx<'static, Async>,
    pub imu1: (SpiDevice, ExtiInput<'static, Async>),
    pub imu2: (SpiDevice, ExtiInput<'static, Async>),
    pub bus1: SharedI2cBus,
    pub bus2: SharedI2cBus,
}

pub fn prepare(resources: resources::AssignedResources) -> PreparedBoard {
    let gps1_data = resources.gps1_uart.setup();
    let (_gps1_tx, gps1_rx) = gps1_data.split();

    let gps2_data = resources.gps2_uart.setup();
    let (_gps2_tx, gps2_rx) = gps2_data.split();

    let imu1 = resources.imu1.setup();
    let imu2 = resources.imu2.setup();

    let green_led = resources.green_led.setup();
    let yellow_led = resources.yellow_led.setup();
    let red_led = resources.red_led.setup();

    let bus1 = resources.bus1.setup();
    let bus2 = resources.bus2.setup();

    PreparedBoard {
        services: ServiceResources {
            green_led,
            yellow_led,
            red_led,
        },
        sensors: SensorResources {
            gps1_rx,
            gps2_rx,
            imu1,
            imu2,
            bus1,
            bus2,
        },
    }
}

pub fn spawn_tasks(
    board: PreparedBoard,
    level_t_spawner: Spawner,
    level_0_spawner: SendSpawner,
) {
    // Blinky on interrupt executor for reliable timing
    level_0_spawner.must_spawn(tasks::blinky::task(board.services.yellow_led));

    // IMU tasks on thread executor
    let (imu1_spi, imu1_int1) = board.sensors.imu1;
    let (imu2_spi, imu2_int1) = board.sensors.imu2;
    level_t_spawner.must_spawn(tasks::imu::task(imu1_spi, imu1_int1));
    level_t_spawner.must_spawn(tasks::imu::task(imu2_spi, imu2_int1));

    // Barometer tasks on thread executor (one per I2C bus)
    level_t_spawner.must_spawn(tasks::barometer::task(board.sensors.bus1));
    level_t_spawner.must_spawn(tasks::barometer::task(board.sensors.bus2));
}
