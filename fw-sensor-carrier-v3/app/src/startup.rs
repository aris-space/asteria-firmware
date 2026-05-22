use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_time::Duration;

use crate::resources::buses::SharedI2cBus;
use crate::resources::sensors::SpiDevice;
use crate::sensors::{
    BAROMETER_0, BAROMETER_1, DHT_0, DHT_1, GNSS_0, GNSS_1, IMU_0, IMU_1, MAGNETOMETER_0,
    MAGNETOMETER_1,
};

use crate::{resources, tasks};

const BAROMETER_DELAY: Duration = Duration::from_millis(20); // TODO: calibrate
const MAGNETOMETER_DELAY: Duration = Duration::from_millis(0); // TODO: calibrate
const GNSS_DELAY: Duration = Duration::from_millis(100); // TODO: calibrate
const DHT_DELAY: Duration = Duration::from_millis(0);

#[allow(dead_code)]
pub struct PreparedBoard {
    pub services: ServiceResources,
    pub sensors: SensorResources,
    pub can: embassy_stm32::can::Can<'static>,
}

#[allow(dead_code)]
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

    let can = resources.can_bus.setup();

    PreparedBoard {
        can,
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

pub fn spawn_tasks(board: PreparedBoard, thread_spawner: Spawner, level_0_spawner: SendSpawner) {
    level_0_spawner.spawn(
        tasks::blinky::task(board.services.yellow_led).expect("Failed to spawn blinky task"),
    );

    // --- Readouts -----------------------------------------------------------
    let (imu1_spi, imu1_int1) = board.sensors.imu1;
    let (imu2_spi, imu2_int1) = board.sensors.imu2;
    level_0_spawner.spawn(
        tasks::readout::imu::task(imu1_spi, imu1_int1, IMU_0).expect("Failed to spawn IMU 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::imu::task(imu2_spi, imu2_int1, IMU_1).expect("Failed to spawn IMU 1 task"),
    );

    level_0_spawner.spawn(
        tasks::readout::barometer::task(board.sensors.bus1, BAROMETER_0, BAROMETER_DELAY)
            .expect("Failed to spawn barometer 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::barometer::task(board.sensors.bus2, BAROMETER_1, BAROMETER_DELAY)
            .expect("Failed to spawn barometer 1 task"),
    );

    level_0_spawner.spawn(
        tasks::readout::magnetometer::task(board.sensors.bus1, MAGNETOMETER_0, MAGNETOMETER_DELAY)
            .expect("Failed to spawn magnetometer 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::magnetometer::task(board.sensors.bus2, MAGNETOMETER_1, MAGNETOMETER_DELAY)
            .expect("Failed to spawn magnetometer 1 task"),
    );

    level_0_spawner.spawn(
        tasks::readout::gnss::task(board.sensors.gps1_rx, GNSS_0, GNSS_DELAY)
            .expect("Failed to spawn GNSS 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::gnss::task(board.sensors.gps2_rx, GNSS_1, GNSS_DELAY)
            .expect("Failed to spawn GNSS 1 task"),
    );

    level_0_spawner.spawn(
        tasks::readout::dht::task(board.sensors.bus1, DHT_0, DHT_DELAY)
            .expect("Failed to spawn DHT 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::dht::task(board.sensors.bus2, DHT_1, DHT_DELAY)
            .expect("Failed to spawn DHT 1 task"),
    );

    // --- Processing ---------------------------------------------------------
    thread_spawner
        .spawn(tasks::processing::pressure::task().expect("Failed to spawn pressure proc task"));
    thread_spawner.spawn(
        tasks::processing::environmental::task().expect("Failed to spawn environmental proc task"),
    );
    thread_spawner.spawn(
        tasks::processing::magnetic_field::task()
            .expect("Failed to spawn magnetic field proc task"),
    );
    thread_spawner
        .spawn(tasks::processing::inertial::task().expect("Failed to spawn inertial proc task"));
    thread_spawner.spawn(
        tasks::processing::position_velocity::task()
            .expect("Failed to spawn position/velocity proc task"),
    );

    // --- CAN ----------------------------------------------------------------
    let (can_tx, can_rx, _options) = board.can.split();
    thread_spawner.spawn(tasks::can::rx_task(can_rx).expect("Failed to spawn CAN RX task"));
    tasks::can::spawn_tx_tasks(can_tx, thread_spawner);
}
