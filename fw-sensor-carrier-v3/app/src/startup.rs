use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;

use crate::resources::buses::SharedI2cBus;
use crate::resources::sensors::SpiDevice;
use crate::sensors::{
    BARO_BUS_1, BARO_BUS_2, DHT_BUS_1, DHT_BUS_2, GNSS_0, GNSS_1, IMU_0, IMU_1, MAG_BUS_1,
    MAG_BUS_2,
};

use crate::{calibration, resources, storage, tasks};

pub struct PreparedBoard {
    pub yellow_led: Output<'static>,
    pub buzzer: resources::buzzer::BuzzerPwm,
    pub sensors: SensorResources,
    pub can: embassy_stm32::can::Can<'static>,
    pub storage: &'static storage::Storage,
    pub usb: resources::usb::UsbDriver,
    pub sd_card: resources::SdCard,
}

pub struct SensorResources {
    pub gps1_rx: UartRx<'static, Async>,
    pub gps2_rx: UartRx<'static, Async>,
    pub imu1: (SpiDevice, ExtiInput<'static, Async>),
    pub imu2: (SpiDevice, ExtiInput<'static, Async>),
    pub bus1: SharedI2cBus,
    pub bus2: SharedI2cBus,
}

pub async fn prepare(resources: resources::AssignedResources) -> PreparedBoard {
    let flash = resources.flash.setup();
    let storage = storage::Storage::init(flash);
    calibration::load(storage).await;

    let gps1_rx = resources.gps1_uart.setup();
    let gps2_rx = resources.gps2_uart.setup();

    let imu1 = resources.imu1.setup();
    let imu2 = resources.imu2.setup();

    let yellow_led = resources.yellow_led.setup();
    let buzzer = resources.buzzer.setup();

    let bus1 = resources.bus1.setup();
    let bus2 = resources.bus2.setup();

    let can = resources.can_bus.setup();

    let usb = resources.usb.setup();

    PreparedBoard {
        can,
        storage,
        usb,
        sd_card: resources.sd_card,
        yellow_led,
        buzzer,
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

pub async fn spawn_tasks(
    board: PreparedBoard,
    thread_spawner: Spawner,
    level_0_spawner: SendSpawner,
    level_1_spawner: SendSpawner,
) {
    level_0_spawner
        .spawn(tasks::blinky::task(board.yellow_led).expect("Failed to spawn blinky task"));
    thread_spawner.spawn(tasks::buzzer::task(board.buzzer).expect("Failed to spawn buzzer task"));

    let (sd, detect, power) = board.sd_card.setup();
    thread_spawner.spawn(
        tasks::sd_logging::task(sd, detect, power).expect("Failed to spawn SD logging task"),
    );

    let (imu1_spi, imu1_int1) = board.sensors.imu1;
    let (imu2_spi, imu2_int1) = board.sensors.imu2;
    level_0_spawner.spawn(
        tasks::readout::imu::task(imu1_spi, imu1_int1, IMU_0).expect("Failed to spawn IMU 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::imu::task(imu2_spi, imu2_int1, IMU_1).expect("Failed to spawn IMU 1 task"),
    );

    let mag1 = tasks::readout::mag::init(board.sensors.bus1, MAG_BUS_1).await;
    let baro1 = tasks::readout::baro::init(board.sensors.bus1, BARO_BUS_1).await;
    let mag2 = tasks::readout::mag::init(board.sensors.bus2, MAG_BUS_2).await;
    let baro2 = tasks::readout::baro::init(board.sensors.bus2, BARO_BUS_2).await;
    let dht1 = tasks::readout::dht::init(board.sensors.bus1, DHT_BUS_1).await;
    let dht2 = tasks::readout::dht::init(board.sensors.bus2, DHT_BUS_2).await;

    level_0_spawner.spawn(
        tasks::readout::mag::task(mag1, board.sensors.bus1, MAG_BUS_1)
            .expect("Failed to spawn magnetometer 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::baro::task(baro1, board.sensors.bus1, BARO_BUS_1)
            .expect("Failed to spawn barometer 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::mag::task(mag2, board.sensors.bus2, MAG_BUS_2)
            .expect("Failed to spawn magnetometer 1 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::baro::task(baro2, board.sensors.bus2, BARO_BUS_2)
            .expect("Failed to spawn barometer 1 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::dht::task(dht1, board.sensors.bus1, DHT_BUS_1)
            .expect("Failed to spawn DHT 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::dht::task(dht2, board.sensors.bus2, DHT_BUS_2)
            .expect("Failed to spawn DHT 1 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::gnss::task(board.sensors.gps1_rx, GNSS_0)
            .expect("Failed to spawn GNSS 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::gnss::task(board.sensors.gps2_rx, GNSS_1)
            .expect("Failed to spawn GNSS 1 task"),
    );

    level_1_spawner
        .spawn(tasks::state_estimation::task().expect("Failed to spawn state estimation task"));

    let (can_tx, can_rx, _options) = board.can.split();
    level_1_spawner.spawn(tasks::can::rx::task(can_rx).expect("Failed to spawn CAN RX task"));
    tasks::can::tx::spawn(can_tx, level_1_spawner);

    thread_spawner.spawn(tasks::state_report::task().expect("Failed to spawn state report task"));
    tasks::console::spawn(board.usb, board.storage, thread_spawner);
}
