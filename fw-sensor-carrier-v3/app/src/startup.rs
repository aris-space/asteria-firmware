use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{UartRx, UartTx};

use crate::resources::buses::SharedI2cBus;
use crate::resources::sensors::SpiDevice;
use crate::sensors::{BARO_BUS_1, BARO_BUS_2, GNSS_0, GNSS_1, IMU_0, IMU_1, MAG_BUS_1, MAG_BUS_2};

use crate::{calibration, resources, storage, tasks};

#[allow(dead_code)]
pub struct PreparedBoard {
    pub services: ServiceResources,
    pub sensors: SensorResources,
    pub can: embassy_stm32::can::Can<'static>,
    pub storage: &'static storage::Storage,
    pub usb: resources::usb::UsbDriver,
    pub sd_card: resources::SdCard,
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
    pub gps2_tx: UartTx<'static, Async>,
    pub imu1: (SpiDevice, ExtiInput<'static, Async>),
    pub imu2: (SpiDevice, ExtiInput<'static, Async>),
    pub bus1: SharedI2cBus,
    pub bus2: SharedI2cBus,
}

pub async fn prepare(resources: resources::AssignedResources) -> PreparedBoard {
    let flash = resources.flash.setup();
    let storage = storage::Storage::init(flash);
    calibration::mag::load(storage).await;
    calibration::imu::load(storage).await;

    let gps1_data = resources.gps1_uart.setup();
    let (_gps1_tx, gps1_rx) = gps1_data.split();

    let gps2_data = resources.gps2_uart.setup();
    let (gps2_tx, gps2_rx) = gps2_data.split();

    let imu1 = resources.imu1.setup();
    let imu2 = resources.imu2.setup();

    let green_led = resources.green_led.setup();
    let yellow_led = resources.yellow_led.setup();
    let red_led = resources.red_led.setup();

    let bus1 = resources.bus1.setup();
    let bus2 = resources.bus2.setup();

    let can = resources.can_bus.setup();

    let usb = resources.usb.setup();

    PreparedBoard {
        can,
        storage,
        usb,
        sd_card: resources.sd_card,
        services: ServiceResources {
            green_led,
            yellow_led,
            red_led,
        },
        sensors: SensorResources {
            gps1_rx,
            gps2_rx,
            gps2_tx,
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
) {
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

    // Do not replace this unless you know why it was there in the first place.
    let mag1 = tasks::readout::magnetometer::init(board.sensors.bus1, MAG_BUS_1).await;
    let baro1 = tasks::readout::barometer::init(board.sensors.bus1, BARO_BUS_1).await;
    let mag2 = tasks::readout::magnetometer::init(board.sensors.bus2, MAG_BUS_2).await;
    let baro2 = tasks::readout::barometer::init(board.sensors.bus2, BARO_BUS_2).await;

    // DHT readout is disabled for the height bench. Initializing DHT caused
    // both I2C barometers to time out; the barometers stay active without it.
    level_0_spawner.spawn(
        tasks::readout::magnetometer::read_task(mag1, board.sensors.bus1, MAG_BUS_1)
            .expect("Failed to spawn magnetometer 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::barometer::read_task(baro1, board.sensors.bus1, BARO_BUS_1)
            .expect("Failed to spawn barometer 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::magnetometer::read_task(mag2, board.sensors.bus2, MAG_BUS_2)
            .expect("Failed to spawn magnetometer 1 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::barometer::read_task(baro2, board.sensors.bus2, BARO_BUS_2)
            .expect("Failed to spawn barometer 1 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::gnss::task(board.sensors.gps1_rx, None, GNSS_0)
            .expect("Failed to spawn GNSS 0 task"),
    );
    level_0_spawner.spawn(
        tasks::readout::gnss::task(board.sensors.gps2_rx, Some(board.sensors.gps2_tx), GNSS_1)
            .expect("Failed to spawn GNSS 1 task"),
    );

    // --- Processing ---------------------------------------------------------
    thread_spawner.spawn(
        tasks::processing::state_estimation::task().expect("Failed to spawn state estimation task"),
    );

    let (sd, detect, power) = board.sd_card.setup();
    thread_spawner.spawn(
        tasks::sd_logging::task(sd, detect, power).expect("Failed to spawn SD logging task"),
    );

    // --- CAN ----------------------------------------------------------------
    let (can_tx, can_rx, _options) = board.can.split();
    thread_spawner.spawn(tasks::can::rx_task(can_rx).expect("Failed to spawn CAN RX task"));
    tasks::can::spawn_tx_tasks(can_tx, thread_spawner);

    // --- USB console --------------------------------------------------------
    tasks::console::start(board.usb, board.storage, thread_spawner);
}
