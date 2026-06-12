use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_time::Duration;

use crate::resources::buses::SharedI2cBus;
use crate::resources::buzzer::BuzzerPwm;
use crate::resources::flash::BoardFlash;
use crate::resources::sd::Sd;
use crate::resources::sensors::SpiDevice;
use crate::sensors::{
    BAROMETER_0, BAROMETER_1, GNSS_0, GNSS_1, IMU_0, IMU_1, MAGNETOMETER_0, MAGNETOMETER_1,
};
use crate::storage;
use defmt_brtt::DefmtConsumer;

use crate::{resources, tasks};

const BAROMETER_DELAY: Duration = Duration::from_millis(20); // TODO: calibrate
const MAGNETOMETER_DELAY: Duration = Duration::from_millis(0); // TODO: calibrate
const GNSS_DELAY: Duration = Duration::from_millis(100); // TODO: calibrate

#[allow(dead_code)]
pub struct PreparedBoard {
    pub services: ServiceResources,
    pub sensors: SensorResources,
    pub flash: &'static mut BoardFlash,
    pub sd: (Sd, Input<'static>, Output<'static>),
}

#[allow(dead_code)]
pub struct ServiceResources {
    pub green_led: Output<'static>,
    pub yellow_led: Output<'static>,
    pub red_led: Output<'static>,
    pub buzzer: BuzzerPwm,
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
    let buzzer = resources.buzzer.setup();

    let bus1 = resources.bus1.setup();
    let bus2 = resources.bus2.setup();

    let flash = resources.flash.setup();

    let sd = resources.sd_card.setup();

    PreparedBoard {
        flash,
        sd,
        services: ServiceResources {
            green_led,
            yellow_led,
            red_led,
            buzzer,
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
    thread_spawner: Spawner,
    level_0_spawner: SendSpawner,
    level_1_spawner: SendSpawner,
    defmt_consumer: DefmtConsumer,
) {
    level_0_spawner.spawn(
        tasks::blinky::task(board.services.yellow_led).expect("Failed to spawn blinky task"),
    );

    // Audible status beeps (calibration done / first GNSS fix) on thread-mode.
    thread_spawner
        .spawn(tasks::buzzer::task(board.services.buzzer).expect("Failed to spawn buzzer task"));

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
        tasks::processing::inertial::task().expect("Failed to spawn inertial processing task"),
    );

    level_1_spawner.spawn(
        tasks::processing::state_estimation::task().expect("Failed to spawn state estimation task"),
    );

    // Storage/config init runs on thread-mode so blocking flash I/O never starves readouts.
    thread_spawner
        .spawn(storage::task(board.flash, defmt_consumer).expect("Failed to spawn storage task"));

    // SD logging runs on thread-mode so FAT write latency cannot starve the
    // interrupt-priority sensor readouts. Producers feed the writer via an
    // internal channel.
    let (sdmmc, sd_detect, sd_power) = board.sd;
    thread_spawner
        .spawn(tasks::sd::task(sdmmc, sd_detect, sd_power).expect("Failed to spawn SD writer task"));
    // Only IMU_0 is logged: it drives the EKF, and dropping IMU_1 cuts the SD
    // record rate by a third (more headroom for card-latency spikes).
    thread_spawner
        .spawn(tasks::sd::imu_producer(IMU_0).expect("Failed to spawn SD IMU 0 producer"));
    thread_spawner
        .spawn(tasks::sd::gnss_producer(GNSS_0).expect("Failed to spawn SD GNSS 0 producer"));
    thread_spawner
        .spawn(tasks::sd::gnss_producer(GNSS_1).expect("Failed to spawn SD GNSS 1 producer"));
    // EKF state is logged directly from the state-estimation task (every
    // predict), so there is no Watch-polling producer here.
}
