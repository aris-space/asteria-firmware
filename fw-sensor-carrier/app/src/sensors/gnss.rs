use crate::Debug2Format;
use crate::drivers::position_velocity::PositionVelocityTimeDriver;
use crate::sensors::{CommonSensorConfig, SensorId, SensorStatus, update_status};
use crate::util::ExponentialBackoff;
use core::fmt::Debug;
use core::future::pending;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_time::{Duration, Instant};
use embedded_utils::fmt::*;
use ublox::{GpsFix, PacketRef, Parser};

#[derive(Debug, Clone, Copy)]
pub struct PvtData {
    pub lon_deg: f64,
    pub lat_deg: f64,
    pub fix_type: GpsFix,
    pub height_msl: f32,
    pub num_satellites: u8,
    pub heading_deg: f32,
    pub heading_accuracy_estimate: f32,
    pub heading_of_vehicle_deg: f32,
    pub vel_north: f32,
    pub vel_east: f32,
    pub vel_down: f32,
    pub pdop: u16,
    pub vert_accuracy: u32,
    pub horiz_accuracy: u32,
    pub magnetic_declination_deg: f32,
    pub magnetic_declination_accuracy_deg: f32,
}

pub const SAMPLE_FREQUENCY_HZ: u32 = 10;
const SAMPLE_INTERVAL: Duration =
    Duration::from_millis((1000f32 / SAMPLE_FREQUENCY_HZ as f32) as u64);

struct InactivePositionSensor<'a, RX> {
    driver: &'a PositionVelocityTimeDriver<'a>,
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
    attempt: u8,
    cfg: CommonSensorConfig,
    id: SensorId,
}

impl<'a, RX, E> InactivePositionSensor<'a, RX>
where
    RX: embedded_io_async::Read<Error = E>,
    E: Debug,
{
    fn new(
        driver: &'a PositionVelocityTimeDriver<'a>,
        rx: RX,
        parser: Parser<ublox::FixedLinearBuffer<'a>>,
        cfg: CommonSensorConfig,
        id: SensorId,
    ) -> Self {
        Self {
            driver,
            rx,
            parser,
            attempt: 0,
            cfg,
            id,
        }
    }

    async fn run(mut self) -> ActivePositionSensor<'a, RX> {
        debug!("{:?} initializing", self.id);

        let mut consecutive_errors = 0;
        let fix_type;
        let mut recv_buf = [0u8; 64];

        'outer: loop {
            if consecutive_errors > self.cfg.max_consecutive_errors {
                self.attempt += 1;
                ExponentialBackoff::new(self.cfg.base_backoff_ms, self.cfg.max_backoff_ms)
                    .wait(self.attempt)
                    .await;
                debug!("{:?} re-initializing", self.id);
            }

            let read_bytes = match self.rx.read(&mut recv_buf).await {
                Ok(0) => continue, // nothing read
                Ok(n) => n,
                Err(e) => {
                    warn!("{:?} read error: {:?}", self.id, Debug2Format(&e));
                    consecutive_errors += 1;
                    continue;
                }
            };

            let mut parsed = self.parser.consume(&recv_buf[..read_bytes]);

            while let Some(msg) = parsed.next() {
                match msg {
                    Ok(PacketRef::NavStatus(stat)) => {
                        debug!("{:?} NAV status {:?}", self.id, Debug2Format(&stat));
                        match stat.fix_type() {
                            // These are ok. We continue if we get these.
                            GpsFix::Fix2D
                            | GpsFix::Fix3D
                            | GpsFix::GPSPlusDeadReckoning
                            | GpsFix::DeadReckoningOnly
                            | GpsFix::TimeOnlyFix => {
                                fix_type = stat.fix_type();
                                break 'outer;
                            }
                            _ => {
                                // GpsFix::NoFix is ok, but we have to wait until
                                // we do have a usable fix.
                                self.attempt = 0;
                                consecutive_errors = 0;
                            }
                        }
                    }
                    Ok(_) => {
                        // this message is ok, just shows us that everything is working.
                        self.attempt = 0;
                        consecutive_errors = 0;
                    }
                    Err(e) => {
                        warn!("{:?} parse error: {:?}", self.id, Debug2Format(&e));
                        consecutive_errors += 1;
                    }
                }
            }
        }

        info!(
            "{:?} initialized (fix type: {:?})",
            self.id,
            Debug2Format(&fix_type)
        );
        ActivePositionSensor::new(self.driver, self.rx, self.parser, self.cfg, self.id)
    }
}

struct ActivePositionSensor<'a, RX> {
    driver: &'a PositionVelocityTimeDriver<'a>,
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
    cfg: CommonSensorConfig,
    errors: u8,
    id: SensorId,
}

impl<'a, RX, E> ActivePositionSensor<'a, RX>
where
    RX: embedded_io_async::Read<Error = E>,
    E: Debug,
{
    fn new(
        driver: &'a PositionVelocityTimeDriver<'a>,
        rx: RX,
        parser: Parser<ublox::FixedLinearBuffer<'a>>,
        cfg: CommonSensorConfig,
        id: SensorId,
    ) -> Self {
        Self {
            driver,
            rx,
            parser,
            cfg,
            errors: 0,
            id,
        }
    }

    async fn run(mut self) -> InactivePositionSensor<'a, RX> {
        let mut recv_buf = [0u8; 4096];

        loop {
            match self.rx.read(&mut recv_buf).await {
                Ok(n) if n > 0 => {
                    self.errors = 0;
                    let mut msgs = self.parser.consume(&recv_buf[..n]);
                    while let Some(pkt) = msgs.next() {
                        match pkt {
                            Ok(PacketRef::NavTimeUTC(_time)) => {
                                //todo: send time to the driver
                            }
                            Ok(PacketRef::NavPvt(pvt))
                                if matches!(
                                    pvt.fix_type(),
                                    GpsFix::Fix2D
                                        | GpsFix::Fix3D
                                        | GpsFix::GPSPlusDeadReckoning
                                        | GpsFix::DeadReckoningOnly
                                ) =>
                            {
                                // TODO: we might want to check that for the GPS_FIX_OK flag
                                let data = PvtData {
                                    lon_deg: pvt.lon_degrees(),
                                    lat_deg: pvt.lat_degrees(),
                                    fix_type: pvt.fix_type(),
                                    height_msl: pvt.height_msl() as f32,
                                    num_satellites: pvt.num_satellites(),
                                    heading_deg: pvt.heading_degrees() as f32,
                                    heading_accuracy_estimate: pvt.heading_accuracy_estimate()
                                        as f32,
                                    heading_of_vehicle_deg: pvt.heading_of_vehicle_degrees() as f32,
                                    vel_north: pvt.vel_north() as f32,
                                    vel_east: pvt.vel_east() as f32,
                                    vel_down: pvt.vel_down() as f32,
                                    pdop: pvt.pdop(),
                                    vert_accuracy: pvt.vert_accuracy(),
                                    horiz_accuracy: pvt.horiz_accuracy(),
                                    magnetic_declination_deg: pvt.magnetic_declination_degrees()
                                        as f32,
                                    magnetic_declination_accuracy_deg: pvt
                                        .magnetic_declination_accuracy_degrees()
                                        as f32,
                                };

                                // todo is not correct or accurate. We should use timestamp from message
                                let measurement_ts = Instant::now();

                                self.driver.update(data, measurement_ts, self.id).await;
                            }
                            Ok(_x) => {
                                // TODO: handle other messages
                            }
                            Err(e) => {
                                warn!("{:?} parse error: {:?}", self.id, Debug2Format(&e));
                                self.errors += 1;
                            }
                        }
                    }
                }
                Err(e) => {
                    warn!("{:?} read error: {:?}", self.id, Debug2Format(&e));
                    self.errors += 1;
                }
                _ => {}
            }

            if self.errors >= self.cfg.max_consecutive_errors {
                error!("{:?} offline (too many consecutive errors)", self.id);
                return InactivePositionSensor::new(
                    self.driver,
                    self.rx,
                    self.parser,
                    self.cfg,
                    self.id,
                );
            }
        }
    }
}

struct PositionSensorNode<'a> {
    driver: &'a PositionVelocityTimeDriver<'a>,
    cfg: CommonSensorConfig,
    id: SensorId,
}

impl<'a> PositionSensorNode<'a> {
    fn new(
        driver: &'a PositionVelocityTimeDriver<'a>,
        cfg: CommonSensorConfig,
        id: SensorId,
    ) -> Self {
        Self { driver, cfg, id }
    }

    pub async fn run<RX, E>(self, rx: RX, parser: Parser<ublox::FixedLinearBuffer<'a>>) -> !
    where
        RX: embedded_io_async::Read<Error = E>,
        E: Debug,
    {
        let mut inactive = InactivePositionSensor::new(self.driver, rx, parser, self.cfg, self.id);

        loop {
            let active = inactive.run().await;
            update_status(SensorStatus::Active, self.id);
            inactive = active.run().await;
            update_status(SensorStatus::Inactive, self.id);
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn gnss_task(
    rx: UartRx<'static, Async>,
    driver: &'static PositionVelocityTimeDriver<'static>,
    sensor_id: SensorId,
) -> ! {
    let cfg = CommonSensorConfig {
        max_consecutive_errors: 10,
        base_backoff_ms: 100,
        max_backoff_ms: 20_000,
    };

    let mut uart_ring_buf = [0u8; 4096];
    let rx = rx.into_ring_buffered(&mut uart_ring_buf);

    let mut parse_buf = [0u8; 4096];
    let linear_buf = ublox::FixedLinearBuffer::new(&mut parse_buf);

    let parser = Parser::new(linear_buf);
    PositionSensorNode::new(driver, cfg, sensor_id)
        .run(rx, parser)
        .await;
    #[allow(unreachable_code)]
    loop {
        pending::<()>().await;
    }
}
