use defmt::{info, warn};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_time::Duration;
use ublox::{GpsFix, PacketRef, Parser};

use crate::tasks::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::measurements::{GnssSample, PvtData, Timestamped};
use crate::sensors::GnssId;
use crate::signals;

struct Inactive<'a, RX> {
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
    id: GnssId,
    delay: Duration,
    attempt: u8,
}

impl<'a, RX: embedded_io_async::Read> Inactive<'a, RX> {
    async fn run(mut self) -> Active<'a, RX> {
        let mut consecutive_errors: u8 = 0;
        let mut recv_buf = [0u8; 64];

        loop {
            if consecutive_errors > MAX_CONSECUTIVE_ERRORS {
                self.attempt = self.attempt.saturating_add(1);
                warn!("gnss: too many parse errors (attempt {})", self.attempt);
                consecutive_errors = 0;
                embassy_time::Timer::after(backoff(self.attempt)).await;
            }

            let n = match self.rx.read(&mut recv_buf).await {
                Ok(0) => continue,
                Ok(n) => n,
                Err(_) => {
                    consecutive_errors += 1;
                    continue;
                }
            };

            let mut got_fix = false;
            {
                let mut parsed = self.parser.consume(&recv_buf[..n]);
                while let Some(msg) = parsed.next() {
                    match msg {
                        Ok(PacketRef::NavStatus(stat)) => match stat.fix_type() {
                            GpsFix::Fix2D
                            | GpsFix::Fix3D
                            | GpsFix::GPSPlusDeadReckoning
                            | GpsFix::TimeOnlyFix => {
                                got_fix = true;
                                break;
                            }
                            _ => {
                                self.attempt = 0;
                                consecutive_errors = 0;
                            }
                        },
                        Ok(_) => {
                            self.attempt = 0;
                            consecutive_errors = 0;
                        }
                        Err(_) => {
                            consecutive_errors += 1;
                        }
                    }
                }
            }

            if got_fix {
                info!("gnss: active (fix acquired)");
                return Active {
                    rx: self.rx,
                    parser: self.parser,
                    id: self.id,
                    delay: self.delay,
                    errors: 0,
                };
            }
        }
    }
}

struct Active<'a, RX> {
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
    id: GnssId,
    delay: Duration,
    errors: u8,
}

impl<'a, RX: embedded_io_async::Read> Active<'a, RX> {
    async fn run(mut self) -> Inactive<'a, RX> {
        let mut recv_buf = [0u8; 4096];

        loop {
            match self.rx.read(&mut recv_buf).await {
                Ok(n) if n > 0 => {
                    let mut msgs = self.parser.consume(&recv_buf[..n]);
                    while let Some(pkt) = msgs.next() {
                        match pkt {
                            Ok(PacketRef::NavPvt(pvt))
                                if matches!(pvt.fix_type(), GpsFix::Fix2D | GpsFix::Fix3D) =>
                            {
                                self.errors = 0;
                                let sample = GnssSample {
                                    sensor_id: self.id,
                                    data: Timestamped::now_with_delay(
                                        PvtData {
                                            lon_deg: pvt.lon_degrees(),
                                            lat_deg: pvt.lat_degrees(),
                                            fix_type: pvt.fix_type(),
                                            height_msl: pvt.height_msl() as f32,
                                            num_satellites: pvt.num_satellites(),
                                            heading_deg: pvt.heading_degrees() as f32,
                                            heading_accuracy_estimate: pvt
                                                .heading_accuracy_estimate()
                                                as f32,
                                            heading_of_vehicle_deg: pvt
                                                .heading_of_vehicle_degrees()
                                                as f32,
                                            vel_north: pvt.vel_north() as f32,
                                            vel_east: pvt.vel_east() as f32,
                                            vel_down: pvt.vel_down() as f32,
                                            pdop: pvt.pdop(),
                                            vert_accuracy: pvt.vert_accuracy(),
                                            horiz_accuracy: pvt.horiz_accuracy(),
                                            magnetic_declination_deg: pvt
                                                .magnetic_declination_degrees()
                                                as f32,
                                            magnetic_declination_accuracy_deg: pvt
                                                .magnetic_declination_accuracy_degrees()
                                                as f32,
                                        },
                                        self.delay,
                                    ),
                                };
                                signals::submit_gnss_sample(sample);
                            }
                            Ok(_) => {}
                            Err(_) => {
                                self.errors += 1;
                            }
                        }
                    }
                }
                Err(_) => {
                    self.errors += 1;
                }
                _ => {}
            }

            if self.errors >= MAX_CONSECUTIVE_ERRORS {
                warn!("gnss: inactive (too many errors)");
                return Inactive {
                    rx: self.rx,
                    parser: self.parser,
                    id: self.id,
                    delay: self.delay,
                    attempt: 0,
                };
            }
        }
    }
}

async fn run_inner<'a, RX: embedded_io_async::Read>(
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
    id: GnssId,
    delay: Duration,
) -> ! {
    let mut inactive = Inactive {
        rx,
        parser,
        id,
        delay,
        attempt: 0,
    };

    loop {
        let active = inactive.run().await;
        inactive = active.run().await;
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(rx: UartRx<'static, Async>, id: GnssId, delay: Duration) -> ! {
    let mut uart_ring_buf = [0u8; 4096];
    let rx = rx.into_ring_buffered(&mut uart_ring_buf);

    let mut parse_buf = [0u8; 4096];
    let linear_buf = ublox::FixedLinearBuffer::new(&mut parse_buf);
    let parser = Parser::new(linear_buf);

    run_inner(rx, parser, id, delay).await
}
