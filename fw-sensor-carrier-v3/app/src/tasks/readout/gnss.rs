use defmt::{Debug2Format, debug, error, info, warn};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_time::{Duration, Instant};
use ublox::{
    AlignmentToReferenceTime, CfgMsgSinglePortBuilder, CfgPrtUartBuilder, CfgRateBuilder, DataBits,
    GpsFix, InProtoMask, NavPvt, NavPvtFlags, NavStatus, OutProtoMask, PacketRef, Parity, Parser,
    StopBits, UartMode, UartPortId,
};

use core::sync::atomic::Ordering;

use super::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::calibration;
use crate::sensors::{GNSS_STATUS, GnssId, SensorStatus};
use crate::signals;
use crate::types::{Pvt, RawGnssSample};

struct Inactive<'a, RX> {
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
    id: GnssId,
    attempt: u8,
}

impl<'a, RX: embedded_io_async::Read> Inactive<'a, RX> {
    async fn run(mut self) -> Active<'a, RX> {
        debug!("{} initializing", self.id);

        let mut consecutive_errors: u8 = 0;
        let mut recv_buf = [0u8; 64];
        let mut fix_type = GpsFix::NoFix;
        let mut report_at = Instant::now() + Duration::from_secs(30);
        let mut valid_packets = 0u32;
        let mut status_packets = 0u32;
        let mut pvt_packets = 0u32;
        let mut received_bytes = 0u32;

        loop {
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                self.attempt = self.attempt.saturating_add(1);
                consecutive_errors = 0;
                debug!("{} re-initializing", self.id);
                embassy_time::Timer::after(backoff(self.attempt)).await;
            }

            let n = match self.rx.read(&mut recv_buf).await {
                Ok(0) => continue,
                Ok(n) => n,
                Err(e) => {
                    warn!("{} read error: {:?}", self.id, Debug2Format(&e));
                    consecutive_errors = consecutive_errors.saturating_add(1);
                    continue;
                }
            };
            received_bytes = received_bytes.saturating_add(n as u32);

            let mut got_fix = false;
            {
                let mut parsed = self.parser.consume(&recv_buf[..n]);
                while let Some(msg) = parsed.next() {
                    match msg {
                        Ok(PacketRef::NavStatus(stat)) => {
                            status_packets = status_packets.saturating_add(1);
                            match stat.fix_type() {
                                GpsFix::Fix2D
                                | GpsFix::Fix3D
                                | GpsFix::GPSPlusDeadReckoning
                                | GpsFix::TimeOnlyFix => {
                                    fix_type = stat.fix_type();
                                    got_fix = true;
                                    break;
                                }
                                _ => {
                                    valid_packets = valid_packets.saturating_add(1);
                                    self.attempt = 0;
                                    consecutive_errors = 0;
                                }
                            }
                        }
                        Ok(PacketRef::NavPvt(_)) => {
                            pvt_packets = pvt_packets.saturating_add(1);
                            valid_packets = valid_packets.saturating_add(1);
                            self.attempt = 0;
                            consecutive_errors = 0;
                        }
                        Ok(_) => {
                            valid_packets = valid_packets.saturating_add(1);
                            self.attempt = 0;
                            consecutive_errors = 0;
                        }
                        Err(e) => {
                            warn!("{} parse error: {:?}", self.id, Debug2Format(&e));
                            consecutive_errors = consecutive_errors.saturating_add(1);
                        }
                    }
                }
            }

            if Instant::now() >= report_at {
                info!(
                    "{} GNSS link: {} bytes, {} UBX, {} NAV-STATUS, {} NAV-PVT in 30 s",
                    self.id, received_bytes, valid_packets, status_packets, pvt_packets
                );
                received_bytes = 0;
                valid_packets = 0;
                status_packets = 0;
                pvt_packets = 0;
                report_at = Instant::now() + Duration::from_secs(30);
            }

            if got_fix {
                info!(
                    "{} initialized (fix type: {:?})",
                    self.id,
                    Debug2Format(&fix_type)
                );
                return Active {
                    rx: self.rx,
                    parser: self.parser,
                    id: self.id,
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
    errors: u8,
}

impl<'a, RX: embedded_io_async::Read> Active<'a, RX> {
    async fn run(mut self) -> Inactive<'a, RX> {
        let mut recv_buf = [0u8; 4096];
        let mut next_report = Instant::now();

        loop {
            match self.rx.read(&mut recv_buf).await {
                Ok(n) if n > 0 => {
                    let mut msgs = self.parser.consume(&recv_buf[..n]);
                    while let Some(pkt) = msgs.next() {
                        match pkt {
                            Ok(PacketRef::NavPvt(pvt)) => {
                                let fix_ok = pvt.flags().contains(NavPvtFlags::GPS_FIX_OK);
                                if Instant::now() >= next_report {
                                    info!(
                                        "{} GNSS: MSL={} m, vAcc={} mm, vDown={} m/s, sAcc={} m/s, PDOP={}, sats={}, fix={:?}, fixOk={}",
                                        self.id,
                                        pvt.height_msl(),
                                        pvt.vert_accuracy(),
                                        pvt.vel_down(),
                                        pvt.speed_accuracy_estimate(),
                                        pvt.pdop(),
                                        pvt.num_satellites(),
                                        Debug2Format(&pvt.fix_type()),
                                        fix_ok,
                                    );
                                    next_report = Instant::now() + Duration::from_secs(1);
                                }
                                if !fix_ok
                                    || !matches!(pvt.fix_type(), GpsFix::Fix2D | GpsFix::Fix3D)
                                {
                                    continue;
                                }
                                self.errors = 0;
                                let raw = RawGnssSample {
                                    src: self.id,
                                    ts: Instant::now(),
                                    pvt: Pvt {
                                        fix_type: pvt.fix_type(),
                                        height_msl: pvt.height_msl() as f32,
                                        vel_down: pvt.vel_down() as f32,
                                        pdop: pvt.pdop(),
                                        vert_accuracy: pvt.vert_accuracy(),
                                        speed_accuracy_mps: pvt.speed_accuracy_estimate() as f32,
                                    },
                                };
                                signals::submit_gnss_sample(calibration::gnss::apply_calibration(
                                    raw,
                                ));
                            }
                            Ok(_) => {}
                            Err(e) => {
                                warn!("{} parse error: {:?}", self.id, Debug2Format(&e));
                                self.errors = self.errors.saturating_add(1);
                            }
                        }
                    }
                }
                Err(e) => {
                    warn!("{} read error: {:?}", self.id, Debug2Format(&e));
                    self.errors = self.errors.saturating_add(1);
                }
                _ => {}
            }

            if self.errors >= MAX_CONSECUTIVE_ERRORS {
                error!("{} offline (too many consecutive errors)", self.id);
                return Inactive {
                    rx: self.rx,
                    parser: self.parser,
                    id: self.id,
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
) -> ! {
    let mut inactive = Inactive {
        rx,
        parser,
        id,
        attempt: 0,
    };

    loop {
        let active = inactive.run().await;
        GNSS_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
        inactive = active.run().await;
        GNSS_STATUS[id.index()].store(SensorStatus::Inactive, Ordering::Relaxed);
    }
}

async fn configure_gnss_1(tx: &mut UartTx<'static, Async>, rx: &UartRx<'static, Async>) {
    // A receiver with factory settings sends NMEA at 38400 baud. An already
    // configured receiver ignores this first packet and accepts the later ones.
    let port = CfgPrtUartBuilder {
        portid: UartPortId::Uart1,
        reserved0: 0,
        tx_ready: 0,
        mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
        baud_rate: 921_600,
        in_proto_mask: InProtoMask::all(),
        out_proto_mask: OutProtoMask::UBLOX,
        flags: 0,
        reserved5: 0,
    }
    .into_packet_bytes();
    if let Err(e) = tx.write(&port).await {
        warn!("GNSS_1 port configuration failed: {:?}", Debug2Format(&e));
    }
    embassy_time::Timer::after_millis(100).await;
    if let Err(e) = rx.set_baudrate(921_600) {
        warn!("GNSS_1 baud change failed: {:?}", Debug2Format(&e));
        return;
    }

    // Match GNSS_0 so both receivers can produce the same navigation epochs.
    let rate = CfgRateBuilder {
        measure_rate_ms: 50,
        nav_rate: 1,
        time_ref: AlignmentToReferenceTime::Gps,
    }
    .into_packet_bytes();
    let status = CfgMsgSinglePortBuilder::set_rate_for::<NavStatus>(1).into_packet_bytes();
    let pvt = CfgMsgSinglePortBuilder::set_rate_for::<NavPvt>(1).into_packet_bytes();
    for packet in [&rate[..], &status[..], &pvt[..]] {
        if let Err(e) = tx.write(packet).await {
            warn!(
                "GNSS_1 message configuration failed: {:?}",
                Debug2Format(&e)
            );
            return;
        }
        embassy_time::Timer::after_millis(20).await;
    }
    info!("GNSS_1 UBX configuration sent at 921600 baud");
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(
    rx: UartRx<'static, Async>,
    mut tx: Option<UartTx<'static, Async>>,
    id: GnssId,
) -> ! {
    if let Some(tx) = tx.as_mut() {
        configure_gnss_1(tx, &rx).await;
    }
    let mut uart_ring_buf = [0u8; 4096];
    let rx = rx.into_ring_buffered(&mut uart_ring_buf);

    let mut parse_buf = [0u8; 4096];
    let linear_buf = ublox::FixedLinearBuffer::new(&mut parse_buf);
    let parser = Parser::new(linear_buf);

    run_inner(rx, parser, id).await
}
