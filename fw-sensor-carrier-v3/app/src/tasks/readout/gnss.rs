use defmt::{Debug2Format, debug, error, info, warn};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_time::{Duration, Instant, with_timeout};
use ublox::cfg_val::CfgVal;
use ublox::{
    AlignmentToReferenceTime, CfgLayer, CfgMsgSinglePortBuilder, CfgPrtUartBuilder, CfgRate,
    CfgRateBuilder, CfgValSetBuilder, DataBits, GpsFix, InProtoMask, MonVer, NavPvt, NavPvtFlags,
    NavStatus, OutProtoMask, PacketRef, Parity, Parser, StopBits, UartMode, UartPortId,
    UbxPacketMeta, UbxPacketRequest,
};

use core::sync::atomic::Ordering;

use super::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::sensors::{GNSS_STATUS, GnssId, SensorStatus};
use crate::signals;
use crate::types::{GnssSample, Pvt};

// NAV-PVT is requested every 50 ms; a two-second gap means the UART link is silent.
const LINK_SILENCE_TIMEOUT: Duration = Duration::from_secs(2);

fn log_configuration_packet(id: GnssId, packet: &PacketRef<'_>) {
    match packet {
        PacketRef::AckAck(ack) => info!("{} UBX ACK class={} id={}", id, ack.class(), ack.msg_id()),
        PacketRef::AckNak(nak) => warn!("{} UBX NAK class={} id={}", id, nak.class(), nak.msg_id()),
        PacketRef::MonVer(version) => {
            info!(
                "{} receiver software={} hardware={}",
                id,
                version.software_version(),
                version.hardware_version()
            );
            for extension in version.extension() {
                info!("{} receiver extension={}", id, extension);
            }
        }
        PacketRef::Unknown(raw)
            if raw.class == CfgRate::CLASS
                && raw.msg_id == CfgRate::ID
                && raw.payload.len() == 6 =>
        {
            let measure_rate_ms = u16::from_le_bytes([raw.payload[0], raw.payload[1]]);
            let nav_rate = u16::from_le_bytes([raw.payload[2], raw.payload[3]]);
            info!(
                "{} CFG-RATE readback: measure={} ms, nav_rate={}",
                id, measure_rate_ms, nav_rate
            );
        }
        _ => {}
    }
}

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
        let mut link_active = false;

        loop {
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                self.attempt = self.attempt.saturating_add(1);
                consecutive_errors = 0;
                link_active = false;
                GNSS_STATUS[self.id.index()].store(SensorStatus::Inactive, Ordering::Relaxed);
                debug!("{} re-initializing", self.id);
                embassy_time::Timer::after(backoff(self.attempt)).await;
            }

            let n = match with_timeout(LINK_SILENCE_TIMEOUT, self.rx.read(&mut recv_buf)).await {
                Ok(Ok(0)) => continue,
                Ok(Ok(n)) => n,
                Ok(Err(e)) => {
                    warn!("{} read error: {:?}", self.id, Debug2Format(&e));
                    consecutive_errors = consecutive_errors.saturating_add(1);
                    continue;
                }
                Err(_) => {
                    if link_active {
                        warn!("{} UBX link silent", self.id);
                        GNSS_STATUS[self.id.index()]
                            .store(SensorStatus::Inactive, Ordering::Relaxed);
                        link_active = false;
                    }
                    consecutive_errors = MAX_CONSECUTIVE_ERRORS;
                    continue;
                }
            };
            received_bytes = received_bytes.saturating_add(n as u32);

            let mut got_fix = false;
            {
                let mut parsed = self.parser.consume(&recv_buf[..n]);
                while let Some(msg) = parsed.next() {
                    if let Ok(ref packet) = msg {
                        log_configuration_packet(self.id, packet);
                        if !link_active {
                            GNSS_STATUS[self.id.index()]
                                .store(SensorStatus::Active, Ordering::Relaxed);
                            info!("{} UBX link active", self.id);
                            link_active = true;
                        }
                    }
                    match msg {
                        Ok(PacketRef::NavStatus(_)) => {
                            status_packets = status_packets.saturating_add(1);
                            valid_packets = valid_packets.saturating_add(1);
                            self.attempt = 0;
                            consecutive_errors = 0;
                        }
                        Ok(PacketRef::NavPvt(pvt)) => {
                            pvt_packets = pvt_packets.saturating_add(1);
                            valid_packets = valid_packets.saturating_add(1);
                            self.attempt = 0;
                            consecutive_errors = 0;
                            // NAV-PVT alone is enough to establish a usable link and fix.
                            if pvt.flags().contains(NavPvtFlags::GPS_FIX_OK)
                                && matches!(pvt.fix_type(), GpsFix::Fix2D | GpsFix::Fix3D)
                            {
                                fix_type = pvt.fix_type();
                                got_fix = true;
                            }
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
        let mut report_started_at = Instant::now();
        let mut next_report = report_started_at + Duration::from_secs(1);
        let mut pvt_count = 0_u32;
        let mut status_count = 0_u32;
        let mut last_itow = None;
        let mut max_epoch_gap_ms = 0_u32;

        loop {
            match with_timeout(LINK_SILENCE_TIMEOUT, self.rx.read(&mut recv_buf)).await {
                Ok(Ok(n)) if n > 0 => {
                    let mut msgs = self.parser.consume(&recv_buf[..n]);
                    while let Some(pkt) = msgs.next() {
                        if let Ok(ref packet) = pkt {
                            log_configuration_packet(self.id, packet);
                        }
                        match pkt {
                            Ok(PacketRef::NavPvt(pvt)) => {
                                pvt_count = pvt_count.saturating_add(1);
                                if let Some(previous) = last_itow {
                                    max_epoch_gap_ms =
                                        max_epoch_gap_ms.max(pvt.itow().wrapping_sub(previous));
                                }
                                last_itow = Some(pvt.itow());
                                let fix_ok = pvt.flags().contains(NavPvtFlags::GPS_FIX_OK);
                                let now = Instant::now();
                                if now >= next_report {
                                    info!(
                                        "{} GNSS: PVT={}, STATUS={}, report_ms={}, max_epoch_gap={} ms, MSL={} m, vAcc={} mm, vDown={} m/s, sAcc={} m/s, PDOP={}, sats={}, fix={:?}, fixOk={}, utc={}-{}-{}T{}:{}:{} ns={}, iTOW={} ms",
                                        self.id,
                                        pvt_count,
                                        status_count,
                                        now.saturating_duration_since(report_started_at)
                                            .as_millis(),
                                        max_epoch_gap_ms,
                                        pvt.height_msl(),
                                        pvt.vert_accuracy(),
                                        pvt.vel_down(),
                                        pvt.speed_accuracy_estimate(),
                                        pvt.pdop(),
                                        pvt.num_satellites(),
                                        Debug2Format(&pvt.fix_type()),
                                        fix_ok,
                                        pvt.year(),
                                        pvt.month(),
                                        pvt.day(),
                                        pvt.hour(),
                                        pvt.min(),
                                        pvt.sec(),
                                        pvt.nanosecond(),
                                        pvt.itow(),
                                    );
                                    pvt_count = 0;
                                    status_count = 0;
                                    max_epoch_gap_ms = 0;
                                    report_started_at = now;
                                    next_report = now + Duration::from_secs(1);
                                }
                                if !fix_ok
                                    || !matches!(pvt.fix_type(), GpsFix::Fix2D | GpsFix::Fix3D)
                                {
                                    continue;
                                }
                                self.errors = 0;
                                let sample = GnssSample {
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
                                signals::submit_gnss_sample(sample);
                            }
                            Ok(PacketRef::NavStatus(_)) => {
                                status_count = status_count.saturating_add(1);
                            }
                            Ok(_) => {}
                            Err(e) => {
                                warn!("{} parse error: {:?}", self.id, Debug2Format(&e));
                                self.errors = self.errors.saturating_add(1);
                            }
                        }
                    }
                }
                Ok(Err(e)) => {
                    warn!("{} read error: {:?}", self.id, Debug2Format(&e));
                    self.errors = self.errors.saturating_add(1);
                }
                Err(_) => {
                    warn!("{} UBX link silent", self.id);
                    return Inactive {
                        rx: self.rx,
                        parser: self.parser,
                        id: self.id,
                        attempt: 0,
                    };
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
    mut tx: Option<UartTx<'static, Async>>,
) -> ! {
    let mut inactive = Inactive {
        rx,
        parser,
        id,
        attempt: 0,
    };

    loop {
        let active = inactive.run().await;
        if let Some(tx) = tx.as_mut() {
            // Poll after the receive loop has survived startup UART errors.
            embassy_time::Timer::after_millis(100).await;
            poll_gnss_1_configuration(tx).await;
        }
        GNSS_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
        inactive = active.run().await;
        GNSS_STATUS[id.index()].store(SensorStatus::Inactive, Ordering::Relaxed);
    }
}

async fn configure_gnss_1_port(
    tx: &mut UartTx<'static, Async>,
    rx: &UartRx<'static, Async>,
) -> bool {
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
        return false;
    }
    true
}

async fn configure_gnss_1_messages(tx: &mut UartTx<'static, Async>) {
    // GPS + Galileo sustains 20 PVT solutions/s on this F9P. Keep the
    // receiver's stored signal configuration available after a reset.
    let signals = [
        CfgVal::SignalGpsEna(true),
        CfgVal::SignalGalEna(true),
        CfgVal::SignalGloEna(false),
        CfgVal::SignalBdsEna(false),
    ];
    let mut constellation = heapless::Vec::<u8, 64>::new();
    CfgValSetBuilder {
        version: 0,
        layers: CfgLayer::RAM,
        reserved1: 0,
        cfg_data: &signals,
    }
    .extend_to(&mut constellation);
    let rate = CfgRateBuilder {
        measure_rate_ms: 50,
        nav_rate: 1,
        time_ref: AlignmentToReferenceTime::Gps,
    }
    .into_packet_bytes();
    let status = CfgMsgSinglePortBuilder::set_rate_for::<NavStatus>(1).into_packet_bytes();
    let pvt = CfgMsgSinglePortBuilder::set_rate_for::<NavPvt>(1).into_packet_bytes();
    for packet in [&constellation[..], &rate[..], &status[..], &pvt[..]] {
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

async fn poll_gnss_1_configuration(tx: &mut UartTx<'static, Async>) {
    let poll_rate = UbxPacketRequest::request_for::<CfgRate>().into_packet_bytes();
    let poll_version = UbxPacketRequest::request_for::<MonVer>().into_packet_bytes();
    for packet in [&poll_rate[..], &poll_version[..]] {
        if let Err(e) = tx.write(packet).await {
            warn!("GNSS_1 configuration poll failed: {:?}", Debug2Format(&e));
        }
        embassy_time::Timer::after_millis(20).await;
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(
    rx: UartRx<'static, Async>,
    mut tx: Option<UartTx<'static, Async>>,
    id: GnssId,
) -> ! {
    let gnss_1_ready = if let Some(tx) = tx.as_mut() {
        configure_gnss_1_port(tx, &rx).await
    } else {
        false
    };
    let mut uart_ring_buf = [0u8; 4096];
    let rx = rx.into_ring_buffered(&mut uart_ring_buf);

    // Start DMA reception before asking for acknowledgements and readbacks.
    if gnss_1_ready {
        configure_gnss_1_messages(tx.as_mut().expect("GNSS_1 TX present")).await;
    }

    let mut parse_buf = [0u8; 4096];
    let linear_buf = ublox::FixedLinearBuffer::new(&mut parse_buf);
    let parser = Parser::new(linear_buf);

    run_inner(rx, parser, id, tx).await
}
