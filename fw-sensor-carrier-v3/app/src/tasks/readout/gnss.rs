//! u-blox GNSS readout over UART. NAV-PVT epochs are logged to SD from the
//! first packet; only fixes reach the estimator. `Inactive` already reports
//! the UART link as active before the first fix.

use core::sync::atomic::Ordering;

use defmt::{Debug2Format, debug, error, info, warn};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use ublox::cfg_val::CfgVal;
use ublox::{
    AlignmentToReferenceTime, CfgLayer, CfgMsgSinglePortBuilder, CfgPrtUartBuilder, CfgRate,
    CfgRateBuilder, CfgValSetBuilder, DataBits, GpsFix, InProtoMask, MonVer, NavPvt, NavPvtFlags,
    NavStatus, OutProtoMask, PacketRef, Parity, Parser, StopBits, UartMode, UartPortId,
    UbxPacketMeta, UbxPacketRequest,
};

use super::{MAX_CONSECUTIVE_ERRORS, State, backoff};
use crate::calibration;
use crate::sensors::{GNSS_STATUS, GnssId, SensorStatus};
use crate::signals;
use crate::types::{GnssSample, Pvt, RawGnssSample, SdLogRecord};

// NAV-PVT is requested every 50 ms; a two-second gap means the UART link is silent.
const LINK_SILENCE_TIMEOUT: Duration = Duration::from_secs(2);

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
        warn!("GNSS_1: port configuration failed: {:?}", Debug2Format(&e));
    }
    Timer::after_millis(100).await;
    if let Err(e) = rx.set_baudrate(921_600) {
        warn!("GNSS_1: baud change failed: {:?}", Debug2Format(&e));
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
                "GNSS_1: message configuration failed: {:?}",
                Debug2Format(&e)
            );
            return;
        }
        Timer::after_millis(20).await;
    }
    info!("GNSS_1: UBX configuration sent at 921600 baud");
}

async fn poll_gnss_1_configuration(tx: &mut UartTx<'static, Async>) {
    let poll_rate = UbxPacketRequest::request_for::<CfgRate>().into_packet_bytes();
    let poll_version = UbxPacketRequest::request_for::<MonVer>().into_packet_bytes();
    for packet in [&poll_rate[..], &poll_version[..]] {
        if let Err(e) = tx.write(packet).await {
            warn!("GNSS_1: configuration poll failed: {:?}", Debug2Format(&e));
        }
        Timer::after_millis(20).await;
    }
}

struct Inactive<'a, RX> {
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
    /// Present for GNSS_1, whose configuration this firmware sends.
    tx: Option<UartTx<'static, Async>>,
    id: GnssId,
    attempt: u8,
}

impl<'a, RX: embedded_io_async::Read> State for Inactive<'a, RX> {
    type Next = Active<'a, RX>;

    async fn run(mut self) -> Active<'a, RX> {
        debug!("{}: initializing", self.id);

        let mut consecutive_errors: u8 = 0;
        let mut recv_buf = [0u8; 64];
        let mut link_active = false;

        loop {
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                self.attempt = self.attempt.saturating_add(1);
                consecutive_errors = 0;
                link_active = false;
                GNSS_STATUS[self.id.index()].store(SensorStatus::Inactive, Ordering::Relaxed);
                debug!("{}: re-initializing", self.id);
                Timer::after(backoff(self.attempt)).await;
            }

            let n = match with_timeout(LINK_SILENCE_TIMEOUT, self.rx.read(&mut recv_buf)).await {
                Ok(Ok(0)) => continue,
                Ok(Ok(n)) => n,
                Ok(Err(e)) => {
                    warn!("{}: read error: {:?}", self.id, Debug2Format(&e));
                    consecutive_errors = consecutive_errors.saturating_add(1);
                    continue;
                }
                Err(_) => {
                    if link_active {
                        warn!("{}: UBX link silent", self.id);
                        GNSS_STATUS[self.id.index()]
                            .store(SensorStatus::Inactive, Ordering::Relaxed);
                        link_active = false;
                    }
                    consecutive_errors = MAX_CONSECUTIVE_ERRORS;
                    continue;
                }
            };

            let mut fix = None;
            let mut parsed = self.parser.consume(&recv_buf[..n]);
            while let Some(msg) = parsed.next() {
                let packet = match msg {
                    Ok(packet) => packet,
                    Err(e) => {
                        warn!("{}: parse error: {:?}", self.id, Debug2Format(&e));
                        consecutive_errors = consecutive_errors.saturating_add(1);
                        continue;
                    }
                };
                log_configuration_packet(self.id, &packet);
                if !link_active {
                    GNSS_STATUS[self.id.index()].store(SensorStatus::Active, Ordering::Relaxed);
                    info!("{}: UBX link active", self.id);
                    link_active = true;
                }
                self.attempt = 0;
                consecutive_errors = 0;
                if let PacketRef::NavPvt(pvt) = packet {
                    // NAV-PVT alone is enough to establish a usable link and fix.
                    let sample = read_pvt(self.id, &pvt);
                    if has_fix(&sample.pvt) {
                        fix = Some(sample.pvt.fix_type);
                    }
                }
            }
            drop(parsed);

            if let Some(fix_type) = fix {
                info!(
                    "{}: active (fix type: {:?})",
                    self.id,
                    Debug2Format(&fix_type)
                );
                if let Some(tx) = self.tx.as_mut() {
                    // Poll after the receive loop has survived startup UART errors.
                    Timer::after_millis(100).await;
                    poll_gnss_1_configuration(tx).await;
                }
                return Active {
                    rx: self.rx,
                    parser: self.parser,
                    tx: self.tx,
                    id: self.id,
                };
            }
        }
    }
}

struct Active<'a, RX> {
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
    tx: Option<UartTx<'static, Async>>,
    id: GnssId,
}

impl<'a, RX: embedded_io_async::Read> State for Active<'a, RX> {
    type Next = Inactive<'a, RX>;

    async fn run(mut self) -> Inactive<'a, RX> {
        let mut recv_buf = [0u8; 4096];
        let mut errors: u8 = 0;

        loop {
            match with_timeout(LINK_SILENCE_TIMEOUT, self.rx.read(&mut recv_buf)).await {
                Ok(Ok(n)) => {
                    let mut parsed = self.parser.consume(&recv_buf[..n]);
                    while let Some(msg) = parsed.next() {
                        match msg {
                            Ok(PacketRef::NavPvt(pvt)) => {
                                let sample = read_pvt(self.id, &pvt);
                                if has_fix(&sample.pvt) {
                                    errors = 0;
                                    signals::submit_gnss_sample(sample);
                                }
                            }
                            Ok(packet) => log_configuration_packet(self.id, &packet),
                            Err(e) => {
                                warn!("{}: parse error: {:?}", self.id, Debug2Format(&e));
                                errors = errors.saturating_add(1);
                            }
                        }
                    }
                }
                Ok(Err(e)) => {
                    warn!("{}: read error: {:?}", self.id, Debug2Format(&e));
                    errors = errors.saturating_add(1);
                }
                Err(_) => {
                    warn!("{}: UBX link silent", self.id);
                    return self.into_inactive();
                }
            }

            if errors >= MAX_CONSECUTIVE_ERRORS {
                error!("{}: offline (too many consecutive errors)", self.id);
                return self.into_inactive();
            }
        }
    }
}

impl<'a, RX> Active<'a, RX> {
    fn into_inactive(self) -> Inactive<'a, RX> {
        Inactive {
            rx: self.rx,
            parser: self.parser,
            tx: self.tx,
            id: self.id,
            attempt: 0,
        }
    }
}

/// Logs a NAV-PVT epoch and returns it calibrated.
fn read_pvt(id: GnssId, pvt: &ublox::NavPvtRef<'_>) -> GnssSample {
    let read_ts = Instant::now();
    let raw = RawGnssSample {
        src: id,
        ts: read_ts,
        read_ts,
        pvt: Pvt {
            itow_ms: pvt.itow(),
            num_satellites: pvt.num_satellites(),
            fix_type: pvt.fix_type(),
            fix_ok: pvt.flags().contains(NavPvtFlags::GPS_FIX_OK),
            latitude_deg: pvt.lat_degrees(),
            longitude_deg: pvt.lon_degrees(),
            height_msl_m: pvt.height_msl() as f32,
            velocity_down_mps: pvt.vel_down() as f32,
            pdop_centi: pvt.pdop(),
            horizontal_accuracy_mm: pvt.horiz_accuracy(),
            vertical_accuracy_mm: pvt.vert_accuracy(),
            speed_accuracy_mps: pvt.speed_accuracy_estimate() as f32,
        },
    };
    let cal = calibration::gnss::apply_calibration(raw);
    signals::submit_sd_log(SdLogRecord::Gnss { raw, cal });
    cal
}

fn has_fix(pvt: &Pvt) -> bool {
    pvt.fix_ok && matches!(pvt.fix_type, GpsFix::Fix2D | GpsFix::Fix3D)
}

fn log_configuration_packet(id: GnssId, packet: &PacketRef<'_>) {
    match packet {
        PacketRef::AckAck(ack) => {
            info!("{}: UBX ACK class={} id={}", id, ack.class(), ack.msg_id())
        }
        PacketRef::AckNak(nak) => {
            warn!("{}: UBX NAK class={} id={}", id, nak.class(), nak.msg_id())
        }
        PacketRef::MonVer(version) => {
            info!(
                "{}: receiver software={} hardware={}",
                id,
                version.software_version(),
                version.hardware_version()
            );
            for extension in version.extension() {
                info!("{}: receiver extension={}", id, extension);
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
                "{}: CFG-RATE readback: measure={} ms, nav_rate={}",
                id, measure_rate_ms, nav_rate
            );
        }
        _ => {}
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

    let inactive = Inactive {
        rx,
        parser,
        tx,
        id,
        attempt: 0,
    };
    super::run(&GNSS_STATUS[id.index()], inactive).await
}
