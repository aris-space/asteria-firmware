//! u-blox GNSS readout over UART. NAV-PVT epochs are logged to SD from the
//! first packet; only fixes reach the estimator. `Inactive` already reports
//! the UART link as active before the first fix.

use core::sync::atomic::Ordering;

use defmt::{Debug2Format, debug, error, info, warn};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use ublox::{GpsFix, NavPvtFlags, PacketRef, Parser};

use super::{MAX_CONSECUTIVE_ERRORS, State, backoff};
use crate::calibration;
use crate::sensors::{GNSS_STATUS, GnssId, SensorStatus};
use crate::signals;
use crate::types::{GnssSample, Pvt, RawGnssSample, SdLogRecord};

// The receivers send NAV-PVT every 50 ms; a two-second gap means the link is silent.
const LINK_SILENCE_TIMEOUT: Duration = Duration::from_secs(2);

struct Inactive<'a, RX> {
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
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
                return Active {
                    rx: self.rx,
                    parser: self.parser,
                    id: self.id,
                };
            }
        }
    }
}

struct Active<'a, RX> {
    rx: RX,
    parser: Parser<ublox::FixedLinearBuffer<'a>>,
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
                            Ok(_) => {}
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

#[embassy_executor::task(pool_size = 2)]
pub async fn task(rx: UartRx<'static, Async>, id: GnssId) -> ! {
    let mut uart_ring_buf = [0u8; 4096];
    let mut parse_buf = [0u8; 4096];
    let inactive = Inactive {
        rx: rx.into_ring_buffered(&mut uart_ring_buf),
        parser: Parser::new(ublox::FixedLinearBuffer::new(&mut parse_buf)),
        id,
        attempt: 0,
    };
    super::run(&GNSS_STATUS[id.index()], inactive).await
}
