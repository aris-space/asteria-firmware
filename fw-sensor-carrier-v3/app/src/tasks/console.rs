//! USB-CDC serial console. A line is read over the USB-C port and dispatched
//! to a command; commands run on-board routines (calibration) and manage the
//! stored calibration keys. One task runs the USB device, one runs the console.

use core::fmt::{self, Write as _};
use core::str::SplitAsciiWhitespace;

use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};
use embedded_io_async_06::{ErrorType, Read, Write};
use heapless::String;
use noline::builder::EditorBuilder;
use static_cell::StaticCell;

use crate::calibration::{Name, mag};
use crate::resources::flash;
use crate::resources::usb::UsbDriver;
use crate::sensors::MagnetometerId;
use crate::storage::{self, Storage};

type Class = CdcAcmClass<'static, UsbDriver>;

/// Wrap a string literal in an ANSI SGR colour, resetting after. The result is
/// itself a literal, so it works anywhere a `&str` is expected (e.g. a `say`
/// argument).
macro_rules! paint {
    (red, $s:literal) => {
        concat!("\x1b[31m", $s, "\x1b[0m")
    };
    (green, $s:literal) => {
        concat!("\x1b[32m", $s, "\x1b[0m")
    };
    (yellow, $s:literal) => {
        concat!("\x1b[33m", $s, "\x1b[0m")
    };
}

/// `embedded-io-async` adapter over the CDC class so noline can read and write.
/// CDC reads come a USB packet at a time, so a one-packet buffer hands bytes out
/// in whatever chunk sizes the caller asks for; writes go out a packet at a time.
struct ConsoleIo<'a> {
    class: &'a mut Class,
    rx: [u8; 64],
    rx_pos: usize,
    rx_len: usize,
}

impl<'a> ConsoleIo<'a> {
    fn new(class: &'a mut Class) -> Self {
        Self {
            class,
            rx: [0; 64],
            rx_pos: 0,
            rx_len: 0,
        }
    }
}

#[derive(Debug)]
struct IoError;

impl embedded_io_async_06::Error for IoError {
    fn kind(&self) -> embedded_io_async_06::ErrorKind {
        embedded_io_async_06::ErrorKind::Other
    }
}

impl ErrorType for ConsoleIo<'_> {
    type Error = IoError;
}

impl Read for ConsoleIo<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, IoError> {
        // Refill from a fresh packet, skipping any zero-length ones so this
        // never reports a false end-of-stream.
        while self.rx_pos >= self.rx_len {
            self.rx_len = self
                .class
                .read_packet(&mut self.rx)
                .await
                .map_err(|_| IoError)?;
            self.rx_pos = 0;
        }
        let n = buf.len().min(self.rx_len - self.rx_pos);
        buf[..n].copy_from_slice(&self.rx[self.rx_pos..self.rx_pos + n]);
        self.rx_pos += n;
        Ok(n)
    }
}

impl Write for ConsoleIo<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, IoError> {
        let n = buf.len().min(64);
        self.class
            .write_packet(&buf[..n])
            .await
            .map_err(|_| IoError)?;
        Ok(n)
    }
}

const PROMPT: &str = "asteria> ";
const HELP: &str = r"commands:
  cal mag <name>     run magnetometer calibration (label required)
  cal show           show stored calibrations
  flash info         show chip id and status register
  flash test         erase/write/read-back a scratch sector
  flash list         list the keys you can clear
  flash clear <key>  clear a key (needs --yes)
  flash erase        wipe all stored config (needs --yes)
  reset              reboot the board
  help               show this
";

/// Build the USB device + CDC class from static buffers and spawn the device
/// and console tasks.
pub fn start(driver: UsbDriver, storage: &'static Storage, spawner: Spawner) {
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let mut config = embassy_usb::Config::new(0xc0de, 0xca10);
    config.manufacturer = Some("Asteria");
    config.product = Some("Sensor Carrier v3");
    config.serial_number = Some("scv3");

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);
    let usb = builder.build();

    spawner.spawn(usb_device_task(usb).expect("spawn usb device task"));
    spawner.spawn(console_task(class, storage).expect("spawn console task"));
}

#[embassy_executor::task]
async fn usb_device_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

#[embassy_executor::task]
async fn console_task(mut class: Class, storage: &'static Storage) -> ! {
    let mut line_buf = [0u8; 128];
    let mut history = [0u8; 256];
    loop {
        class.wait_connection().await;
        let mut io = ConsoleIo::new(&mut class);
        // build_async probes the terminal (cursor query); a disconnect here just
        // sends us back to waiting for the next connection.
        let Ok(mut editor) = EditorBuilder::from_slice(&mut line_buf)
            .with_slice_history(&mut history)
            .build_async(&mut io)
            .await
        else {
            continue;
        };
        // `line` borrows the editor, not `io`, so handler output can keep flowing
        // through `io` while the parsed command is in hand. A read error means the
        // host disconnected or the terminal failed; drop back to wait_connection.
        while let Ok(line) = editor.readline(PROMPT, &mut io).await {
            handle_line(&mut io, line, storage).await;
        }
    }
}

async fn handle_line(class: &mut ConsoleIo<'_>, line: &str, storage: &Storage) {
    let mut args = line.split_ascii_whitespace();
    match args.next() {
        None => {}
        Some("help") => say(class, HELP).await,
        Some("cal") => cmd_cal(class, &mut args, storage).await,
        Some("flash") => cmd_flash(class, &mut args, storage).await,
        Some("reset") => {
            say(class, "resetting...\n").await;
            cortex_m::peripheral::SCB::sys_reset();
        }
        Some(_) => say(class, paint!(red, "unknown command (try 'help')\n")).await,
    }
}

async fn cmd_cal(
    class: &mut ConsoleIo<'_>,
    args: &mut SplitAsciiWhitespace<'_>,
    storage: &Storage,
) {
    match args.next() {
        Some("mag") => cmd_cal_mag(class, args, storage).await,
        Some("show") => cal_show(class, storage).await,
        _ => say(class, paint!(red, "usage: cal <mag <name>|show>\n")).await,
    }
}

async fn cmd_cal_mag(
    class: &mut ConsoleIo<'_>,
    args: &mut SplitAsciiWhitespace<'_>,
    storage: &Storage,
) {
    let Some(name) = cal_name_or_report(class, args, paint!(red, "usage: cal mag <name>\n")).await
    else {
        return;
    };
    say(
        class,
        "mag cal: tumble the board slowly through all orientations (~30s)\n",
    )
    .await;
    let mut cal = mag::MagCal::default();
    for _ in 0..mag::PROGRESS_TICKS {
        cal.collect_tick().await;
        let n = cal.counts();
        let mut s: String<48> = String::new();
        let _ = writeln!(s, "  collecting... mag0={} mag1={}", n[0], n[1]);
        say(class, &s).await;
    }
    say(class, "results:\n").await;
    let reports = cal.finish(name, storage).await;
    let mut stored = false;
    for r in &reports {
        let mut s: String<384> = String::new();
        let _ = writeln!(s, "{r}\n");
        say(class, &s).await;
        stored |= r.stored();
    }
    report_outcome(class, stored).await;
}

async fn cal_show(class: &mut ConsoleIo<'_>, storage: &Storage) {
    show_mag_cals(class, storage).await;
}

async fn show_mag_cals(class: &mut ConsoleIo<'_>, storage: &Storage) {
    let applied = mag::applied();
    let stored = mag::stored(storage).await;
    for id in MagnetometerId::ALL {
        let i = id.index();
        let pending = stored[i]
            .filter(|st| st.differs_from(&applied[i]))
            .map(|st| st.name);
        show_cal_slot(class, id.name(), applied[i], pending).await;
    }
}

async fn show_cal_slot(
    class: &mut ConsoleIo<'_>,
    label: &str,
    applied: impl fmt::Display,
    pending: Option<Name>,
) {
    let mut s: String<384> = String::new();
    let _ = writeln!(s, "{label} {} {}", paint!(green, "(applied)"), applied);
    if let Some(name) = pending {
        write_pending_cal(&mut s, name);
    }
    say(class, &s).await;
    say(class, "\n").await;
}

fn write_pending_cal(out: &mut impl fmt::Write, name: Name) {
    let _ = writeln!(
        out,
        paint!(yellow, "  flash has \"{}\" pending; reset to apply"),
        name
    );
}

async fn report_outcome(class: &mut ConsoleIo<'_>, stored: bool) {
    if stored {
        say(class, paint!(green, "written to flash; reset to apply.\n")).await;
    } else {
        say(class, paint!(red, "NOT stored (see results above).\n")).await;
    }
}

#[derive(Clone, Copy)]
enum NameError {
    Missing,
    TooLong,
}

fn parse_cal_name<'a>(args: &mut SplitAsciiWhitespace<'a>) -> Result<&'a str, NameError> {
    let name = args.next().ok_or(NameError::Missing)?;
    if name.len() > Name::CAP {
        return Err(NameError::TooLong);
    }
    Ok(name)
}

async fn cal_name_or_report<'a>(
    class: &mut ConsoleIo<'_>,
    args: &mut SplitAsciiWhitespace<'a>,
    usage: &'static str,
) -> Option<&'a str> {
    match parse_cal_name(args) {
        Ok(name) => Some(name),
        Err(NameError::Missing) => {
            say(class, usage).await;
            None
        }
        Err(NameError::TooLong) => {
            let mut s: String<48> = String::new();
            let _ = writeln!(s, paint!(red, "name too long (max {} chars)"), Name::CAP);
            say(class, &s).await;
            None
        }
    }
}

async fn cmd_flash(
    class: &mut ConsoleIo<'_>,
    args: &mut SplitAsciiWhitespace<'_>,
    storage: &Storage,
) {
    match args.next() {
        Some("info") => flash_info(class, storage).await,
        Some("test") => flash_test(class, storage).await,
        Some("list") => flash_list(class).await,
        Some("clear") => match args.next() {
            Some(name) if args.next() == Some("--yes") => flash_clear(class, storage, name).await,
            Some(_) => {
                say(
                    class,
                    paint!(
                        yellow,
                        "refusing: this clears a stored key. re-run as 'flash clear <key> --yes'\n"
                    ),
                )
                .await
            }
            None => say(class, paint!(red, "usage: flash clear <key> --yes\n")).await,
        },
        Some("erase") if args.next() == Some("--yes") => flash_erase(class, storage).await,
        Some("erase") => {
            say(
                class,
                paint!(
                    yellow,
                    "refusing: this wipes ALL stored config. re-run as 'flash erase --yes'\n"
                ),
            )
            .await
        }
        _ => {
            say(
                class,
                paint!(red, "usage: flash <list|clear <key> --yes|erase --yes>\n"),
            )
            .await
        }
    }
}

async fn flash_info(class: &mut ConsoleIo<'_>, storage: &Storage) {
    let id = storage.read_jedec_id().await;
    let status = storage.status().await;
    let mut s: String<192> = String::new();
    write_flash_identity(&mut s, &id, status);
    say(class, &s).await;
}

fn write_flash_identity(out: &mut impl fmt::Write, id: &flash::JedecId, status: u8) {
    let detected = if id.manufacturer == flash::WINBOND_MANUFACTURER_ID
        && id.memory_type == flash::W25Q_IM_MEMORY_TYPE
        && id.capacity == flash::W25Q01JV_CAPACITY
    {
        "W25Q01JV"
    } else if id.manufacturer == flash::WINBOND_MANUFACTURER_ID {
        "Winbond, unexpected JEDEC id"
    } else {
        "UNKNOWN (check wiring/power)"
    };
    let _ = writeln!(
        out,
        "jedec id:   {:02x} {:02x} {:02x}",
        id.manufacturer, id.memory_type, id.capacity
    );
    let _ = writeln!(out, "detected:   {detected}");
    let _ = writeln!(out, "status reg: {:#04x} (wip={})", status, status & 1);
    let _ = write!(out, "capacity:   ");
    let _ = write_bytes(out, flash::CAPACITY);
    let _ = write!(out, ", config region ");
    let _ = write_bytes(out, storage::CONFIG_LEN);
    let _ = writeln!(out);
}

async fn flash_test(class: &mut ConsoleIo<'_>, storage: &Storage) {
    say(class, "flash self-test (scratch sector)...\n").await;
    match storage.self_test().await {
        Ok(()) => say(class, paint!(green, "self-test passed\n")).await,
        Err(e) => {
            let mut s: String<48> = String::new();
            let _ = writeln!(s, paint!(red, "self-test FAILED: {}"), e);
            say(class, &s).await;
        }
    }
}

async fn flash_list(class: &mut ConsoleIo<'_>) {
    for id in MagnetometerId::ALL {
        let mut s: String<24> = String::new();
        let _ = writeln!(s, "{}", id.name());
        say(class, &s).await;
    }
}

async fn flash_clear(class: &mut ConsoleIo<'_>, storage: &Storage, name: &str) {
    if name.len() > storage::KEY_LEN {
        let mut s: String<48> = String::new();
        let _ = writeln!(
            s,
            paint!(red, "key name too long (max {})"),
            storage::KEY_LEN
        );
        say(class, &s).await;
        return;
    }
    let mut s: String<48> = String::new();
    if storage.remove(&storage::key(name)).await {
        let _ = writeln!(s, paint!(green, "cleared {}; reset to apply."), name);
    } else {
        let _ = write!(s, paint!(red, "flash error"));
    }
    say(class, &s).await;
}

async fn flash_erase(class: &mut ConsoleIo<'_>, storage: &Storage) {
    let msg = if storage.erase().await {
        paint!(green, "erased flash; reset to apply.\n")
    } else {
        paint!(red, "flash error\n")
    };
    say(class, msg).await;
}

fn write_bytes(out: &mut impl fmt::Write, bytes: u32) -> fmt::Result {
    const KIB: u32 = 1024;
    const MIB: u32 = 1024 * KIB;
    if bytes.is_multiple_of(MIB) {
        write!(out, "{} MiB", bytes / MIB)
    } else if bytes.is_multiple_of(KIB) {
        write!(out, "{} KiB", bytes / KIB)
    } else {
        write!(out, "{} B", bytes)
    }
}

/// Write `msg` to the host, translating each `\n` to CRLF so a serial terminal
/// returns to column 0. Best-effort: stops on the first write error (e.g. the
/// host disconnected); the read loop detects the disconnect.
async fn say(class: &mut ConsoleIo<'_>, msg: &str) {
    for (i, segment) in msg.split('\n').enumerate() {
        if i > 0 && class.write_all(b"\r\n").await.is_err() {
            return;
        }
        if class.write_all(segment.as_bytes()).await.is_err() {
            return;
        }
    }
}
