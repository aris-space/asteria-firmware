//! USB-CDC serial console. A line is read over the USB-C port and dispatched
//! to a command; commands run on-board routines (calibration) and manage the
//! stored calibration keys. One task runs the USB device, one runs the console.

use core::fmt::{self, Write as _};
use core::str::SplitAsciiWhitespace;

use embassy_executor::Spawner;
use embassy_time::Instant;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};
use embedded_io_async_06::{ErrorType, Read, Write};
use heapless::String;
use noline::builder::EditorBuilder;
use static_cell::StaticCell;

use crate::calibration::{self, Calibrations, Correction, baro, dht, gnss, imu, mag};
use crate::resources::flash;
use crate::resources::usb::UsbDriver;
use crate::sensors::SensorId;
use crate::signals;
use crate::storage::{self, Storage};
use crate::types::Mark;

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
  cal set <id> <key>=<value>...  store a line printed by the calibration tool
  cal show                       print the applied calibration as cal set lines
  mark <label>                   label this moment in the SD log, e.g. 'mark still'
  flash info                     show chip id and status register
  flash list                     list the keys you can clear
  flash clear <key>              clear a key (needs --yes)
  flash erase                    wipe all stored config (needs --yes)
  reset                          reboot the board
  help                           show this
";

/// Build the USB device + CDC class from static buffers and spawn the device
/// and console tasks.
pub fn spawn(driver: UsbDriver, storage: &'static Storage, spawner: Spawner) {
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
    // Fits the longest `cal set` line, the magnetometer's.
    let mut line_buf = [0u8; 256];
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
        Some("mark") => cmd_mark(class, &mut args).await,
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
    match (args.next(), args.next()) {
        (Some("set"), Some(sensor)) => match calibration::set(storage, sensor, args).await {
            Ok(()) => {
                sayf(
                    class,
                    format_args!(paint!(green, "stored {}; reset to apply\n"), sensor),
                )
                .await
            }
            Err(reason) => {
                sayf(
                    class,
                    format_args!(paint!(red, "{} not stored: {}\n"), sensor, reason),
                )
                .await
            }
        },
        (Some("show"), None) => {
            show_cals(class, storage, &imu::CAL).await;
            show_cals(class, storage, &mag::CAL).await;
            show_cals(class, storage, &gnss::CAL).await;
            show_cals(class, storage, &baro::CAL).await;
            show_cals(class, storage, &dht::CAL).await;
        }
        _ => {
            say(
                class,
                paint!(red, "usage: cal <set <id> <key>=<value>...|show>\n"),
            )
            .await
        }
    }
}

/// Prints each applied calibration as the `cal set` line that stores it, and
/// any different one waiting in flash for a reset.
async fn show_cals<Id: SensorId, C: Correction, const N: usize>(
    class: &mut ConsoleIo<'_>,
    storage: &Storage,
    cals: &Calibrations<Id, C, N>,
) {
    for id in cals.ids() {
        let applied = cals.applied(id);
        sayf(class, format_args!("cal set {} {}\n", id.name(), applied)).await;
        if let Some(stored) = cals.stored(storage, id).await.filter(|s| *s != applied) {
            sayf(
                class,
                format_args!(
                    paint!(yellow, "  in flash, applies after reset: cal set {} {}\n"),
                    id.name(),
                    stored
                ),
            )
            .await;
        }
    }
}

async fn cmd_mark(class: &mut ConsoleIo<'_>, args: &mut SplitAsciiWhitespace<'_>) {
    let label = match (args.next(), args.next()) {
        (Some(label), None) if !label.contains(',') => String::try_from(label).ok(),
        _ => None,
    };
    match label {
        Some(label) => {
            sayf(class, format_args!("marked {}\n", label)).await;
            signals::submit_mark(Mark {
                ts: Instant::now(),
                label,
            });
        }
        None => {
            say(
                class,
                paint!(
                    red,
                    "usage: mark <label> (one word, up to 16 characters, no commas)\n"
                ),
            )
            .await
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
    sayf(
        class,
        format_args!(
            "jedec id:   {:02x} {:02x} {:02x}\ndetected:   {detected}\nstatus reg: {:#04x} (wip={})\ncapacity:   {} MiB, config region {} KiB\n",
            id.manufacturer,
            id.memory_type,
            id.capacity,
            status,
            status & 1,
            flash::CAPACITY / (1024 * 1024),
            storage::CONFIG_LEN / 1024,
        ),
    )
    .await;
}

async fn flash_list(class: &mut ConsoleIo<'_>) {
    list_keys(class, &imu::CAL).await;
    list_keys(class, &mag::CAL).await;
    list_keys(class, &gnss::CAL).await;
    list_keys(class, &baro::CAL).await;
    list_keys(class, &dht::CAL).await;
}

async fn list_keys<Id: SensorId, C: Correction, const N: usize>(
    class: &mut ConsoleIo<'_>,
    cals: &Calibrations<Id, C, N>,
) {
    for id in cals.ids() {
        let key = calibration::key(id);
        let len = key.iter().position(|&b| b == 0).unwrap_or(key.len());
        let name = core::str::from_utf8(&key[..len]).unwrap_or("?");
        sayf(class, format_args!("{name}\n")).await;
    }
}

async fn flash_clear(class: &mut ConsoleIo<'_>, storage: &Storage, name: &str) {
    if name.len() > storage::KEY_LEN {
        sayf(
            class,
            format_args!(
                paint!(red, "key name too long (max {})\n"),
                storage::KEY_LEN
            ),
        )
        .await;
    } else if storage.remove(&storage::key(name)).await {
        sayf(
            class,
            format_args!(paint!(green, "cleared {}; reset to apply.\n"), name),
        )
        .await;
    } else {
        say(class, paint!(red, "flash error\n")).await;
    }
}

async fn flash_erase(class: &mut ConsoleIo<'_>, storage: &Storage) {
    let msg = if storage.erase().await {
        paint!(green, "erased flash; reset to apply.\n")
    } else {
        paint!(red, "flash error\n")
    };
    say(class, msg).await;
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

/// [`say`] for a formatted message.
async fn sayf(class: &mut ConsoleIo<'_>, args: fmt::Arguments<'_>) {
    let mut msg: String<512> = String::new();
    let _ = msg.write_fmt(args);
    say(class, &msg).await;
}
