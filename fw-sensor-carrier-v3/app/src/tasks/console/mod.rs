//! USB-CDC serial console. A line is read over the USB-C port and dispatched
//! to a command: storing and showing calibration, marking the SD log, and
//! managing the flash. One task runs the USB device, one runs the console.

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

mod cal;
mod flash;
mod io;

use core::str::SplitAsciiWhitespace;

use embassy_executor::Spawner;
use embassy_time::Instant;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};
use heapless::String;
use noline::builder::EditorBuilder;
use static_cell::StaticCell;

use self::io::{ConsoleIo, PACKET_SIZE, say, sayf};
use crate::resources::usb::UsbDriver;
use crate::signals;
use crate::storage::Storage;
use crate::types::Mark;

type Class = CdcAcmClass<'static, UsbDriver>;

const DESCRIPTOR_LEN: usize = 256;
// Endpoint 0's packet size.
const CONTROL_BUF_LEN: usize = 64;
// Fits the longest `cal set` line, the magnetometer's.
const LINE_LEN: usize = 256;
const HISTORY_LEN: usize = 256;

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
    static CONFIG_DESC: StaticCell<[u8; DESCRIPTOR_LEN]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; DESCRIPTOR_LEN]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; CONTROL_BUF_LEN]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let mut config = embassy_usb::Config::new(0xc0de, 0xca10);
    config.manufacturer = Some("Asteria");
    config.product = Some("Sensor Carrier v3");
    config.serial_number = Some("scv3");

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; DESCRIPTOR_LEN]),
        BOS_DESC.init([0; DESCRIPTOR_LEN]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; CONTROL_BUF_LEN]),
    );
    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), PACKET_SIZE as u16);
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
    let mut line_buf = [0u8; LINE_LEN];
    let mut history = [0u8; HISTORY_LEN];
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

async fn handle_line(io: &mut ConsoleIo<'_>, line: &str, storage: &Storage) {
    let mut args = line.split_ascii_whitespace();
    match args.next() {
        None => {}
        Some("help") => say(io, HELP).await,
        Some("cal") => cal::command(io, &mut args, storage).await,
        Some("mark") => mark(io, &mut args).await,
        Some("flash") => flash::command(io, &mut args, storage).await,
        Some("reset") => {
            say(io, "resetting...\n").await;
            cortex_m::peripheral::SCB::sys_reset();
        }
        Some(_) => say(io, paint!(red, "unknown command (try 'help')\n")).await,
    }
}

async fn mark(io: &mut ConsoleIo<'_>, args: &mut SplitAsciiWhitespace<'_>) {
    let label = match (args.next(), args.next()) {
        (Some(label), None) if !label.contains(',') => String::try_from(label).ok(),
        _ => None,
    };
    match label {
        Some(label) => {
            sayf(io, format_args!("marked {}\n", label)).await;
            signals::submit_mark(Mark {
                ts: Instant::now(),
                label,
            });
        }
        None => {
            sayf(
                io,
                format_args!(
                    paint!(
                        red,
                        "usage: mark <label> (one word, up to {} characters, no commas)\n"
                    ),
                    Mark::LABEL_LEN
                ),
            )
            .await
        }
    }
}
