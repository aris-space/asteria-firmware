//! USB-CDC serial console. A line is read over the USB-C port and dispatched
//! to a command: storing and showing calibration, marking the SD log, and
//! managing the flash. One task runs the USB device, one runs the console.

#[macro_use]
mod io;
mod cal;
mod flash;

use core::str::SplitAsciiWhitespace;

use embassy_executor::Spawner;
use embassy_time::Instant;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};
use heapless::String;
use noline::builder::EditorBuilder;
use static_cell::StaticCell;

use self::io::{ConsoleIo, say, sayf};
use crate::resources::usb::UsbDriver;
use crate::signals;
use crate::storage::Storage;
use crate::types::Mark;

type Class = CdcAcmClass<'static, UsbDriver>;

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
            say(
                io,
                paint!(
                    red,
                    "usage: mark <label> (one word, up to 16 characters, no commas)\n"
                ),
            )
            .await
        }
    }
}
