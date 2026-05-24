//! USB-CDC serial console. A line is read over the USB-C port and dispatched
//! to a command; commands run on-board routines (calibration) and manage the
//! stored calibration keys. One task runs the USB device, one runs the console.

use core::fmt::Write as _;
use core::str::SplitAsciiWhitespace;

use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};
use heapless::{String, Vec};
use static_cell::StaticCell;

use crate::calibration::mag;
use crate::resources::flash;
use crate::resources::usb::UsbDriver;
use crate::storage::{self, Storage};

type Class = CdcAcmClass<'static, UsbDriver>;

const PROMPT: &str = "asteria> ";
const HELP: &str = r"commands:
  cal mag <name>     run magnetometer calibration (label required)
  cal show           show stored calibrations
  flash info         show chip id and status register
  flash test         erase/write/read-back a scratch sector
  flash list         list the keys you can clear
  flash clear <key>  clear a key
  flash erase        wipe all stored config
  reset              reboot the board
  help               show this
";

/// The keys the firmware reads/writes, shown by `flash list`.
const KEYS: &str = "mag0\nmag1\nimu0\nimu1\n";

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
    let mut line: Vec<u8, 128> = Vec::new();
    loop {
        class.wait_connection().await;
        say(&mut class, "\n").await;
        loop {
            say(&mut class, PROMPT).await;
            match read_line(&mut class, &mut line).await {
                Some(cmd) => handle_line(&mut class, cmd, storage).await,
                None => break, // host disconnected
            }
        }
    }
}

/// Read one newline-terminated command, echoing input and handling backspace.
/// Returns the trimmed command, or `None` if the host disconnected.
async fn read_line<'a>(class: &mut Class, line: &'a mut Vec<u8, 128>) -> Option<&'a str> {
    line.clear();
    let mut buf = [0u8; 64];
    loop {
        let n = class.read_packet(&mut buf).await.ok()?;
        for &b in &buf[..n] {
            match b {
                b'\r' | b'\n' => {
                    say(class, "\n").await;
                    return Some(core::str::from_utf8(line).unwrap_or("").trim());
                }
                0x08 | 0x7f => {
                    if line.pop().is_some() {
                        let _ = class.write_packet(b"\x08 \x08").await;
                    }
                }
                _ => {
                    if line.push(b).is_ok() {
                        let _ = class.write_packet(&[b]).await;
                    }
                }
            }
        }
    }
}

async fn handle_line(class: &mut Class, line: &str, storage: &Storage) {
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
        Some(_) => say(class, "unknown command (try 'help')\n").await,
    }
}

async fn cmd_cal(class: &mut Class, args: &mut SplitAsciiWhitespace<'_>, storage: &Storage) {
    match args.next() {
        Some("mag") => {
            let Some(name) = args.next() else {
                say(class, "usage: cal mag <name>\n").await;
                return;
            };
            if name.len() > 16 {
                say(class, "name too long (max 16 chars)\n").await;
                return;
            }
            say(
                class,
                "mag cal: tumble the board slowly through all orientations (~30s)\n",
            )
            .await;
            let reports = mag::run(storage, name, async |n0, n1| {
                let mut s: String<48> = String::new();
                let _ = writeln!(s, "  collecting... mag0={n0} mag1={n1}");
                say(class, &s).await;
            })
            .await;
            say(class, "results:\n").await;
            let mut stored = false;
            for r in &reports {
                let mut s: String<384> = String::new();
                let _ = writeln!(s, "{r}\n");
                say(class, &s).await;
                stored |= r.stored();
            }
            if stored {
                say(class, "written to flash; reset to apply.\n").await;
            }
        }
        Some("show") => cal_show(class, storage).await,
        Some("imu") => say(class, "imu cal not implemented yet\n").await,
        _ => say(class, "usage: cal <mag <name>|show>\n").await,
    }
}

async fn cal_show(class: &mut Class, storage: &Storage) {
    let applied = mag::applied();
    let stored = mag::stored(storage).await;
    for (i, label) in ["mag0", "mag1"].iter().enumerate() {
        let mut s: String<384> = String::new();
        let _ = writeln!(s, "{label} (applied) {}", applied[i]);
        // If flash holds a different cal than what's live, it's waiting on a reset.
        if let Some(st) = &stored[i]
            && (st.name != applied[i].name || st.field_nt != applied[i].field_nt)
        {
            let _ = writeln!(
                s,
                "  flash has \"{}\" pending; reset to apply",
                st.name_str()
            );
        }
        say(class, &s).await;
        say(class, "\n").await;
    }
}

async fn cmd_flash(class: &mut Class, args: &mut SplitAsciiWhitespace<'_>, storage: &Storage) {
    match args.next() {
        Some("info") => flash_info(class, storage).await,
        Some("test") => flash_test(class, storage).await,
        Some("list") => say(class, KEYS).await,
        Some("clear") => match args.next() {
            Some(name) => flash_clear(class, storage, name).await,
            None => say(class, "usage: flash clear <key>\n").await,
        },
        Some("erase") => {
            let msg = if storage.erase().await {
                "erased flash; reset to apply.\n"
            } else {
                "flash error\n"
            };
            say(class, msg).await;
        }
        _ => say(class, "usage: flash <list|clear <key>|erase>\n").await,
    }
}

async fn flash_info(class: &mut Class, storage: &Storage) {
    let id = storage.jedec_id().await;
    let status = storage.status().await;
    // Only the manufacturer byte is reliable over this OSPI path (the 0x9F
    // continuation bytes mis-clock); the chip's real health is `flash test`.
    let detected = if id[0] == flash::EXPECTED_JEDEC_ID[0] {
        "Winbond (id bytes 1-2 unreliable; use 'flash test')"
    } else {
        "UNKNOWN (check wiring/power)"
    };
    let mut s: String<192> = String::new();
    let _ = writeln!(s, "jedec id:   {:02x} {:02x} {:02x}", id[0], id[1], id[2]);
    let _ = writeln!(s, "detected:   {detected}");
    let _ = writeln!(s, "status reg: {:#04x} (wip={})", status, status & 1);
    let _ = writeln!(s, "capacity:   32 MiB, config region 64 KiB");
    say(class, &s).await;
}

async fn flash_test(class: &mut Class, storage: &Storage) {
    say(class, "flash self-test (scratch sector)...\n").await;
    match storage.self_test().await {
        Ok(()) => say(class, "self-test passed\n").await,
        Err(e) => {
            let mut s: String<48> = String::new();
            let _ = writeln!(s, "self-test FAILED: {e}");
            say(class, &s).await;
        }
    }
}

async fn flash_clear(class: &mut Class, storage: &Storage, name: &str) {
    if name.len() > 16 {
        say(class, "key name too long (max 16)\n").await;
        return;
    }
    let mut s: String<48> = String::new();
    if storage.remove(&storage::key(name)).await {
        let _ = writeln!(s, "cleared {name}; reset to apply.");
    } else {
        let _ = write!(s, "flash error");
    }
    say(class, &s).await;
}

/// Write `msg` to the host, translating each `\n` to CRLF so a serial terminal
/// returns to column 0. Best-effort: stops on the first write error (e.g. the
/// host disconnected); the read loop detects the disconnect.
async fn say(class: &mut Class, msg: &str) {
    for (i, segment) in msg.split('\n').enumerate() {
        if i > 0 && class.write_packet(b"\r\n").await.is_err() {
            return;
        }
        for chunk in segment.as_bytes().chunks(64) {
            if class.write_packet(chunk).await.is_err() {
                return;
            }
        }
    }
}
