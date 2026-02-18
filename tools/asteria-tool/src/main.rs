//! Host CLI tool for interacting with Asteria boards over USB.
//!
//! Supports:
//! - path-based virtual filesystem commands (`fs ...`)
//! - interactive shell (`shell`)

mod connection;
mod fs_cmd;
mod ops;
mod shell;
mod vfs;

use std::io::IsTerminal;
use std::time::Instant;

use anyhow::Result;
use clap::{Parser, Subcommand};
use connection::TransportMode;
use fs_cmd::FsAction;

const CLI_AFTER_HELP: &str = "\
Examples:
  asteria-tool shell
  asteria-tool shell --connect raw
  asteria-tool shell --stats
  asteria-tool shell --connect serial --port /dev/ttyACM0
  asteria-tool fs ls /
  asteria-tool fs ls -s /logs_25
  asteria-tool fs du /
  asteria-tool fs du -d 1 /
  asteria-tool fs du -a -d 3 /
  asteria-tool fs du -r /
  asteria-tool --stats fs ls /
  asteria-tool fs cat /logs_25/build_info.txt
  asteria-tool fs info
  asteria-tool fs pull /logs_25/defmt.bin ~/Downloads/defmt.bin
  asteria-tool fs pull -r /logs_25 ~/Downloads
  asteria-tool fs rm /logs_12/build_info.txt
  asteria-tool fs rm -r /logs_12
  asteria-tool fs erase-storage
  asteria-tool reset
  asteria-tool panic
  asteria-tool panic \"panic test message\"

Notes:
  - Remote path is always the first argument for pull/cp style commands.
  - `--stats` prints elapsed time for commands.
  - `rm` refuses deleting `/`, and board may reject deleting active/in-use logs.
  - `erase-storage` is explicit destructive erase of external flash and reboots the board.
  - `reset` triggers a board reboot.
  - `panic` triggers an intentional firmware panic then reboot (message max 64 chars).
  - Shell auto-reconnects after board restarts/disconnects.
";

#[derive(Parser)]
#[command(name = "asteria-tool")]
#[command(about = "Interact with Asteria boards over USB")]
#[command(after_help = CLI_AFTER_HELP)]
struct Cli {
    /// Serial port path (for --connect serial or auto fallback).
    #[arg(short, long, global = true)]
    port: Option<String>,

    /// Baud rate for serial mode.
    #[arg(short, long, default_value = "115200", global = true)]
    baud: u32,

    /// Connection transport: auto/raw/serial.
    #[arg(long, value_enum, default_value_t = TransportMode::Auto, global = true)]
    connect: TransportMode,

    /// Print operation timing stats.
    #[arg(long, global = true)]
    stats: bool,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Interactive shell mode (`ls`, `cd`, `cat`, `pull`, ...)
    Shell,
    /// Path-based virtual filesystem commands (script-friendly).
    Fs {
        #[command(subcommand)]
        action: FsAction,
    },
    /// Reboot the board.
    Reset,
    /// Trigger an intentional firmware panic (for panic/recovery validation).
    Panic {
        /// Optional panic message (max 64 chars).
        message: Option<String>,
    },
    /// Get runtime + filesystem status info.
    Info,
}

#[tokio::main]
async fn main() {
    if let Err(e) = run().await {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

async fn run() -> Result<()> {
    let cli = Cli::parse();
    let mode = cli.connect;
    let stats = cli.stats;

    match cli.command {
        Commands::Shell => {
            shell::run_shell(mode, cli.port.as_deref(), cli.baud, stats).await?;
        }
        Commands::Fs { action } => {
            let (client, _connected) =
                connection::connect(mode, cli.port.as_deref(), cli.baud).await?;
            fs_cmd::run_fs_action(&client, action, stats).await?;
        }
        Commands::Info => {
            let (client, _connected) =
                connection::connect(mode, cli.port.as_deref(), cli.baud).await?;
            let started = Instant::now();
            let report = ops::build_storage_report(&client).await?;
            println!("{}", ops::format_storage_report(&report));
            if stats {
                print_stats_line(&format!(
                    "info took {:.3}s",
                    started.elapsed().as_secs_f64()
                ));
            }
        }
        Commands::Reset => {
            let (client, _connected) =
                connection::connect(mode, cli.port.as_deref(), cli.baud).await?;
            let started = Instant::now();
            ops::reset_board(&client).await?;
            println!("reset acknowledged; board will now restart");
            if stats {
                print_stats_line(&format!(
                    "reset took {:.3}s",
                    started.elapsed().as_secs_f64()
                ));
            }
        }
        Commands::Panic { message } => {
            let (client, _connected) =
                connection::connect(mode, cli.port.as_deref(), cli.baud).await?;
            let started = Instant::now();
            let message = message.unwrap_or_else(|| "panic requested via cli".to_string());
            ops::panic_board(&client, &message).await?;
            println!("panic acknowledged; board will now panic and restart: {message}");
            if stats {
                print_stats_line(&format!(
                    "panic took {:.3}s",
                    started.elapsed().as_secs_f64()
                ));
            }
        }
    }

    Ok(())
}

fn print_stats_line(msg: &str) {
    if use_color_for_stats() {
        eprintln!("\x1b[90mstats:\x1b[0m {msg}");
    } else {
        eprintln!("stats: {msg}");
    }
}

fn use_color_for_stats() -> bool {
    let term_ok = std::env::var("TERM").map(|v| v != "dumb").unwrap_or(true);
    std::io::stderr().is_terminal() && std::env::var_os("NO_COLOR").is_none() && term_ok
}
