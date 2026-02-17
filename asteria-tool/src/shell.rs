use std::io::{self, IsTerminal, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use rustyline::DefaultEditor;
use rustyline::error::ReadlineError;
use terminal_size::{Width, terminal_size};

use crate::connection::{self, TransportMode};
use crate::fs_cmd;
use crate::ops::RpcBackend;

pub struct SessionState {
    pub cwd: String,
    pub transport: TransportMode,
    pub connected_via: String,
    pub device_name: Option<String>,
    pub stats: bool,
}

#[derive(Clone, Copy)]
struct UiStyle {
    color: bool,
}

impl UiStyle {
    fn detect() -> Self {
        let term_ok = std::env::var("TERM").map(|v| v != "dumb").unwrap_or(true);
        let color = io::stdout().is_terminal()
            && io::stderr().is_terminal()
            && std::env::var_os("NO_COLOR").is_none()
            && term_ok;
        Self { color }
    }

    fn prompt(&self, cwd: &str, connected: &str) -> String {
        if !self.color {
            return format!("asteria:{} [{}]> ", cwd, connected);
        }
        format!(
            "{}:{} [{}]> ",
            self.wrap("asteria", "1;32"),
            self.wrap(cwd, "1;34"),
            self.wrap(connected, "36")
        )
    }

    fn info_label(&self, label: &str) -> String {
        if self.color {
            self.wrap(label, "90")
        } else {
            label.to_string()
        }
    }

    fn error_label(&self, label: &str) -> String {
        if self.color {
            self.wrap(label, "1;31")
        } else {
            label.to_string()
        }
    }

    fn wrap(&self, text: &str, ansi: &str) -> String {
        format!("\x1b[{ansi}m{text}\x1b[0m")
    }
}

pub async fn run_shell(
    transport: TransportMode,
    port: Option<&str>,
    baud: u32,
    stats: bool,
) -> Result<()> {
    let style = UiStyle::detect();
    let mut state = SessionState {
        cwd: "/".to_string(),
        transport,
        connected_via: "disconnected".to_string(),
        device_name: None,
        stats,
    };

    let mut client = Some(reconnect(transport, port, baud, &mut state).await?);
    println!(
        "{} Asteria shell connected via {}",
        style.info_label("status:"),
        state.connected_via
    );
    if let Some(name) = &state.device_name {
        println!("Device: {name}");
    }
    println!("Connection mode: {}", mode_label(state.transport));
    println!("Type `help` for commands.");

    let mut editor = DefaultEditor::new()?;
    let history_path = history_path();
    if let Some(path) = history_path.as_deref() {
        let _ = editor.load_history(path);
    }

    loop {
        let prompt = style.prompt(&state.cwd, &state.connected_via);
        let line = match editor.readline(&prompt) {
            Ok(line) => line,
            Err(ReadlineError::Interrupted) => continue,
            Err(ReadlineError::Eof) => break,
            Err(e) => {
                if let Some(path) = history_path.as_deref() {
                    let _ = editor.save_history(path);
                }
                bail!("read error: {e}");
            }
        };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let _ = editor.add_history_entry(line);

        let tokens = match shell_words::split(line) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("{} parse error: {e}", style.error_label("error:"));
                continue;
            }
        };

        let cmd_started = Instant::now();
        let mut retry_current = false;
        let mut print_stats_for_command = true;
        let mut command_should_continue = true;
        loop {
            enum ShellEvent {
                Command(Result<bool>),
                Interrupted,
            }

            let event = {
                let client_ref = client
                    .as_ref()
                    .ok_or_else(|| anyhow::anyhow!("internal error: missing client"))?;
                tokio::select! {
                    r = dispatch_tokens(client_ref, &mut state, &tokens) => ShellEvent::Command(r),
                    _ = tokio::signal::ctrl_c() => ShellEvent::Interrupted,
                }
            };

            let continue_loop = match event {
                ShellEvent::Interrupted => {
                    print_stats_for_command = false;
                    eprintln!("\n{} interrupted", style.info_label("status:"));
                    reconnect_after_drop(&mut client, transport, port, baud, &mut state).await?;
                    break;
                }
                ShellEvent::Command(result) => match result {
                    Ok(v) => {
                        if v && command_reboots_board(&tokens) {
                            eprintln!(
                                "{} waiting for board reboot...",
                                style.info_label("status:")
                            );
                            reconnect_after_drop(
                                &mut client,
                                transport,
                                port,
                                baud,
                                &mut state,
                            )
                            .await?;
                            println!("{} reconnected", style.info_label("status:"));
                        }
                        v
                    }
                    Err(e) if is_disconnect_error(&e) => {
                        eprintln!("{} link lost, reconnecting...", style.info_label("status:"));
                        reconnect_after_drop(&mut client, transport, port, baud, &mut state)
                            .await?;
                        println!("{} reconnected", style.info_label("status:"));
                        retry_current = true;
                        true
                    }
                    Err(e) => {
                        eprintln!("{} {e}", style.error_label("error:"));
                        true
                    }
                },
            };
            if !continue_loop {
                command_should_continue = false;
                if let Some(path) = history_path.as_deref() {
                    let _ = editor.save_history(path);
                }
                break;
            }
            if retry_current {
                retry_current = false;
                continue;
            }
            break;
        }

        if !command_should_continue {
            return Ok(());
        }
        if state.stats && print_stats_for_command {
            let _ = io::stdout().flush();
            eprintln!(
                "{} {} took {:.3}s",
                style.info_label("stats:"),
                command_label(&tokens),
                cmd_started.elapsed().as_secs_f64()
            );
        }
    }

    if let Some(path) = history_path.as_deref() {
        let _ = editor.save_history(path);
    }
    Ok(())
}

async fn reconnect(
    mode: TransportMode,
    port: Option<&str>,
    baud: u32,
    state: &mut SessionState,
) -> Result<HostClient<WireError>> {
    let mut attempts: u32 = 0;
    loop {
        match connection::connect(mode, port, baud).await {
            Ok((client, connected)) => {
                state.connected_via = connected.label();
                state.device_name = connected.device_name();
                return Ok(client);
            }
            Err(e) => {
                attempts = attempts.saturating_add(1);
                if attempts == 1 {
                    eprintln!("status: waiting for board... ({})", compact_err(&e));
                } else if attempts % 10 == 0 {
                    eprintln!("status: still waiting for board...");
                }
                tokio::time::sleep(Duration::from_millis(600)).await;
            }
        }
    }
}

async fn reconnect_after_drop(
    client: &mut Option<HostClient<WireError>>,
    mode: TransportMode,
    port: Option<&str>,
    baud: u32,
    state: &mut SessionState,
) -> Result<()> {
    let _ = client.take();
    // Give raw-USB backend a brief moment to fully release the claimed interface.
    tokio::time::sleep(Duration::from_millis(120)).await;
    *client = Some(reconnect(mode, port, baud, state).await?);
    Ok(())
}

fn compact_err(err: &anyhow::Error) -> String {
    let first_line = err.to_string();
    let mut s = first_line
        .lines()
        .next()
        .map(str::to_string)
        .unwrap_or_else(|| "unknown connection error".to_string());
    const CAP: usize = 140;
    if s.chars().count() > CAP {
        s = s.chars().take(CAP).collect::<String>();
        s.push_str("...");
    }
    s
}

fn is_disconnect_error(err: &anyhow::Error) -> bool {
    let msg = err.to_string().to_ascii_lowercase();
    msg.contains("closed")
        || msg.contains("disconnected")
        || msg.contains("broken pipe")
        || msg.contains("connection reset")
        || msg.contains("connection aborted")
        || msg.contains("timed out")
}

async fn dispatch_tokens<B: RpcBackend + Sync>(
    backend: &B,
    state: &mut SessionState,
    tokens: &[String],
) -> Result<bool> {
    let cmd = tokens.first().map(String::as_str).unwrap_or_default();
    let args = &tokens[1..];

    match cmd {
        "help" => print_help(),
        "pwd" => println!("{}", state.cwd),
        "ls" => {
            let (sizes, path) = parse_ls_args(args)?;
            let entries = fs_cmd::list_dir_lines(backend, &state.cwd, path.as_deref(), sizes).await?;
            if sizes {
                for entry in entries {
                    println!("{entry}");
                }
            } else {
                print_ls_entries(&entries);
            }
        }
        "du" => {
            let (all, recursive, depth, path) = parse_du_args(args)?;
            let lines = fs_cmd::du_path_lines(
                backend,
                &state.cwd,
                path.as_deref(),
                all,
                recursive,
                depth,
            )
            .await?;
            for line in lines {
                println!("{line}");
            }
        }
        "cd" => {
            let path = args.first().map(String::as_str).unwrap_or("/");
            state.cwd = fs_cmd::normalize_and_require_dir(backend, &state.cwd, path).await?;
        }
        "cat" => {
            let path = require_arg(args, "cat requires a path argument")?;
            let out = fs_cmd::cat_text(backend, &state.cwd, path).await?;
            println!("{out}");
        }
        "pull" | "cp" => {
            let (recursive, remote, local) = parse_pull_args(args)?;
            let result =
                fs_cmd::pull_remote_to_local(backend, &state.cwd, &remote, &local, recursive)
                    .await?;
            fs_cmd::print_pull_result(&result);
        }
        "rm" => {
            let (recursive, path) = parse_rm_args(args)?;
            let result = fs_cmd::remove_remote_path(backend, &state.cwd, &path, recursive).await?;
            fs_cmd::print_remove_result(&result);
        }
        "erasestorage" | "erase-storage" => {
            warn_erase_storage();
            require_erase_storage_confirmation(args)?;
            let _ = crate::ops::erase_storage(backend).await?;
            println!("external flash erase complete; board will now restart");
        }
        "reset" => {
            if !args.is_empty() {
                bail!("usage: reset");
            }
            crate::ops::reset_board(backend).await?;
            println!("reset acknowledged; board will now restart");
        }
        "panic" => {
            let message = if args.is_empty() {
                "panic requested via shell".to_string()
            } else {
                args.join(" ")
            };
            crate::ops::panic_board(backend, &message).await?;
            println!("panic acknowledged; board will now panic and restart: {message}");
        }
        "info" => {
            let report = crate::ops::build_storage_report(backend).await?;
            println!("{}", crate::ops::format_storage_report(&report));
        }
        "exit" => {
            return Ok(false);
        }
        _ => {
            bail!("unknown command: {cmd}");
        }
    }

    Ok(true)
}

fn print_help() {
    println!("commands:");
    println!("  help");
    println!("  pwd");
    println!("  ls [-s|--sizes] [path]  (example: ls -s /)");
    println!("  du [-a|--all] [-r|--recursive] [-d|--depth N] [path]");
    println!("                           example: du /, du -d 1 /, du -a -d 3 /");
    println!("  cd [path]");
    println!("  cat <path>              (example: cat /logs_12/build_info.txt)");
    println!("  pull [-r] <remote_path> <local_path>");
    println!("                           target path comes first");
    println!("                           file: pull /logs_12/defmt.bin ~/Downloads");
    println!("                           dir : pull -r /logs_12 ~/Downloads");
    println!("  cp [-r] <remote_path> <local_path> (alias)");
    println!("  rm [-r] <path>          (example: rm /logs_12/build_info.txt)");
    println!("                           dir : rm -r /logs_12");
    println!("  erase-storage --yes     (erase entire external flash + reboot)");
    println!("  reset                   (reboot board)");
    println!("  panic [message]         (intentional panic + reboot, max 64 chars)");
    println!("  info");
    println!("  exit");
}

fn require_arg<'a>(args: &'a [String], msg: &str) -> Result<&'a str> {
    args.first()
        .map(String::as_str)
        .ok_or_else(|| anyhow::anyhow!("{}", msg))
}

fn parse_pull_args(args: &[String]) -> Result<(bool, String, String)> {
    let mut recursive = false;
    let mut positional: Vec<String> = Vec::new();

    for arg in args {
        match arg.as_str() {
            "-r" | "-R" | "--recursive" => recursive = true,
            other => positional.push(other.to_string()),
        }
    }

    if positional.len() != 2 {
        bail!("usage: pull [-r] <remote_path> <local_path>");
    }

    Ok((recursive, positional[0].clone(), positional[1].clone()))
}

fn parse_rm_args(args: &[String]) -> Result<(bool, String)> {
    let mut recursive = false;
    let mut positional: Vec<String> = Vec::new();

    for arg in args {
        match arg.as_str() {
            "-r" | "-R" | "--recursive" => recursive = true,
            other => positional.push(other.to_string()),
        }
    }

    if positional.len() != 1 {
        bail!("usage: rm [-r] <path>");
    }

    Ok((recursive, positional[0].clone()))
}

fn parse_ls_args(args: &[String]) -> Result<(bool, Option<String>)> {
    let mut sizes = false;
    let mut positional: Vec<String> = Vec::new();

    for arg in args {
        match arg.as_str() {
            "-s" | "--sizes" => sizes = true,
            other => positional.push(other.to_string()),
        }
    }

    if positional.len() > 1 {
        bail!("usage: ls [-s|--sizes] [path]");
    }

    Ok((sizes, positional.into_iter().next()))
}

fn parse_du_args(args: &[String]) -> Result<(bool, bool, Option<usize>, Option<String>)> {
    let mut all = false;
    let mut recursive = false;
    let mut depth: Option<usize> = None;
    let mut positional: Vec<String> = Vec::new();
    let mut i = 0usize;

    while i < args.len() {
        match args[i].as_str() {
            "-a" | "--all" => {
                all = true;
                i += 1;
            }
            "-r" | "--recursive" => {
                recursive = true;
                i += 1;
            }
            "-d" | "--depth" => {
                let Some(next) = args.get(i + 1) else {
                    bail!("usage: du [-a|--all] [-r|--recursive] [-d|--depth N] [path]");
                };
                let parsed = next
                    .parse::<usize>()
                    .map_err(|_| anyhow::anyhow!("du depth must be a non-negative integer"))?;
                depth = Some(parsed);
                i += 2;
            }
            long if long.starts_with("--depth=") => {
                let val = long.trim_start_matches("--depth=");
                let parsed = val
                    .parse::<usize>()
                    .map_err(|_| anyhow::anyhow!("du depth must be a non-negative integer"))?;
                depth = Some(parsed);
                i += 1;
            }
            other => {
                positional.push(other.to_string());
                i += 1;
            }
        }
    }

    if positional.len() > 1 {
        bail!("usage: du [-a|--all] [-r|--recursive] [-d|--depth N] [path]");
    }

    Ok((all, recursive, depth, positional.into_iter().next()))
}

fn require_erase_storage_confirmation(args: &[String]) -> Result<()> {
    if args.len() == 1 && args[0] == "--yes" {
        return Ok(());
    }
    bail!("confirmation required: erase-storage --yes")
}

fn warn_erase_storage() {
    if use_color_for_warning() {
        eprintln!(
            "\x1b[1;31mwarning:\x1b[0m \x1b[31mthis erases the entire external flash and reboots the board\x1b[0m"
        );
    } else {
        eprintln!("warning: this erases the entire external flash and reboots the board");
    }
}

fn use_color_for_warning() -> bool {
    let term_ok = std::env::var("TERM").map(|v| v != "dumb").unwrap_or(true);
    io::stderr().is_terminal() && std::env::var_os("NO_COLOR").is_none() && term_ok
}

fn mode_label(mode: TransportMode) -> &'static str {
    match mode {
        TransportMode::Auto => "auto",
        TransportMode::Raw => "raw",
        TransportMode::Serial => "serial",
    }
}

fn history_path() -> Option<PathBuf> {
    std::env::var_os("HOME")
        .map(PathBuf::from)
        .map(|home| home.join(".asteria-tool-history"))
}

fn print_ls_entries(entries: &[String]) {
    if entries.is_empty() {
        return;
    }

    if !io::stdout().is_terminal() {
        for entry in entries {
            println!("{entry}");
        }
        return;
    }

    // Keep the table compact on very wide terminals instead of stretching end-to-end.
    let term_width = detect_terminal_width()
        .unwrap_or(80)
        .saturating_mul(3)
        .saturating_div(4)
        .max(20);
    let cell_width = entries
        .iter()
        .map(|s| s.chars().count())
        .max()
        .unwrap_or(1)
        .saturating_add(2);
    let cols = (term_width / cell_width).max(1);

    for row in entries.chunks(cols) {
        let mut line = String::new();
        for (i, entry) in row.iter().enumerate() {
            if i == row.len() - 1 {
                line.push_str(entry);
            } else {
                line.push_str(entry);
                let pad = cell_width.saturating_sub(entry.chars().count());
                line.extend(std::iter::repeat_n(' ', pad));
            }
        }
        println!("{line}");
    }
}

fn detect_terminal_width() -> Option<usize> {
    terminal_size()
        .map(|(Width(w), _)| usize::from(w))
        .or_else(|| {
            std::env::var("COLUMNS")
                .ok()
                .and_then(|v| v.parse::<usize>().ok())
                .filter(|w| *w > 0)
        })
}

fn command_label(tokens: &[String]) -> &str {
    tokens.first().map(String::as_str).unwrap_or("<empty>")
}

fn command_reboots_board(tokens: &[String]) -> bool {
    matches!(
        tokens.first().map(String::as_str),
        Some("erasestorage" | "erase-storage" | "reset" | "panic")
    )
}
