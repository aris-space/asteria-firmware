use std::fs;
use std::io::{self, IsTerminal, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::{Result, bail};
use clap::Subcommand;

use crate::ops::{self, DirEntry, RpcBackend};
use crate::vfs;
use wire_types::FsNodeKind;

#[derive(Debug, Clone, Subcommand)]
pub enum FsAction {
    /// List entries at a remote path.
    Ls {
        /// Show entry sizes.
        #[arg(short = 's', long)]
        sizes: bool,
        /// Path to list (defaults to current dir in shell, or "/" in non-interactive mode)
        path: Option<String>,
    },
    /// Show disk usage for a path.
    Du {
        /// Include file entries, not only directory totals.
        #[arg(short = 'a', long)]
        all: bool,
        /// Show all nested directories recursively.
        #[arg(short = 'r', long)]
        recursive: bool,
        /// Show directory usage up to depth N.
        #[arg(short = 'd', long)]
        depth: Option<usize>,
        /// Path to inspect (defaults to "/" in non-interactive mode)
        path: Option<String>,
    },
    /// Normalize and print a path (script-friendly)
    Cd { path: String },
    /// Print current directory (always "/" in non-interactive mode)
    Pwd,
    /// Read a remote file.
    Cat { path: String },
    /// Copy a remote file or directory to local disk (remote path first, local path second).
    Pull {
        /// Recursively copy directories.
        #[arg(short = 'r', long)]
        recursive: bool,
        /// Remote filesystem path.
        remote_path: String,
        /// Local host path destination.
        local_path: PathBuf,
    },
    /// Remove a remote file or directory.
    Rm {
        /// Recursively remove directories.
        #[arg(short = 'r', long)]
        recursive: bool,
        /// Remote filesystem path.
        path: String,
    },
    /// Erase external flash storage on device and reboot.
    #[command(name = "erase-storage")]
    EraseStorage {
        /// Confirm destructive erase (required).
        #[arg(long)]
        yes: bool,
    },
    /// Show runtime + filesystem status info.
    Info,
}

pub async fn run_fs_action<B: RpcBackend + Sync>(
    backend: &B,
    action: FsAction,
    stats: bool,
) -> Result<()> {
    let cwd = "/";
    let label = action_label(&action);
    let started = Instant::now();

    match action {
        FsAction::Ls { sizes, path } => {
            let lines = list_dir_lines(backend, cwd, path.as_deref(), sizes).await?;
            for line in lines {
                println!("{line}");
            }
        }
        FsAction::Du {
            all,
            recursive,
            depth,
            path,
        } => {
            let lines = du_path_lines(backend, cwd, path.as_deref(), all, recursive, depth).await?;
            for line in lines {
                println!("{line}");
            }
        }
        FsAction::Cd { path } => {
            let normalized = normalize_and_require_dir(backend, cwd, &path).await?;
            println!("{normalized}");
        }
        FsAction::Pwd => {
            println!("/");
        }
        FsAction::Cat { path } => {
            let out = cat_text(backend, cwd, &path).await?;
            println!("{out}");
        }
        FsAction::Pull {
            recursive,
            remote_path,
            local_path,
        } => {
            let result = pull_remote_to_local(
                backend,
                cwd,
                &remote_path,
                local_path.to_string_lossy().as_ref(),
                recursive,
            )
            .await?;
            print_pull_result(&result);
        }
        FsAction::Info => {
            let report = ops::build_storage_report(backend).await?;
            println!("{}", ops::format_storage_report(&report));
        }
        FsAction::Rm { recursive, path } => {
            let result = remove_remote_path(backend, cwd, &path, recursive).await?;
            print_remove_result(&result);
        }
        FsAction::EraseStorage { yes } => {
            if !yes {
                bail!("confirmation required: fs erase-storage --yes");
            }
            warn_erase_storage();
            let _ = ops::erase_storage(backend).await?;
            println!("external flash erase complete; board will now restart");
        }
    }

    if stats {
        print_stats_line(&format!(
            "{label} took {:.3}s",
            started.elapsed().as_secs_f64()
        ));
    }
    Ok(())
}

fn action_label(action: &FsAction) -> String {
    match action {
        FsAction::Ls { .. } => "fs ls".to_string(),
        FsAction::Du { .. } => "fs du".to_string(),
        FsAction::Cd { .. } => "fs cd".to_string(),
        FsAction::Pwd => "fs pwd".to_string(),
        FsAction::Cat { .. } => "fs cat".to_string(),
        FsAction::Pull { .. } => "fs pull".to_string(),
        FsAction::Rm { .. } => "fs rm".to_string(),
        FsAction::EraseStorage { .. } => "fs erase-storage".to_string(),
        FsAction::Info => "fs info".to_string(),
    }
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
    io::stderr().is_terminal() && std::env::var_os("NO_COLOR").is_none() && term_ok
}

fn warn_erase_storage() {
    if use_color_for_stats() {
        eprintln!(
            "\x1b[1;31mwarning:\x1b[0m \x1b[31mthis erases the entire external flash and reboots the board\x1b[0m"
        );
    } else {
        eprintln!("warning: this erases the entire external flash and reboots the board");
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum RemoveKind {
    File,
    Directory,
}

#[derive(Debug, Clone)]
pub struct RemoveResult {
    pub kind: RemoveKind,
    pub remote: String,
    pub paths_removed: u32,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum PullKind {
    File,
    Directory,
}

#[derive(Debug, Clone)]
pub struct PullResult {
    pub kind: PullKind,
    pub remote: String,
    pub local: PathBuf,
    pub files: u32,
    pub bytes: u64,
    pub failed_files: u32,
    pub failures: Vec<String>,
}

pub async fn pull_remote_to_local<B: RpcBackend + Sync>(
    backend: &B,
    cwd: &str,
    remote_input: &str,
    local_input: &str,
    recursive: bool,
) -> Result<PullResult> {
    let remote = vfs::normalize_path(cwd, remote_input)?;
    let node = ops::stat_path(backend, &remote).await?;

    match node.kind {
        FsNodeKind::File => {
            let local = resolve_local_pull_file_path(local_input, &remote)?;
            let bytes = pull_file_with_progress(backend, &remote, &local).await?;
            Ok(PullResult {
                kind: PullKind::File,
                remote,
                local,
                files: 1,
                bytes: bytes as u64,
                failed_files: 0,
                failures: Vec::new(),
            })
        }
        FsNodeKind::Dir => {
            if !recursive {
                bail!("omitting directory '{remote}'; use -r/--recursive");
            }
            let local_root = resolve_local_pull_dir_root(local_input, &remote)?;
            let (files, bytes, failures) =
                pull_directory_tree(backend, &remote, &local_root).await?;
            let failed_files = failures.len() as u32;
            Ok(PullResult {
                kind: PullKind::Directory,
                remote,
                local: local_root,
                files,
                bytes,
                failed_files,
                failures,
            })
        }
    }
}

pub fn print_pull_result(result: &PullResult) {
    match result.kind {
        PullKind::File => {
            println!(
                "pulled {} bytes from {} to {}",
                result.bytes,
                result.remote,
                result.local.display()
            );
        }
        PullKind::Directory => {
            println!(
                "pulled {} file(s), {} bytes from {} to {}",
                result.files,
                result.bytes,
                result.remote,
                result.local.display()
            );
            if result.failed_files > 0 {
                eprintln!(
                    "warning: {} file(s) failed during pull (partial copy kept)",
                    result.failed_files
                );
                for failure in result.failures.iter().take(20) {
                    eprintln!("  - {failure}");
                }
                if result.failures.len() > 20 {
                    eprintln!("  - ... and {} more", result.failures.len() - 20);
                }
            }
        }
    }
}

pub fn print_remove_result(result: &RemoveResult) {
    match result.kind {
        RemoveKind::File => {
            println!("removed {}", result.remote);
        }
        RemoveKind::Directory => {
            println!(
                "removed {} path(s) under {}",
                result.paths_removed, result.remote
            );
        }
    }
}

async fn pull_directory_tree<B: RpcBackend + Sync>(
    backend: &B,
    remote_root: &str,
    local_root: &Path,
) -> Result<(u32, u64, Vec<String>)> {
    let mut stack: Vec<(String, PathBuf)> =
        vec![(remote_root.to_string(), local_root.to_path_buf())];
    let mut files = 0u32;
    let mut bytes = 0u64;
    let mut failures: Vec<String> = Vec::new();

    while let Some((remote_dir, local_dir)) = stack.pop() {
        if let Err(e) = fs::create_dir_all(&local_dir) {
            push_pull_failure(
                &mut failures,
                format!(
                    "{remote_dir}: cannot create local directory {} ({e})",
                    local_dir.display()
                ),
            );
            continue;
        }

        let entries = match ops::list_dir_entries(backend, &remote_dir).await {
            Ok(v) => v,
            Err(e) => {
                push_pull_failure(&mut failures, format!("{remote_dir}: list failed ({e})"));
                continue;
            }
        };

        for DirEntry { kind, name } in entries {
            let child_remote = vfs::join(&remote_dir, &name)?;
            let child_local = local_dir.join(&name);
            match kind {
                FsNodeKind::Dir => stack.push((child_remote, child_local)),
                FsNodeKind::File => {
                    match pull_file_with_progress(backend, &child_remote, &child_local).await {
                        Ok(n) => {
                            files = files.saturating_add(1);
                            bytes = bytes.saturating_add(n as u64);
                        }
                        Err(e) => {
                            push_pull_failure(
                                &mut failures,
                                format!("{child_remote}: pull failed ({e})"),
                            );
                        }
                    }
                }
            }
        }
    }

    Ok((files, bytes, failures))
}

fn fmt_bytes(bytes: usize) -> String {
    if bytes >= 1_048_576 {
        format!("{:.1} MB", bytes as f64 / 1_048_576.0)
    } else if bytes >= 1_024 {
        format!("{:.1} KB", bytes as f64 / 1_024.0)
    } else {
        format!("{bytes} B")
    }
}

async fn pull_file_with_progress<B: RpcBackend + Sync>(
    backend: &B,
    remote_path: &str,
    local_path: &Path,
) -> Result<usize> {
    let is_tty = io::stderr().is_terminal();
    let mut last_percent = 0u8;
    let start = Instant::now();

    if is_tty {
        eprint!("pulling {remote_path} ...");
        let _ = io::stderr().flush();
    } else {
        eprintln!("pulling {remote_path}");
    }

    let result =
        ops::pull_file_to_path_with_progress(backend, remote_path, local_path, |done, total| {
            if !is_tty {
                return;
            }
            let pct = done
                .saturating_mul(100)
                .checked_div(total)
                .unwrap_or(100)
                .min(100) as u8;
            if pct >= last_percent.saturating_add(5) || pct == 100 {
                last_percent = pct;
                let elapsed = start.elapsed().as_secs_f64().max(1e-6);
                let rate = done as f64 / elapsed;
                let rate_str = fmt_bytes(rate as usize);
                eprint!(
                    "\rpulling {remote_path} ... {:>3}%  {} / {}  {}/s  ",
                    pct,
                    fmt_bytes(done),
                    fmt_bytes(total),
                    rate_str,
                );
                let _ = io::stderr().flush();
            }
        })
        .await;

    if is_tty {
        eprintln!();
    }
    result
}

fn push_pull_failure(failures: &mut Vec<String>, msg: String) {
    const MAX_FAILURES_RECORDED: usize = 512;
    if failures.len() < MAX_FAILURES_RECORDED {
        failures.push(msg);
    }
}

pub async fn remove_remote_path<B: RpcBackend + Sync>(
    backend: &B,
    cwd: &str,
    remote_input: &str,
    recursive: bool,
) -> Result<RemoveResult> {
    let remote = vfs::normalize_path(cwd, remote_input)?;
    if remote == "/" {
        bail!("refusing to remove root directory '/'");
    }

    let node = ops::stat_path(backend, &remote).await?;
    match node.kind {
        FsNodeKind::File => {
            let _ = ops::remove_path(backend, &remote).await?;
            Ok(RemoveResult {
                kind: RemoveKind::File,
                remote,
                paths_removed: 1,
            })
        }
        FsNodeKind::Dir => {
            if !recursive {
                bail!("cannot remove directory '{remote}' without -r/--recursive");
            }
            let paths_removed = remove_directory_tree(backend, &remote).await?;
            Ok(RemoveResult {
                kind: RemoveKind::Directory,
                remote,
                paths_removed,
            })
        }
    }
}

async fn remove_directory_tree<B: RpcBackend + Sync>(backend: &B, root: &str) -> Result<u32> {
    enum Pending {
        Visit(String),
        RemoveDir(String),
    }

    let mut stack: Vec<Pending> = vec![Pending::Visit(root.to_string())];
    let mut removed = 0u32;

    while let Some(item) = stack.pop() {
        match item {
            Pending::Visit(path) => {
                let node = ops::stat_path(backend, &path).await?;
                match node.kind {
                    FsNodeKind::File => {
                        let _ = ops::remove_path(backend, &path).await?;
                        removed = removed.saturating_add(1);
                    }
                    FsNodeKind::Dir => {
                        stack.push(Pending::RemoveDir(path.clone()));
                        let entries = ops::list_dir_entries(backend, &path).await?;
                        for DirEntry { kind: _, name } in entries.into_iter().rev() {
                            let child = vfs::join(&path, &name)?;
                            stack.push(Pending::Visit(child));
                        }
                    }
                }
            }
            Pending::RemoveDir(path) => {
                let _ = ops::remove_path(backend, &path).await?;
                removed = removed.saturating_add(1);
            }
        }
    }

    Ok(removed)
}

pub fn resolve_local_pull_file_path(raw_local: &str, remote_file_path: &str) -> Result<PathBuf> {
    let mut out = PathBuf::from(expand_tilde(raw_local));
    if out.is_dir() {
        out.push(vfs::file_name(remote_file_path)?);
    }
    Ok(out)
}

pub fn resolve_local_pull_dir_root(raw_local: &str, remote_dir_path: &str) -> Result<PathBuf> {
    let out = PathBuf::from(expand_tilde(raw_local));
    if out.exists() {
        if out.is_file() {
            bail!("cannot copy directory into file: {}", out.display());
        }
        if remote_dir_path == "/" {
            return Ok(out);
        }
        return Ok(out.join(vfs::file_name(remote_dir_path)?));
    }
    Ok(out)
}

fn expand_tilde(input: &str) -> String {
    if input == "~" {
        return home_dir().unwrap_or_else(|| input.to_string());
    }
    if let Some(rest) = input.strip_prefix("~/")
        && let Some(home) = home_dir()
    {
        let mut p = PathBuf::from(home);
        p.push(rest);
        return p.to_string_lossy().into_owned();
    }
    if let Some(rest) = input.strip_prefix("~\\")
        && let Some(home) = home_dir()
    {
        let mut p = PathBuf::from(home);
        p.push(rest);
        return p.to_string_lossy().into_owned();
    }
    input.to_string()
}

fn home_dir() -> Option<String> {
    std::env::var("HOME")
        .ok()
        .or_else(|| std::env::var("USERPROFILE").ok())
}

pub async fn list_dir_lines<B: RpcBackend + Sync>(
    backend: &B,
    cwd: &str,
    path: Option<&str>,
    show_sizes: bool,
) -> Result<Vec<String>> {
    let normalized = match path {
        Some(p) => vfs::normalize_path(cwd, p)?,
        None => cwd.to_string(),
    };

    let node = ops::stat_path(backend, &normalized).await?;
    if node.kind == FsNodeKind::File {
        let name = vfs::file_name(&normalized)?.to_string();
        if show_sizes {
            return Ok(vec![format!(
                "{:>10} {name}",
                format_size_bytes_u64(node.size_bytes as u64)
            )]);
        }
        return Ok(vec![name]);
    }

    let mut out = Vec::new();
    for DirEntry { kind, name } in ops::list_dir_entries(backend, &normalized).await? {
        if kind == FsNodeKind::Dir {
            if show_sizes {
                out.push(format!("{:>10} {name}/", "-"));
            } else {
                out.push(format!("{name}/"));
            }
        } else {
            if show_sizes {
                let child = vfs::join(&normalized, &name)?;
                let size = ops::stat_path(backend, &child).await?.size_bytes;
                out.push(format!("{:>10} {name}", format_size_bytes_u64(size as u64)));
            } else {
                out.push(name);
            }
        }
    }

    Ok(out)
}

fn format_size_bytes_u64(bytes: u64) -> String {
    let mut value = bytes as f64;
    let units = ["B", "KB", "MB", "GB"];
    let mut idx = 0usize;
    while value >= 1024.0 && idx + 1 < units.len() {
        value /= 1024.0;
        idx += 1;
    }
    if idx == 0 {
        format!("{bytes}B")
    } else {
        format!("{value:.1}{}", units[idx])
    }
}

async fn dir_total_bytes<B: RpcBackend + Sync>(backend: &B, root: &str) -> Result<u64> {
    let mut total = 0u64;
    let mut stack: Vec<String> = vec![root.to_string()];

    while let Some(dir) = stack.pop() {
        for DirEntry { kind, name } in ops::list_dir_entries(backend, &dir).await? {
            let child = vfs::join(&dir, &name)?;
            match kind {
                FsNodeKind::File => {
                    let meta = ops::stat_path(backend, &child).await?;
                    total = total.saturating_add(meta.size_bytes as u64);
                }
                FsNodeKind::Dir => stack.push(child),
            }
        }
    }

    Ok(total)
}

pub async fn du_path_lines<B: RpcBackend + Sync>(
    backend: &B,
    cwd: &str,
    path: Option<&str>,
    include_files: bool,
    recursive: bool,
    depth: Option<usize>,
) -> Result<Vec<String>> {
    let normalized = match path {
        Some(p) => vfs::normalize_path(cwd, p)?,
        None => cwd.to_string(),
    };

    let node = ops::stat_path(backend, &normalized).await?;
    if node.kind == FsNodeKind::File {
        return Ok(vec![format!(
            "{:>10} {normalized}",
            format_size_bytes_u64(node.size_bytes as u64)
        )]);
    }

    let max_depth = match depth {
        Some(d) => d,
        None if recursive => usize::MAX,
        None => 0,
    };

    let mut out: Vec<String> = Vec::new();
    let mut stack: Vec<(String, usize)> = vec![(normalized, 0)];
    while let Some((dir, d)) = stack.pop() {
        let total = dir_total_bytes(backend, &dir).await?;
        out.push(format!("{:>10} {}", format_size_bytes_u64(total), dir));

        if d >= max_depth {
            continue;
        }

        let mut child_dirs: Vec<String> = Vec::new();
        for DirEntry { kind, name } in ops::list_dir_entries(backend, &dir).await? {
            let child = vfs::join(&dir, &name)?;
            let child_depth = d.saturating_add(1);
            if kind == FsNodeKind::Dir {
                child_dirs.push(child);
            } else if include_files && child_depth <= max_depth {
                let meta = ops::stat_path(backend, &child).await?;
                out.push(format!(
                    "{:>10} {}",
                    format_size_bytes_u64(meta.size_bytes as u64),
                    child
                ));
            }
        }
        child_dirs.sort();
        for child in child_dirs.into_iter().rev() {
            stack.push((child, d + 1));
        }
    }

    Ok(out)
}

pub async fn normalize_and_require_dir<B: RpcBackend + Sync>(
    backend: &B,
    cwd: &str,
    input: &str,
) -> Result<String> {
    let normalized = vfs::normalize_path(cwd, input)?;
    let node = ops::stat_path(backend, &normalized).await?;
    if node.kind != FsNodeKind::Dir {
        bail!("not a directory: {normalized}");
    }
    Ok(normalized)
}

pub async fn cat_text<B: RpcBackend + Sync>(backend: &B, cwd: &str, path: &str) -> Result<String> {
    let normalized = vfs::normalize_path(cwd, path)?;
    let node = ops::stat_path(backend, &normalized).await?;
    if node.kind != FsNodeKind::File {
        bail!("cat expects a file path");
    }

    let bytes = ops::read_file_bytes(backend, &normalized).await?;
    match String::from_utf8(bytes) {
        Ok(s) => Ok(s),
        Err(_) => Ok("binary file content; use `pull <remote_path> <local_path>`".to_string()),
    }
}
