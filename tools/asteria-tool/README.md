# asteria-tool

Host CLI for interacting with Asteria boards over USB RPC.

## Features

- Interactive shell with filesystem UX (`ls`, `cd`, `cat`, `pull`, `rm`)
- Scriptable non-interactive `fs` commands
- Auto-reconnect in shell mode after board restart/disconnect

## Install/Run

From repo root:

```bash
cargo run --manifest-path tools/asteria-tool/Cargo.toml -- <COMMAND>
```

Or from `tools/asteria-tool`:

```bash
cargo run -- <COMMAND>
```

To use the `asteria-tool` commands below, install it from the repository root:

```bash
cargo install --path tools/asteria-tool --locked
```

Optional:

- Add `--stats` to print command timings.

## Connection Modes

- `--connect auto` (default): tries raw USB first; if raw USB is unavailable and `--port` is provided, falls back to serial.
- `--connect raw`: raw USB only.
- `--connect serial --port <PATH>`: serial only.

Examples:

```bash
asteria-tool shell
asteria-tool shell --stats
asteria-tool shell --connect raw
asteria-tool shell --connect serial --port /dev/ttyACM0 --baud 115200
```

## Filesystem Model

The tool talks directly to the board LittleFS via stateless RPC:

- `fs/info`
- `fs/list_dir`
- `fs/stat`
- `fs/read`
- `fs/remove`
- `fs/erase_storage`

Path rules:

- Absolute and relative paths are supported.
- `.` and `..` are supported.
- `cat` on binary files prints a warning; use `pull` instead.

## Interactive Shell

Start shell:

```bash
asteria-tool shell
```

Prompt format:

```text
asteria:<cwd> [<transport>]>
```

Supported commands:

- `help`
- `pwd`
- `ls [path]`
- `cd [path]`
- `cat <path>`
- `pull [-r] <remote_path> <local_path>`
- `cp [-r] <remote_path> <local_path>` (alias of `pull`)
- `rm [-r] <path>`
- `erase-storage --yes`
- `info`
- `exit`

Important:

- `pull/cp` are download-only (board -> host).
- Recursive `pull -r` is best-effort: successful files are kept even if some files fail.
- `rm` deletes remotely; `rm -r` is required for directories.
- The board rejects deleting `/` and may reject active/in-use log paths.
- `erase-storage --yes` erases the entire external flash and reboots the board.
- Argument order is always: `<remote_path> <local_path>`.
- Example mistake: `pull ~/Downloads defmt.bin` is reversed and will fail.
- `~` is expanded for local destination paths.
- If local target is an existing directory and remote is a file, filename is appended automatically.
- If shell detects disconnect, it waits and reconnects automatically.
- On connect/reconnect, shell prints board USB identity (manufacturer/product/serial) when available.
- `info` prints runtime + filesystem status (uptime, fs health, current log dir, caps).

Examples:

```bash
# file pull
pull /logs_25/defmt.bin ~/Downloads

# recursive directory pull
pull -r /logs_25 ~/Downloads

# remove one file
rm /logs_25/build_info.txt

# recursive directory remove
rm -r /logs_25

# erase external flash and reboot
erase-storage --yes

```

## Non-interactive FS Commands

```bash
asteria-tool fs ls [PATH]
asteria-tool fs cd PATH
asteria-tool fs pwd
asteria-tool fs cat PATH
asteria-tool fs pull [-r] REMOTE_PATH LOCAL_PATH
asteria-tool fs rm [-r] PATH
asteria-tool fs erase-storage --yes
asteria-tool fs info
```

Examples:

```bash
asteria-tool fs ls /
asteria-tool fs ls /logs_25
asteria-tool fs cat /logs_25/build_info.txt
asteria-tool fs pull /logs_25/defmt.bin ~/Downloads/defmt_25.bin
asteria-tool fs pull -r /logs_25 ~/Downloads
asteria-tool fs rm /logs_25/build_info.txt
asteria-tool fs rm -r /logs_25
asteria-tool fs erase-storage --yes
asteria-tool fs info
```
