# Build and run asteria-tool in interactive shell mode.
connect *args:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    if pids="$(pgrep -x asteria-tool 2>/dev/null || true)" && [[ -n "$pids" ]]; then
        echo "Warning: asteria-tool is already running (PID(s): $pids) — close open connections first." >&2
    fi
    cargo run --manifest-path "$repo_root/tools/Cargo.toml" --bin asteria-tool --release -- shell {{args}}

# Build asteria-tool and run a non-interactive CLI command (script-friendly).
# Usage: just cli fs ls /
#        just cli info
#        just cli reset
cli *args:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    if pids="$(pgrep -x asteria-tool 2>/dev/null || true)" && [[ -n "$pids" ]]; then
        echo "Warning: asteria-tool is already running (PID(s): $pids) — close open connections first." >&2
    fi
    cargo run --manifest-path "$repo_root/tools/Cargo.toml" --bin asteria-tool --release -- {{args}}

# Fetch log(s) from the connected device and decode with defmt-print.
# Selector: (empty) = all | latest | N = specific log index
# Flags: --quiet / -q  suppress decoded output (only saves to decoded.txt)
# Optional output dir arg overrides ASTERIA_LOG_FETCH_DIR
# Usage: just fetch-logs
#        just fetch-logs all
#        just fetch-logs latest
#        just fetch-logs 3
#        just fetch-logs all ./my-logs
#        just fetch-logs --quiet
#        just fetch-logs latest -q
fetch-logs *args:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    exec "$repo_root/just/scripts/fetch-logs.sh" fetch {{args}}

# Decode a previously-fetched log directory (re-runs ELF lookup + defmt-print).
# Usage: just decode-logs .fetched-logs/2026-02-18_01-03/log_0
decode-logs log_dir:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    exec "$repo_root/just/scripts/fetch-logs.sh" decode "{{log_dir}}"
