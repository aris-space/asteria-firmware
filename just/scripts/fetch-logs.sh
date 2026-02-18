#!/usr/bin/env bash
set -euo pipefail

# Source shared helpers from upload-elf.sh (die, get_repo_root,
# resolve_local_artifact_dir, load_b2_config).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=upload-elf.sh
source "$SCRIPT_DIR/upload-elf.sh"

# ---------------------------------------------------------------------------
# Connection guard
# ---------------------------------------------------------------------------

# Warn if another asteria-tool process is already running (it holds the USB
# interface, so a new connection will silently fail through retries).
warn_existing_connections() {
  local pids
  pids="$(pgrep -x asteria-tool 2>/dev/null || true)"
  if [[ -n "$pids" ]]; then
    echo "Warning: asteria-tool is already running (PID(s): $pids)." >&2
    echo "         Close any open connections before running this command." >&2
  fi
}

# ---------------------------------------------------------------------------
# Tool build
# ---------------------------------------------------------------------------

build_asteria_tool() {
  local repo_root host
  repo_root="$(get_repo_root)"
  host="$(rustc -vV | sed -n 's/^host: //p')"
  cargo build --manifest-path "$repo_root/tools/Cargo.toml" \
    --target "$host" --bin asteria-tool --release
  printf "%s\n" "$repo_root/tools/target/$host/release/asteria-tool"
}

# ---------------------------------------------------------------------------
# Log directory listing and selection
# ---------------------------------------------------------------------------

# Prints sorted log_N directory names from the device (one per line).
# Returns 1 (propagating asteria-tool's error) if the device call fails;
# returns 0 with empty output if connected but no log dirs exist.
list_log_dirs() {
  local tool="$1"
  local raw
  raw="$("$tool" fs ls /)" || return 1
  printf "%s\n" "$raw" | grep -oE 'log_[0-9]+' | sort -t_ -k2 -n | uniq || true
}

# Reads log dir names from stdin, outputs those matching selector.
# selector: "" | "all" → all; "latest" → highest; digit-string → log_N only.
select_log_dirs() {
  local selector="$1"
  if [[ -z "$selector" || "$selector" == "all" ]]; then
    cat
  elif [[ "$selector" == "latest" ]]; then
    tail -n1
  elif [[ "$selector" =~ ^[0-9]+$ ]]; then
    local target="log_$selector"
    grep -Fx "$target" || { echo "Error: $target not found on device" >&2; exit 1; }
  else
    die "Invalid selector '$selector': use 'all', 'latest', or a log index number."
  fi
}

# ---------------------------------------------------------------------------
# Fetch
# ---------------------------------------------------------------------------

fetch_log_dir() {
  local tool="$1" log_name="$2" parent_dir="$3"
  # asteria-tool creates log_name/ inside parent_dir.
  mkdir -p "$parent_dir"
  "$tool" fs pull -r "/$log_name" "$parent_dir"
}

# ---------------------------------------------------------------------------
# Artifact / ELF resolution
# ---------------------------------------------------------------------------

# Reads artifact_timestamp_ms from build_info.txt in log_dir.
# Prints the timestamp string, or "none" if missing/unavailable.
parse_artifact_ts() {
  local dir="$1"
  local build_info="$dir/build_info.txt"
  if [[ ! -f "$build_info" ]]; then
    printf "none\n"
    return
  fi
  local ts
  ts="$(grep -E '^artifact_timestamp_ms=' "$build_info" \
        | cut -d= -f2 \
        | tr -d '[:space:]' \
        || true)"
  if [[ -z "$ts" || "$ts" == "none" ]]; then
    printf "none\n"
  else
    printf "%s\n" "$ts"
  fi
}

# Checks local .artifacts/<ts>.elf; downloads from B2 if missing.
# Prints the local ELF path on success; returns 1 on failure.
find_or_download_elf() {
  local ts="$1"
  local artifacts_dir
  artifacts_dir="$(resolve_local_artifact_dir)"
  local local_elf="$artifacts_dir/${ts}.elf"

  if [[ -f "$local_elf" ]]; then
    printf "%s\n" "$local_elf"
    return 0
  fi

  # Not in local cache — check if B2 is configured before attempting download.
  local repo_root b2_env_file
  repo_root="$(get_repo_root)"
  b2_env_file="${ASTERIA_B2_ENV_FILE:-$repo_root/.b2.env}"
  if [[ ! -f "$b2_env_file" ]]; then
    return 1
  fi

  if ! command -v s5cmd >/dev/null 2>&1; then
    echo "s5cmd not found; cannot download ELF from B2." >&2
    return 1
  fi

  load_b2_config
  mkdir -p "$artifacts_dir"
  local object_uri="s3://${B2_BUCKET}/${ts}.elf"
  echo "Downloading ELF from B2: $object_uri" >&2
  if AWS_ACCESS_KEY_ID="$B2_KEY_ID" \
       AWS_SECRET_ACCESS_KEY="$B2_APPLICATION_KEY" \
       s5cmd --endpoint-url "$B2_ENDPOINT" cp "$object_uri" "$local_elf" >&2; then
    printf "%s\n" "$local_elf"
    return 0
  else
    rm -f "$local_elf"
    return 1
  fi
}

# ---------------------------------------------------------------------------
# Decode
# ---------------------------------------------------------------------------

decode_log_dir() {
  local elf="$1" log_dir="$2" quiet="${3:-0}"
  local defmt_bin="$log_dir/defmt.bin"
  local decoded_txt="$log_dir/decoded.txt"
  if [[ ! -f "$defmt_bin" ]]; then
    echo "  No defmt.bin in $log_dir — skipping decode." >&2
    return 1
  fi
  if [[ "$quiet" == "1" ]]; then
    CLICOLOR_FORCE=1 defmt-print -e "$elf" stdin <"$defmt_bin" \
      | perl -pe 's/\e\[[0-9;]*m//g' > "$decoded_txt"
  else
    CLICOLOR_FORCE=1 defmt-print -e "$elf" stdin <"$defmt_bin" \
      | tee >(perl -pe 's/\e\[[0-9;]*m//g' > "$decoded_txt")
  fi
}

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

cmd_fetch() {
  local selector="all"
  local out_base=""
  local quiet=0

  # Scan for --quiet / -q; collect remaining args positionally.
  local positional=()
  for arg in "$@"; do
    case "$arg" in
      --quiet|-q) quiet=1 ;;
      *) positional+=("$arg") ;;
    esac
  done
  selector="${positional[0]:-all}"
  out_base="${positional[1]:-}"

  warn_existing_connections

  command -v defmt-print >/dev/null 2>&1 \
    || die "defmt-print not found; install with: cargo install defmt-print"

  local tool
  tool="$(build_asteria_tool)"
  [[ -x "$tool" ]] || die "asteria-tool not found at: $tool"

  local repo_root
  repo_root="$(get_repo_root)"
  local fetch_base="${out_base:-${ASTERIA_LOG_FETCH_DIR:-$repo_root/.fetched-logs}}"
  local timestamp
  timestamp="$(date -u '+%Y-%m-%d_%H-%M')"
  local out_dir="$fetch_base/$timestamp"

  local log_dirs_raw
  if ! log_dirs_raw="$(list_log_dirs "$tool")"; then
    die "Could not list log directories — is the device connected?"
  fi
  if [[ -z "$log_dirs_raw" ]]; then
    die "No log directories found on device."
  fi

  local selected
  selected="$(printf "%s\n" "$log_dirs_raw" | select_log_dirs "$selector")"
  if [[ -z "$selected" ]]; then
    die "No log directories matched selector '$selector'."
  fi

  mkdir -p "$out_dir"
  echo "Fetching logs to: $out_dir"

  while IFS= read -r log_name; do
    echo ""
    echo "=== $log_name ==="
    local log_out="$out_dir/$log_name"

    echo "Fetching $log_name..."
    fetch_log_dir "$tool" "$log_name" "$out_dir"

    local ts
    ts="$(parse_artifact_ts "$log_out")"
    if [[ "$ts" == "none" ]]; then
      echo "  Warning: artifact_timestamp_ms not found in build_info.txt — skipping decode." >&2
      continue
    fi

    local elf
    if ! elf="$(find_or_download_elf "$ts")"; then
      echo "  Warning: ELF for timestamp $ts not available (not in local cache or B2) — skipping decode." >&2
      continue
    fi

    echo "  Decoding with ELF: $elf"
    decode_log_dir "$elf" "$log_out" "$quiet" || true
  done <<<"$selected"

  echo ""
  echo "Done. Logs saved to: $out_dir"
}

cmd_decode() {
  local log_dir="${1:-}"
  [[ -n "$log_dir" ]] || die "Usage: $0 decode <log_dir>"
  [[ -d "$log_dir" ]] || die "Directory not found: $log_dir"

  warn_existing_connections

  command -v defmt-print >/dev/null 2>&1 \
    || die "defmt-print not found; install with: cargo install defmt-print"

  local ts
  ts="$(parse_artifact_ts "$log_dir")"
  if [[ "$ts" == "none" ]]; then
    die "artifact_timestamp_ms not found in $log_dir/build_info.txt"
  fi

  local elf
  if ! elf="$(find_or_download_elf "$ts")"; then
    die "ELF for timestamp $ts not available (not in local cache or B2)."
  fi

  echo "Decoding $log_dir with ELF: $elf"
  decode_log_dir "$elf" "$log_dir"
}

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

main() {
  [[ $# -ge 1 ]] || die "Usage: $0 <fetch|decode> [args...]"
  local cmd="$1"
  shift
  case "$cmd" in
    fetch)  cmd_fetch  "$@" ;;
    decode) cmd_decode "$@" ;;
    *)      die "Unknown command '$cmd'. Use 'fetch' or 'decode'." ;;
  esac
}

main "$@"
