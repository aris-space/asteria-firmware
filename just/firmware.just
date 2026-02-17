target := "thumbv7em-none-eabihf"
import './artifacts.just'

# The following variables must be set by the caller of this justfile
# `chip`: The STM32 chip being targeted, e.g. `STM32G473RCTx` or `STM32H723ZG`.
# `bin` : The name of the binary being built, e.g. `fw-engine-board` or `fw-sensor-carrier`.
# `use_probe_rs` : Whether to use probe-rs for flashing (true/false).

fmt *args:
    cargo fmt {{args}}

# Alias for cargo build to transparently pass args
build *args:
    cargo build \
        --target {{target}} \
        {{args}}

# Build the firmware, upload its ELF artifact to B2, and flash it to the device.
# Attaches to the target after flashing either with probe-rs (if use_probe_rs=true)
# or with the STM32 Programmer utility otherwise.
run *args:
    #!/usr/bin/env bash
    set -euo pipefail

    if [[ " {{args}} " == *" --message-format "* ]] || [[ " {{args}} " == *" --message-format="* ]]; then
      echo "Do not pass --message-format; recipe needs JSON output." >&2
      exit 2
    fi

    args_str=" {{args}} "

    # Determine requested profile from args (best-effort).
    requested_profile="dev (default)"
    if [[ "$args_str" == *" --release "* ]]; then
      requested_profile="release"
    fi
    if [[ "$args_str" =~ (^|[[:space:]])--profile[[:space:]]+([^[:space:]]+) ]]; then
      p="${BASH_REMATCH[2]}"
      # If both --release and --profile are present, this is ambiguous; treat as error.
      if [[ "$requested_profile" == "release" ]]; then
        echo "Ambiguous: both --release and --profile $p were provided." >&2
        exit 2
      fi
      requested_profile="$p"
    fi

    tmp="$(mktemp)"
    cargo build --manifest-path app/Cargo.toml \
        --bin "{{bin}}" \
        --target "{{target}}" \
        --message-format=json-render-diagnostics \
        {{args}} \
      | tee "$tmp" >/dev/null

    elf="$(
      jq -r '
        select(.reason=="compiler-artifact")
        | select(.target.name=="{{bin}}")
        | (.executable // empty)
      ' "$tmp" | tail -n1
    )"
    rm -f "$tmp"

    if [[ -z "$elf" || ! -f "$elf" ]]; then
      echo "Could not determine built ELF path from cargo JSON output." >&2
      exit 1
    fi

    # Derive the output dir segment (debug/release/<custom>) from the resolved ELF path.
    chosen_dir="$(basename "$(dirname "$elf")")"

    echo "Built with profile: $requested_profile => $chosen_dir"
    echo "Using ELF: $elf"

    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    upload_script="$repo_root/just/scripts/upload-elf.sh"
    if [[ ! -x "$upload_script" ]]; then
      echo "Missing executable upload script: $upload_script" >&2
      exit 1
    fi

    upload_log="$(mktemp)"
    cleanup_upload() {
      rm -f "$upload_log"
    }
    trap cleanup_upload EXIT

    echo "Starting ELF upload in background."
    (
      "$upload_script" "$elf"
    ) >"$upload_log" 2>&1 &
    upload_pid="$!"

    wait_for_upload() {
      if wait "$upload_pid"; then
        cat "$upload_log"
      else
        echo "ELF upload failed. s5cmd output:" >&2
        cat "$upload_log" >&2
        exit 1
      fi
    }

    if [[ "{{use_probe_rs}}" == "true" ]]; then
        probe-rs run --chip {{chip}} \
            --log-format '{t} {L:severity} {s}' "$elf"
        wait_for_upload
    else
        arm-none-eabi-objcopy -O binary "$elf" "${elf}.bin"
        STM32_Programmer_CLI -c port=SWD -d "${elf}.bin" 0x08000000 -rst
        probe-rs attach --chip {{chip}} "$elf"
        # Mirror probe-rs behavior: defer upload wait until the interactive phase ends.
        wait_for_upload
    fi


# Attach to the target with probe-rs, using the most recently built ELF as the specified binary.
attach:
    #!/usr/bin/env bash
    set -euo pipefail

    elf="$(find "target/{{target}}" -type f -name "{{bin}}" -print0 | xargs -0 ls -t 2>/dev/null | head -n1 || true)"
    [[ -n "$elf" ]] || { echo "No built {{bin}} found under target/{{target}}." >&2; exit 1; }

    chosen_dir="$(basename "$(dirname "$elf")")"
    echo "using profile: $chosen_dir"
    echo "attached with: $elf"

    exec probe-rs attach --chip {{chip}} "$elf"

# Run checks that will be used in CI
ci-checks:
    cargo clippy \
        --target {{target}} \
        -- -D warnings

# Run clippy with optional extra args
clippy *args:
    cargo clippy \
        --target {{target}} \
        {{args}}

# Embedded firmware does not run host tests; keep as a no-op for CI orchestration.
test *args:
    @echo "skipping tests (embedded firmware)"

# Firmware docs are not generated in CI; keep as a no-op target.
doc *args:
    @echo "skipping docs (embedded firmware)"

# Clean rust build artifacts
clean:
    cargo clean

# Upload the most recently built ELF for this board.
upload-latest:
    #!/usr/bin/env bash
    set -euo pipefail

    just upload-latest-by "{{target}}" "{{bin}}"
