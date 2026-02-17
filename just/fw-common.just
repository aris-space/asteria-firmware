target := "thumbv7em-none-eabihf"

# The following variables must be set by the caller of this justfile
# `chip`: The STM32 chip being targeted, e.g. `STM32G473RCTx` or `STM32H723ZG`.
# `bin` : The name of the binary being built, e.g. `fw-engine-board` or `fw-sensor-carrier`.
# `flasher` : The tool used to flash the board. Either `stm32cube` or `probe-rs`.

fmt:
    cargo fmt --manifest-path app/Cargo.toml --all

build *args:
    cargo build \
        --manifest-path app/Cargo.toml \
        --target {{target}} \
        {{args}}

run *args:
    #!/usr/bin/env bash
    set -euo pipefail

    if [[ " {{args}} " == *" --message-format "* ]]; then
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

    if [[ "{{flasher}}" == "probe-rs" ]]; then
        probe-rs run --chip {{chip}} \
            --log-format '{t} {L:severity} {s}' "$elf"
    else
        arm-none-eabi-objcopy -O binary "$elf" "${elf}.bin"
        STM32_Programmer_CLI -c port=SWD -d "${elf}.bin" 0x08000000 -rst
        probe-rs attach --chip {{chip}} "$elf"
    fi


attach:
    #!/usr/bin/env bash
    set -euo pipefail

    elf="$(find "target/{{target}}" -type f -name "{{bin}}" -print0 | xargs -0 ls -t 2>/dev/null | head -n1 || true)"
    [[ -n "$elf" ]] || { echo "No built {{bin}} found under target/{{target}}." >&2; exit 1; }

    chosen_dir="$(basename "$(dirname "$elf")")"
    echo "using profile: $chosen_dir"
    echo "attached with: $elf"

    exec probe-rs attach --chip {{chip}} "$elf"



ci-checks:
    cargo clippy \
        --manifest-path app/Cargo.toml \
        --target {{target}} \
        -- -D warnings
