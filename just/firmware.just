set allow-duplicate-variables

target := "thumbv7em-none-eabihf"
import './artifacts.just'
import './logs.just'

# The following variables must be set by the caller of this justfile
# `chip`: The STM32 chip being targeted, e.g. `STM32G473RCTx` or `STM32H723ZG`.
# `bin` : The name of the binary being built, e.g. `fw-engine-board` or `fw-sensor-carrier`.
# `use_probe_rs` : Whether to use probe-rs for flashing (true/false).
#
# Probe-rs flag defaults. A board's justfile can re-assign either of these to override.
probe_rs_run_flags := "--no-catch-hardfault --no-catch-reset --chip-erase --log-format '{t} {L:severity} {s}'"
probe_rs_attach_flags := "--no-catch-hardfault --no-catch-reset"

# Show available capabilities in a board workspace.
[default]
help:
    @just --list

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

    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    build_out="$("$repo_root/just/scripts/build-elf.sh" "{{bin}}" "{{target}}" {{args}})"
    ASTERIA_ARTIFACT_TIMESTAMP_MS="${build_out%%:*}"
    elf="${build_out#*:}"

    # Derive the output dir segment (debug/release/<custom>) from the resolved ELF path.
    chosen_dir="$(basename "$(dirname "$elf")")"

    echo "Built with profile: $requested_profile => $chosen_dir"
    echo "Using ELF: $elf"
    echo "Artifact/build timestamp ms: $ASTERIA_ARTIFACT_TIMESTAMP_MS"

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
      "$upload_script" "$elf" "${ASTERIA_ARTIFACT_TIMESTAMP_MS}.elf"
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
        probe-rs run --chip {{chip}} {{probe_rs_run_flags}} "$elf"
        wait_for_upload
    else
        arm-none-eabi-objcopy -O binary "$elf" "${elf}.bin"
        STM32_Programmer_CLI -c port=SWD -d "${elf}.bin" 0x08000000 -rst
        probe-rs attach --chip {{chip}} {{probe_rs_attach_flags}} "$elf"
        # Mirror probe-rs behavior: defer upload wait until the interactive phase ends.
        wait_for_upload
    fi


# Build the firmware, upload its ELF artifact to B2, and flash it to the device without attaching,
# either with probe-rs (if use_probe_rs=true) or with the STM32 Programmer utility otherwise.
flash *args:
    #!/usr/bin/env bash
    set -euo pipefail

    if [[ " {{args}} " == *" --message-format "* ]] || [[ " {{args}} " == *" --message-format="* ]]; then
      echo "Do not pass --message-format; recipe needs JSON output." >&2
      exit 2
    fi

    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    build_out="$("$repo_root/just/scripts/build-elf.sh" "{{bin}}" "{{target}}" {{args}})"
    ASTERIA_ARTIFACT_TIMESTAMP_MS="${build_out%%:*}"
    elf="${build_out#*:}"

    echo "Using ELF: $elf"
    echo "Artifact/build timestamp ms: $ASTERIA_ARTIFACT_TIMESTAMP_MS"

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
      "$upload_script" "$elf" "${ASTERIA_ARTIFACT_TIMESTAMP_MS}.elf"
    ) >"$upload_log" 2>&1 &
    upload_pid="$!"

    just _flash-elf "$elf"

    if wait "$upload_pid"; then
      cat "$upload_log"
    else
      echo "ELF upload failed. s5cmd output:" >&2
      cat "$upload_log" >&2
      exit 1
    fi

# Flash a built ELF without attaching, shared by flash and flash-tag.
_flash-elf elf:
    #!/usr/bin/env bash
    set -euo pipefail

    if [[ "{{use_probe_rs}}" == "true" ]]; then
        probe-rs download --chip {{chip}} --chip-erase "{{elf}}"
        # download leaves the core halted in its flash loader; start the new firmware.
        probe-rs reset --chip {{chip}}
    else
        arm-none-eabi-objcopy -O binary "{{elf}}" "{{elf}}.bin"
        STM32_Programmer_CLI -c port=SWD -d "{{elf}}.bin" 0x08000000 -rst
    fi

# Usage: just flash-tag portugal --release
# Refuses to run on a dirty tree. Flashing the same commit with the same args again reuses the existing tag.
# Build and flash like `flash`, tag the commit as <event>/<board>/<n>, push it and attach the ELF to a GitHub release.
flash-tag event *args:
    #!/usr/bin/env bash
    set -euo pipefail

    if [[ " {{args}} " == *" --message-format "* ]] || [[ " {{args}} " == *" --message-format="* ]]; then
      echo "Do not pass --message-format; recipe needs JSON output." >&2
      exit 2
    fi

    event="{{event}}"
    if [[ ! "$event" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
      echo "Event name may only contain letters, digits, '.', '_' and '-': $event" >&2
      exit 2
    fi
    board="{{trim_start_match(bin, "fw-")}}"

    repo_root="$(git rev-parse --show-toplevel)"
    dirty="$(git -C "$repo_root" status --porcelain --untracked-files=no --ignore-submodules=none)"
    if [[ -n "$dirty" ]]; then
      echo "Tree is dirty, commit first, then flash-tag:" >&2
      echo "$dirty" >&2
      exit 1
    fi
    commit="$(git rev-parse HEAD)"

    # Pull tags others made for this event so numbering doesn't collide. Fine to fail offline.
    git fetch origin --tags --quiet || echo "Could not fetch tags from origin, numbering from local tags." >&2

    build_out="$("$repo_root/just/scripts/build-elf.sh" "{{bin}}" "{{target}}" {{args}})"
    elf="${build_out#*:}"
    echo "Using ELF: $elf"

    just _flash-elf "$elf"

    args_line="args: {{args}}"
    defmt_line="DEFMT_LOG: ${DEFMT_LOG:-from .cargo/config.toml}"

    # Same commit and same build settings means the same firmware, so keep its tag.
    tag=""
    last=0
    for t in $(git tag -l "$event/$board/*"); do
      n="${t##*/}"
      [[ "$n" =~ ^[0-9]+$ ]] || continue
      (( n > last )) && last="$n"
      msg="$(git tag -l --format='%(contents)' "$t")"
      if [[ "$(git rev-parse "$t^{commit}")" == "$commit" ]] \
        && grep -Fxq "$args_line" <<<"$msg" && grep -Fxq "$defmt_line" <<<"$msg"; then
        tag="$t"
      fi
    done

    if [[ -n "$tag" ]]; then
      echo "Already tagged as $tag, not making a new tag"
      exit 0
    fi

    tag="$event/$board/$((last + 1))"
    git tag -a "$tag" "$commit" -F - <<EOF
    $tag

    board: {{bin}} ({{chip}})
    flashed: $(date -u '+%Y-%m-%d %H:%M UTC')
    $args_line
    $defmt_line

    submodules:
    $(git -C "$repo_root" submodule status)
    EOF
    echo "Tagged $tag at ${commit:0:7}"

    # Keep a copy so the release can still be made later if we're offline now.
    saved_elf="$repo_root/target/flash-tag/$tag/{{bin}}.elf"
    mkdir -p "$(dirname "$saved_elf")"
    cp "$elf" "$saved_elf"

    if ! git push --quiet origin "refs/tags/$tag"; then
      echo "Push failed, the tag only exists locally. Retry with:" >&2
      echo "  git push origin $tag && gh release create $tag $saved_elf --verify-tag --notes-from-tag --latest=false" >&2
      exit 0
    fi
    echo "Pushed $tag to origin"

    if gh release create "$tag" "$saved_elf" --verify-tag --notes-from-tag --latest=false >/dev/null; then
      echo "Attached ELF to release $tag"
    else
      echo "Release failed. Retry with: gh release create $tag $saved_elf --verify-tag --notes-from-tag --latest=false" >&2
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

    exec probe-rs attach --chip {{chip}} {{probe_rs_attach_flags}} "$elf"

# Replay flash logs then attach live via probe-rs.
replay-attach:
    #!/usr/bin/env bash
    set -euo pipefail

    # --- Find ELF (same as attach) ---
    elf="$(find "target/{{target}}" -type f -name "{{bin}}" -print0 \
         | xargs -0 ls -t 2>/dev/null | head -n1 || true)"
    [[ -n "$elf" ]] || { echo "No built {{bin}} found under target/{{target}}." >&2; exit 1; }
    echo "using profile: $(basename "$(dirname "$elf")")"
    echo "elf: $elf"

    # --- Build asteria-tool ---
    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    host="$(rustc -vV | sed -n 's/^host: //p')"
    cargo build --manifest-path "$repo_root/tools/Cargo.toml" \
      --target "$host" --bin asteria-tool --release 2>&1
    tool="$repo_root/tools/target/$host/release/asteria-tool"

    # --- Fetch latest flash log to temp dir ---
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' EXIT

    if raw="$("$tool" fs ls /)"; then
      latest="$(printf "%s\n" "$raw" | grep -oE 'log_[0-9]+' | sort -t_ -k2 -n | tail -n1)"
      if [[ -n "$latest" ]]; then
        echo "Fetching $latest from flash..."
        "$tool" fs pull -r "/$latest" "$tmp"
        defmt_bin="$tmp/$latest/defmt.bin"
        if [[ -f "$defmt_bin" ]]; then
          echo ""
          echo "=== flash log: $latest ==="
          defmt-print -e "$elf" stdin < "$defmt_bin" || true
        else
          echo "No defmt.bin in $latest — skipping replay."
        fi
      else
        echo "No flash logs found — skipping replay."
      fi
    else
      echo "Could not list flash logs (USB not available?) — skipping replay." >&2
    fi

    # --- Transition to live ---
    echo ""
    echo "--- live (probe-rs attach) ---"
    echo ""
    exec probe-rs attach --chip {{chip}} {{probe_rs_attach_flags}} "$elf"

# Run checks that will be used in CI
ci-checks:
    cargo clippy \
        --locked \
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
