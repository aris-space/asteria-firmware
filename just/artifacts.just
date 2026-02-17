# Upload a specific ELF file. Object name defaults to UTC epoch milliseconds.
upload-elf elf_path:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    exec "$repo_root/just/scripts/upload-elf.sh" "{{elf_path}}"

# Upload a specific ELF file with an explicit object name (stored at bucket root).
upload-elf-as elf_path object_name:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    exec "$repo_root/just/scripts/upload-elf.sh" "{{elf_path}}" "{{object_name}}"

# Upload the most recently built ELF for an arbitrary target/bin pair.
upload-latest-by target_triple bin_name:
    #!/usr/bin/env bash
    set -euo pipefail

    elf="$(find "target/{{target_triple}}" -type f -name "{{bin_name}}" -print0 | xargs -0 ls -t 2>/dev/null | head -n1 || true)"
    [[ -n "$elf" ]] || { echo "No built {{bin_name}} found under target/{{target_triple}}." >&2; exit 1; }

    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    exec "$repo_root/just/scripts/upload-elf.sh" "$elf"

# Remove all locally cached ELF artifacts.
clean-artifacts:
    #!/usr/bin/env bash
    set -euo pipefail

    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    artifacts_dir="${ASTERIA_LOCAL_ARTIFACT_DIR:-$repo_root/.artifacts}"
    if [[ "$artifacts_dir" != /* ]]; then
      artifacts_dir="$repo_root/$artifacts_dir"
    fi

    if [[ ! -d "$artifacts_dir" ]]; then
      echo "No local artifacts directory found at: $artifacts_dir"
      exit 0
    fi

    rm -rf "$artifacts_dir"
    mkdir -p "$artifacts_dir"
    echo "Cleaned local artifacts: $artifacts_dir"

# Upload local cached ELF artifacts that are missing in the bucket.
sync-artifacts:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    exec "$repo_root/just/scripts/upload-elf.sh" sync

# Dry-run for syncing local cached ELF artifacts.
sync-artifacts-dry-run:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
    exec "$repo_root/just/scripts/upload-elf.sh" sync --dry-run
