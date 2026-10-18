#!/usr/bin/env bash
set -euo pipefail

die() {
  echo "$*" >&2
  exit 1
}

get_repo_root() {
  git rev-parse --show-toplevel 2>/dev/null || pwd
}

resolve_local_artifact_dir() {
  local repo_root dir
  repo_root="$(get_repo_root)"
  dir="${ASTERIA_LOCAL_ARTIFACT_DIR:-$repo_root/.artifacts}"
  if [[ "$dir" != /* ]]; then
    dir="$repo_root/$dir"
  fi
  printf "%s\n" "$dir"
}

now_utc_ms() {
  local ts
  ts="$(date -u +%s%3N 2>/dev/null || true)"
  if [[ ! "$ts" =~ ^[0-9]{13}$ ]]; then
    ts="$(perl -MTime::HiRes=time -e 'printf("%.0f\n", time() * 1000)')"
  fi
  [[ "$ts" =~ ^[0-9]{13}$ ]] || die "Failed to determine UTC timestamp in milliseconds for artifact naming."
  printf "%s\n" "$ts"
}

artifact_timestamp_ms() {
  local ts
  ts="${ASTERIA_ARTIFACT_TIMESTAMP_MS:-}"
  if [[ -z "$ts" ]]; then
    ts="$(now_utc_ms)"
  fi
  [[ "$ts" =~ ^[0-9]{13}$ ]] || die "Invalid ASTERIA_ARTIFACT_TIMESTAMP_MS (expected 13-digit UTC epoch ms): $ts"
  printf "%s\n" "$ts"
}

load_b2_config() {
  command -v s5cmd >/dev/null 2>&1 || die "s5cmd is required for artifact upload. Install s5cmd and retry."

  local repo_root b2_env_file
  repo_root="$(get_repo_root)"
  b2_env_file="${ASTERIA_B2_ENV_FILE:-$repo_root/.b2.env}"
  [[ -f "$b2_env_file" ]] || die "Missing B2 env file: $b2_env_file"$'\n'"Create it from $repo_root/.b2.env.example (or set ASTERIA_B2_ENV_FILE)."

  set -a
  # shellcheck disable=SC1090
  source "$b2_env_file"
  set +a

  B2_KEY_ID="${ASTERIA_B2_KEY_ID:-${AWS_ACCESS_KEY_ID:-}}"
  B2_APPLICATION_KEY="${ASTERIA_B2_APPLICATION_KEY:-${AWS_SECRET_ACCESS_KEY:-}}"
  B2_BUCKET="${ASTERIA_B2_BUCKET:-asteria-firmware-images}"
  B2_ENDPOINT="${ASTERIA_B2_ENDPOINT:-s3.eu-central-003.backblazeb2.com}"

  [[ -n "$B2_KEY_ID" ]] || die "Missing ASTERIA_B2_KEY_ID (or AWS_ACCESS_KEY_ID)."
  [[ -n "$B2_APPLICATION_KEY" ]] || die "Missing ASTERIA_B2_APPLICATION_KEY (or AWS_SECRET_ACCESS_KEY)."

  case "$B2_ENDPOINT" in
    http://*|https://*) ;;
    *) B2_ENDPOINT="https://$B2_ENDPOINT" ;;
  esac
}

save_local_artifact() {
  local elf_path="$1"
  local object_name="$2"
  local local_dir local_path

  local_dir="$(resolve_local_artifact_dir)"
  mkdir -p "$local_dir"
  local_path="$local_dir/$object_name"
  cp "$elf_path" "$local_path"
  echo "Saved local artifact: $local_path"
}

upload_object() {
  local local_path="$1"
  local object_name="$2"
  local object_uri

  object_uri="s3://${B2_BUCKET}/${object_name}"
  echo "Uploading ELF artifact to: $object_uri"
  AWS_ACCESS_KEY_ID="$B2_KEY_ID" \
    AWS_SECRET_ACCESS_KEY="$B2_APPLICATION_KEY" \
    s5cmd --endpoint-url "$B2_ENDPOINT" cp "$local_path" "$object_uri"
  echo "Uploaded ELF artifact."
}

sync_cached_artifacts() {
  local dry_run="$1"
  local artifacts_dir local_files_tmp remote_names_tmp batch_tmp
  local missing present file name list_out

  load_b2_config

  artifacts_dir="$(resolve_local_artifact_dir)"
  if [[ ! -d "$artifacts_dir" ]]; then
    echo "No local artifacts directory found at: $artifacts_dir"
    return 0
  fi

  local_files_tmp="$(mktemp)"
  remote_names_tmp="$(mktemp)"
  batch_tmp="$(mktemp)"
  missing=0
  present=0

  find "$artifacts_dir" -maxdepth 1 -type f -name '*.elf' -print | sort >"$local_files_tmp"
  if [[ ! -s "$local_files_tmp" ]]; then
    echo "No local ELF artifacts to sync in: $artifacts_dir"
    rm -f "$local_files_tmp" "$remote_names_tmp" "$batch_tmp"
    return 0
  fi

  list_out="$(
    AWS_ACCESS_KEY_ID="$B2_KEY_ID" \
      AWS_SECRET_ACCESS_KEY="$B2_APPLICATION_KEY" \
      s5cmd --endpoint-url "$B2_ENDPOINT" ls "s3://${B2_BUCKET}/*.elf" 2>&1 || true
  )"
  printf "%s\n" "$list_out" \
    | awk '{print $NF}' \
    | sed -E 's#^s3://[^/]*/##' \
    | awk 'NF > 0' \
    | sort -u >"$remote_names_tmp"

  while IFS= read -r file; do
    name="$(basename "$file")"
    if grep -Fxq "$name" "$remote_names_tmp"; then
      present=$((present + 1))
      continue
    fi

    if [[ "$dry_run" -eq 1 ]]; then
      echo "Would upload missing artifact: $file -> s3://${B2_BUCKET}/${name}"
    else
      printf 'cp %s s3://%s/%s\n' "$file" "$B2_BUCKET" "$name" >>"$batch_tmp"
    fi
    missing=$((missing + 1))
  done <"$local_files_tmp"

  if [[ "$dry_run" -eq 1 ]]; then
    echo "Sync dry-run complete: missing=$missing present=$present"
    rm -f "$local_files_tmp" "$remote_names_tmp" "$batch_tmp"
    return 0
  fi

  if [[ "$missing" -eq 0 ]]; then
    echo "Sync complete: uploaded=0 already_present=$present"
    rm -f "$local_files_tmp" "$remote_names_tmp" "$batch_tmp"
    return 0
  fi

  AWS_ACCESS_KEY_ID="$B2_KEY_ID" \
    AWS_SECRET_ACCESS_KEY="$B2_APPLICATION_KEY" \
    s5cmd --endpoint-url "$B2_ENDPOINT" run "$batch_tmp"
  echo "Sync complete: uploaded=$missing already_present=$present"
  rm -f "$local_files_tmp" "$remote_names_tmp" "$batch_tmp"
}

main() {
  [[ $# -ge 1 ]] || die "Usage: $0 <elf-path> [object-name] | sync [--dry-run]"
  local elf_path object_name dry_run

  if [[ "$1" == "sync" ]]; then
    shift
    dry_run=0
    if [[ "${1:-}" == "--dry-run" ]]; then
      dry_run=1
      shift
    fi
    [[ $# -eq 0 ]] || die "Usage: $0 sync [--dry-run]"
    sync_cached_artifacts "$dry_run"
    return 0
  fi

  [[ $# -le 2 ]] || die "Usage: $0 <elf-path> [object-name]"
  elf_path="$1"
  object_name="${2:-}"

  [[ -f "$elf_path" ]] || die "ELF file not found: $elf_path"
  [[ -r "$elf_path" ]] || die "ELF file not readable: $elf_path"

  if [[ -z "$object_name" ]]; then
    object_name="$(artifact_timestamp_ms).elf"
  fi
  [[ "$object_name" == */* ]] && die "Object name must not contain '/': $object_name"

  # Always cache locally, regardless of B2 configuration.
  save_local_artifact "$elf_path" "$object_name"

  # Upload to B2 only if configured.
  local repo_root b2_env_file
  repo_root="$(get_repo_root)"
  b2_env_file="${ASTERIA_B2_ENV_FILE:-$repo_root/.b2.env}"
  if [[ -f "$b2_env_file" ]] && command -v s5cmd >/dev/null 2>&1; then
    load_b2_config
    upload_object "$elf_path" "$object_name"
  else
    echo "WARNING: B2 not configured — skipping remote upload." >&2
    echo "WARNING: ELF artifact saved locally only. Set up .b2.env for remote backup." >&2
  fi
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
