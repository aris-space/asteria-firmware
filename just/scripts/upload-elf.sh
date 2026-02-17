#!/usr/bin/env bash
set -euo pipefail

die() {
  echo "$*" >&2
  exit 1
}

resolve_local_artifact_dir() {
  local dir="${ASTERIA_LOCAL_ARTIFACT_DIR:-$REPO_ROOT/.artifacts}"
  if [[ "$dir" != /* ]]; then
    dir="$REPO_ROOT/$dir"
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

load_b2_config() {
  command -v s5cmd >/dev/null 2>&1 || die "s5cmd is required for artifact upload. Install s5cmd and retry."

  local repo_root b2_env_file
  repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
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

main() {
  [[ $# -ge 1 && $# -le 2 ]] || die "Usage: $0 <elf-path> [object-name]"
  local elf_path="$1"
  local object_name="${2:-}"
  local object_uri
  REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"

  [[ -f "$elf_path" ]] || die "ELF file not found: $elf_path"
  [[ -r "$elf_path" ]] || die "ELF file not readable: $elf_path"

  load_b2_config

  if [[ -z "$object_name" ]]; then
    object_name="$(now_utc_ms).elf"
  fi
  [[ "$object_name" == */* ]] && die "Object name must not contain '/': $object_name"

  save_local_artifact "$elf_path" "$object_name"

  object_uri="s3://${B2_BUCKET}/${object_name}"
  echo "Uploading ELF artifact to: $object_uri"
  AWS_ACCESS_KEY_ID="$B2_KEY_ID" \
    AWS_SECRET_ACCESS_KEY="$B2_APPLICATION_KEY" \
    s5cmd --endpoint-url "$B2_ENDPOINT" cp "$elf_path" "$object_uri"
  echo "Uploaded ELF artifact."
}

main "$@"
