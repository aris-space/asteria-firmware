#!/usr/bin/env bash
# Build a firmware binary and print the resulting ELF path to stdout.
#
# Usage: build-elf.sh <bin> <target> [cargo build args...]
#
# Resolves or initializes ASTERIA_ARTIFACT_TIMESTAMP_MS (exported so the
# caller can read it), then runs cargo build with JSON output and extracts
# the ELF path via jq.
#
# Prints only the ELF path to stdout; all other output goes to stderr.
set -euo pipefail

bin="$1"; shift
target="$1"; shift

# Resolve artifact timestamp.
artifact_timestamp_ms="${ASTERIA_ARTIFACT_TIMESTAMP_MS:-}"
if [[ -z "$artifact_timestamp_ms" ]]; then
  artifact_timestamp_ms="$(date -u +%s%3N 2>/dev/null || true)"
  if [[ ! "$artifact_timestamp_ms" =~ ^[0-9]{13}$ ]]; then
    artifact_timestamp_ms="$(perl -MTime::HiRes=time -e 'printf("%.0f\n", time() * 1000)')"
  fi
fi
if [[ ! "$artifact_timestamp_ms" =~ ^[0-9]{13}$ ]]; then
  echo "Failed to determine UTC timestamp in milliseconds." >&2
  exit 1
fi
export ASTERIA_ARTIFACT_TIMESTAMP_MS="$artifact_timestamp_ms"

tmp="$(mktemp)"
cargo build --manifest-path app/Cargo.toml \
    --bin "$bin" \
    --target "$target" \
    --message-format=json-render-diagnostics \
    "$@" \
  | tee "$tmp" >/dev/null

elf="$(
  jq -r --arg bin "$bin" '
    select(.reason=="compiler-artifact")
    | select(.target.name==$bin)
    | (.executable // empty)
  ' "$tmp" | tail -n1
)"
rm -f "$tmp"

if [[ -z "$elf" || ! -f "$elf" ]]; then
  echo "Could not determine built ELF path from cargo JSON output." >&2
  exit 1
fi

echo "${artifact_timestamp_ms}:${elf}"
