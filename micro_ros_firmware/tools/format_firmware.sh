#!/usr/bin/env bash
set -euo pipefail

firmware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:---check}"

if [[ "$mode" != "--check" && "$mode" != "--write" ]] || [[ $# -gt 1 ]]; then
  printf 'Usage: %s [--check|--write]\n' "$0" >&2
  exit 2
fi

if ! command -v uncrustify >/dev/null 2>&1; then
  printf 'uncrustify is required to format firmware C/C++ sources.\n' >&2
  exit 2
fi

mapfile -d '' source_files < <(
  cd "$firmware_root"
  find src include lib test/host -type f \
    \( -name '*.c' -o -name '*.h' -o -name '*.cc' -o -name '*.cpp' \
       -o -name '*.cxx' -o -name '*.hpp' -o -name '*.hxx' \) -print0 |
    LC_ALL=C sort -z
)

if [[ ${#source_files[@]} -eq 0 ]]; then
  printf 'No firmware C/C++ source files found.\n' >&2
  exit 1
fi

cd "$firmware_root"
if [[ "$mode" == "--write" ]]; then
  uncrustify -q --replace --no-backup -c .uncrustify.cfg "${source_files[@]}"
else
  if check_output="$(uncrustify -q --check -c .uncrustify.cfg "${source_files[@]}" 2>&1)"; then
    printf 'Firmware C/C++ formatting is clean (%d files).\n' "${#source_files[@]}"
  else
    printf '%s\n' "$check_output" >&2
    exit 1
  fi
fi
