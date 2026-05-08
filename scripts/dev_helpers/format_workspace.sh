#!/usr/bin/env bash
set -euo pipefail

mode="${1:-fix}"
if [[ "${mode}" != "fix" && "${mode}" != "--check" && "${mode}" != "check" ]]; then
  echo "Usage: $0 [fix|check|--check]" >&2
  exit 2
fi

check=0
if [[ "${mode}" == "check" || "${mode}" == "--check" ]]; then
  check=1
fi

mapfile -t cpp_files < <(
  rg --files \
    -g '*.cpp' \
    -g '*.hpp' \
    -g '*.cu' \
    -g '*.cuh' \
    -g '!build/**' \
    -g '!install/**' \
    -g '!log/**'
)

if ((${#cpp_files[@]})); then
  if ! command -v clang-format >/dev/null 2>&1; then
    if ((check)); then
      echo "clang-format is required for C++/CUDA formatting checks." >&2
      exit 3
    fi
    echo "clang-format is not installed; skipping C++/CUDA formatting." >&2
  elif ((check)); then
    clang-format --dry-run --Werror "${cpp_files[@]}"
  else
    clang-format -i "${cpp_files[@]}"
  fi
fi

mapfile -t python_targets < <(
  rg --files \
    -g '*.py' \
    -g '!build/**' \
    -g '!install/**' \
    -g '!log/**'
)

if ((${#python_targets[@]})); then
  if python3 -m black --version >/dev/null 2>&1; then
    if ((check)); then
      python3 -m black --check "${python_targets[@]}"
    else
      python3 -m black "${python_targets[@]}"
    fi
  else
    echo "black is not installed; skipping Python formatting." >&2
  fi
fi
