#!/usr/bin/env bash
set -euo pipefail

root="${1:-/opt/jetauto_orin_amr/models}"
mkdir -p "${root}/yolo" "${root}/rf_detr" "${root}/baseline"
cat <<EOF
Model directory layout:
  ${root}/yolo/*.engine
  ${root}/rf_detr/*.engine
  ${root}/baseline/*.engine

Keep raw training/export artifacts outside this repository. Commit export notes and benchmark reports, not model weights.
EOF

