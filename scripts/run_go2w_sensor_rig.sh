#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ $# -lt 1 ]]; then
  echo "usage: $0 NEW_OUTPUT_DIR [additional go2w_teleop.py arguments]" >&2
  exit 2
fi
rig_output=$1
shift
if [[ -e "$rig_output" ]]; then
  echo "refusing to overwrite existing output: $rig_output" >&2
  exit 2
fi
exec ./scripts/run_go2w.sh \
  --policy outputs/training/wheel_slope/policy_jit.pt \
  --steps 300 \
  --sensor-rig-config configs/sensors/stage2_rig.json \
  --sensor-rig-output "$rig_output" "$@"
