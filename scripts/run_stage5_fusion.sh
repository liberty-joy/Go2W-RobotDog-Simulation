#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ $# -lt 1 ]]; then
  echo "usage: $0 NEW_OUTPUT_DIR [--headless]" >&2
  exit 2
fi
run_dir=$1
shift
if [[ -e "$run_dir" ]]; then
  echo "refusing to overwrite existing output: $run_dir" >&2
  exit 2
fi
mkdir -p -- "$run_dir"
./scripts/run_go2w.sh \
  --policy outputs/training/wheel_slope/policy_jit.pt \
  --stage5-fusion-config configs/sensors/stage5_fusion.json \
  --stage5-fusion-output "$run_dir" "$@" 2>&1 | tee "$run_dir/isaac_capture.log"
if [[ ! -f "$run_dir/capture_summary.json" ]]; then
  echo "Stage 5 capture incomplete; partial data/logs retained" >&2
  exit 1
fi
/home/joy/robotics/isaac-env/bin/python -u scripts/sensor_pipeline/analyze_stage5_fusion.py \
  --run "$run_dir" 2>&1 | tee "$run_dir/analysis.log"
