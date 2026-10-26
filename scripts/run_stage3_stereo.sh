#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ $# -lt 1 ]]; then
  echo "usage: $0 NEW_OUTPUT_DIR [--config CONFIG.json] [--params PARAMS.json] [--headless]" >&2
  exit 2
fi
run_dir=$1
shift
config=configs/sensors/stage3_stereo.json
params=configs/sensors/stage3_sgbm.json
extra_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) config=$2; shift 2 ;;
    --params) params=$2; shift 2 ;;
    *) extra_args+=("$1"); shift ;;
  esac
done
if [[ -e "$run_dir" ]]; then
  echo "refusing to overwrite existing output: $run_dir" >&2
  exit 2
fi
mkdir -p -- "$run_dir"
./scripts/run_go2w.sh \
  --policy outputs/training/wheel_slope/policy_jit.pt \
  --steps 12000 \
  --stage3-stereo-config "$config" \
  --stage3-stereo-output "$run_dir" "${extra_args[@]}" 2>&1 | tee "$run_dir/isaac_capture.log"
if [[ ! -f "$run_dir/captures.json" ]]; then
  echo "Isaac capture ended without captures.json; see preserved log and failures.jsonl" >&2
  exit 1
fi
/home/joy/robotics/isaac-env/bin/python -u scripts/sensor_pipeline/process_stage3_stereo.py \
  --run "$run_dir" --params "$params" 2>&1 | tee "$run_dir/analysis.log"
