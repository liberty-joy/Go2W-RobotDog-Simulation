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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
./scripts/run_go2w_ros2.sh \
  --policy outputs/training/wheel_slope/policy_jit.pt \
  --stage4-lidar-config configs/sensors/stage4_lidar.json \
  --stage4-lidar-output "$run_dir" "$@" 2>&1 | tee "$run_dir/isaac_capture.log"
if [[ ! -f "$run_dir/capture_summary.json" ]]; then
  echo "Stage 4 capture did not finish; partial output and failure.json are preserved" >&2
  exit 1
fi
/home/joy/robotics/isaac-env/bin/python -u scripts/sensor_pipeline/analyze_stage4_lidar.py \
  --run "$run_dir" 2>&1 | tee "$run_dir/analysis.log"
