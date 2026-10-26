#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
stage_out="$(realpath -m -- "${1:?Provide a new output directory}")"
mode="${2:-headless}"
if [[ -e "$stage_out" ]]; then echo "Refusing to overwrite $stage_out" >&2; exit 1; fi
mkdir -p "$stage_out"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY=1
unset ROS_AUTOMATIC_DISCOVERY_RANGE
set +u  # Native ROS setup scripts reference optional unset variables.
source scripts/ros2/env.sh
set -u
sim_args=(--headless --steps 3600)
probe_args=(--seconds 30 --bag)
if [[ "$mode" == gui ]]; then
  [[ -n "${DISPLAY:-}" ]] || { echo 'No DISPLAY: GUI not tested' >&2; exit 1; }
  sim_args=(--steps 1800);probe_args=(--seconds 10)
elif [[ "$mode" != headless ]]; then echo 'Mode must be headless or gui' >&2; exit 1; fi
printf '%q ' scripts/run_go2w_ros2.sh "${sim_args[@]}" --ros-image-reliable --policy outputs/training/wheel_slope/policy_jit.pt > "$stage_out/sim_command.txt"
scripts/ros2/python.sh scripts/sensor_pipeline/probe.py --out "$stage_out" --reliable-images "${probe_args[@]}" > "$stage_out/probe.log" 2>&1 &
probe_pid=$!
scripts/run_go2w_ros2.sh "${sim_args[@]}" --ros-image-reliable --policy outputs/training/wheel_slope/policy_jit.pt > "$stage_out/sim.log" 2>&1 &
sim_pid=$!
cleanup() { kill "$probe_pid" "$sim_pid" 2>/dev/null || true; }
trap cleanup EXIT
printf '%s\n' "$sim_pid" > "$stage_out/sim.pid"
(
  while kill -0 "$sim_pid" 2>/dev/null; do
    date -Iseconds
    nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits
    ps -p "$sim_pid" -o rss=,%cpu= || true
    sleep 1
  done
) > "$stage_out/resources.log" &
monitor_pid=$!
trap 'cleanup; kill "$monitor_pid" 2>/dev/null || true' EXIT
wait "$probe_pid"
wait "$sim_pid"
wait "$monitor_pid" || true
trap - EXIT
echo "STAGE0_CAPTURE_PASS $stage_out ($mode); GUI human interaction not tested"
