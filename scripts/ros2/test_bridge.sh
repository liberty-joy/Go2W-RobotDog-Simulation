#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
bridge_out="${ROBOTDOG_OUTPUT_DIR:-outputs/ros2}"
mkdir -p "$bridge_out"
scripts/ros2/python.sh scripts/ros2/verify_bridge.py > "$bridge_out/check-bridge.log" 2>&1 &
check_pid=$!
trap 'kill "$check_pid" 2>/dev/null || true' EXIT
scripts/run_go2w_ros2.sh --headless --steps "${ROBOTDOG_SIM_STEPS:-1800}" "$@" > "$bridge_out/sim-bridge.log" 2>&1
if ! rg -q 'GO2W_SIMULATION_COMPLETE' "$bridge_out/sim-bridge.log"; then
  echo 'Simulator did not reach its completion marker; see sim-bridge.log' >&2
  exit 1
fi
wait "$check_pid"
rg ROS2_INTEGRATION_PASS "$bridge_out/check-bridge.log"
