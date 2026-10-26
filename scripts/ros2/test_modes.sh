#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
mode_out="${ROBOTDOG_OUTPUT_DIR:-outputs/ros2}"
mkdir -p "$mode_out"
scripts/ros2/python.sh scripts/ros2/verify_modes.py > "$mode_out/check-modes.log" 2>&1 &
mode_check_pid=$!
trap 'kill "$mode_check_pid" 2>/dev/null || true' EXIT
scripts/run_go2w_ros2.sh --headless --steps 1800 "$@" > "$mode_out/sim-modes.log" 2>&1
if ! rg -q 'GO2W_SIMULATION_COMPLETE' "$mode_out/sim-modes.log"; then
  echo 'Simulator did not complete; inspect sim-modes.log' >&2
  exit 1
fi
wait "$mode_check_pid"
rg GO2W_MODE_CHECK_PASS "$mode_out/check-modes.log"
