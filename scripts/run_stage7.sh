#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
run="${1:?Usage: scripts/run_stage7.sh NEW_RUN [oracle|estimated] [validation|task] [config]}"
mode="${2:-estimated}"
suite="${3:-validation}"
config="${4:-configs/sensors/stage7_closed_loop.json}"
[[ ! -e "$run" ]] || { echo "Refusing existing run"; exit 1; }
mkdir -p "$run"
cp "$config" "$run/config.json"
export ROS_DOMAIN_ID=101 ROS_LOCALHOST_ONLY=1 ROBOTDOG_ROS_RUNTIME=native
source scripts/ros2/env.sh
/usr/bin/python3 -u scripts/sensor_pipeline/stage7_navigation.py --config "$run/config.json" --out "$run/navigation" --localization "$mode" --suite "$suite" > "$run/navigation.log" 2>&1 &
nav_pid=$!
sim_pid=''
cleanup() { for child in "$nav_pid" "$sim_pid"; do if [[ -n "$child" ]] && kill -0 "$child" 2>/dev/null; then kill "$child"; fi; done; }
trap cleanup EXIT
scripts/run_go2w_ros2.sh --headless --closed-loop-config "$run/config.json" --closed-loop-output "$run/publisher" > "$run/simulator.log" 2>&1 &
sim_pid=$!
deadline=$((SECONDS+260))
while kill -0 "$sim_pid" 2>/dev/null; do
 if (( SECONDS>deadline )); then echo 'Stage7 deadline';exit 1;fi
 if ! kill -0 "$nav_pid" 2>/dev/null && [[ ! -e "$run/publisher/navigation_done.json" ]]; then echo 'Navigation process failed';exit 1;fi
 { date --iso-8601=seconds; ps -o pid=,rss=,pcpu= -p "$sim_pid" "$nav_pid" || true; nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader,nounits; } >> "$run/resources.log" 2>&1
 sleep 1
done
wait "$sim_pid"
wait "$nav_pid"
if [[ -d "$run/publisher/video_frames" ]]; then
 /usr/bin/python3 scripts/sensor_pipeline/stage7_video.py --run "$run" > "$run/video.log" 2>&1
fi
cat "$run/navigation/metrics.json"
