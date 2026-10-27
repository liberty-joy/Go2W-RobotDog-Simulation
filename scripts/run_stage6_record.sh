#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ROBOTDOG_ROS_RUNTIME=native
source scripts/ros2/env.sh
run="${1:-outputs/sensor_pipeline/stage6_$(date +%Y%m%dT%H%M%S)}"
if [[ -e "$run" ]]; then echo "Refusing existing output $run"; exit 1; fi
mkdir -p "$run"
/usr/bin/python3 - "$run" <<'PY'
import sys,shutil,json,subprocess,datetime
from pathlib import Path
p=Path(sys.argv[1]);disk=shutil.disk_usage(p)
assert disk.free>8*1024**3,'Less than 8 GiB free'
(p/'metadata.json').write_text(json.dumps({'started':datetime.datetime.now().astimezone().isoformat(),'git':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'disk_free_before':disk.free,'expected_uncompressed_gib':1.6,'command':'scripts/run_stage6_record.sh '+str(p),'ros_domain_id':97,'rmw':'rmw_fastrtps_cpp','headless':True},indent=2))
shutil.copy('configs/sensors/stage6_ros_ab.json',p/'config.json')
PY
/usr/bin/python3 -u scripts/sensor_pipeline/stage6_bag.py record --out "$run/record" --publisher-output "$run/publisher" > "$run/record.log" 2>&1 &
rec_pid=$!
sim_pid=''
cleanup() { if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then kill "$sim_pid"; fi; if kill -0 "$rec_pid" 2>/dev/null; then kill "$rec_pid"; fi; }
trap cleanup EXIT
for i in {1..100}; do [[ -e "$run/record/ready" ]] && break; kill -0 "$rec_pid"; sleep .1; done
[[ -e "$run/record/ready" ]]
scripts/run_go2w_ros2.sh --headless --sensor-pipeline-config configs/sensors/stage6_ros_ab.json --sensor-pipeline-output "$run/publisher" > "$run/simulator.log" 2>&1 &
sim_pid=$!
record_deadline=$((SECONDS+240))
while kill -0 "$sim_pid" 2>/dev/null; do
  if (( SECONDS > record_deadline )); then echo "Stage6 simulator deadline exceeded; preserving logs" >&2; exit 1; fi
  if ! kill -0 "$rec_pid" 2>/dev/null && [[ ! -e "$run/record/metrics.json" ]]; then echo "Recorder failed; preserving logs" >&2; exit 1; fi
  { date --iso-8601=seconds; ps -o pid=,rss=,pcpu= -p "$sim_pid" || true; nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader,nounits; } >> "$run/resources.log" 2>&1
  sleep 1
done
wait "$sim_pid"
wait "$rec_pid"
sim_pid=''
/usr/bin/python3 -u scripts/sensor_pipeline/stage6_bag.py replay --out "$run/replay" > "$run/replay.log" 2>&1
ros2 bag info "$run/record/bag" > "$run/bag_info.txt"
df -B1 . > "$run/disk_after.txt"
echo "Stage6 recording/replay complete: $run"
