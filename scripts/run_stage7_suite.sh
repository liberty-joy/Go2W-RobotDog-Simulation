#!/usr/bin/env bash
# One-command local-only engineering regression. Does not train or install environments.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
run="${1:-outputs/sensor_pipeline/stage7_$(date +%Y%m%dT%H%M%S)}"
[[ ! -e "$run" && ! -e "${run}_vision_io" ]] || { echo 'Use a new output directory'; exit 1; }
mkdir -p "$(dirname -- "$run")"
/usr/bin/python3 scripts/sensor_pipeline/stage7_prepare_model.py
vision_io="${run}_vision_io"
/home/joy/robotics/isaac-env/bin/python -u scripts/sensor_pipeline/stage7_vision_worker.py --io "$vision_io" > "${run}_vision.log" 2>&1 &
vision_pid=$!
cleanup() {
 if [[ -d "$vision_io" ]]; then touch "$vision_io/stop"; fi
 if kill -0 "$vision_pid" 2>/dev/null; then kill "$vision_pid"; fi
}
trap cleanup EXIT
for i in {1..100}; do
 [[ -f "$vision_io/ready.json" ]] && break
 kill -0 "$vision_pid"
 sleep .1
done
[[ -f "$vision_io/ready.json" ]]
/usr/bin/python3 -u scripts/sensor_pipeline/stage7_suite.py --out "$run" --vision-io "$vision_io" > "${run}_suite.log" 2>&1
touch "$vision_io/stop"
wait "$vision_pid"
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 scripts/sensor_pipeline/stage7_report.py --suite "$run" --dev outputs/sensor_pipeline/stage7_20261027_dev > "$run/report.log" 2>&1
/usr/bin/python3 - "$run" <<'PY'
import json,sys
from pathlib import Path
r=Path(sys.argv[1]);a=json.loads((r/'acceptance.json').read_text())
assert a['completed_trials']==12 and a['all_trials_have_real_results']
assert all(a['same_fixed_parameters'].values()) and all(a['frozen_source_hashes_match'].values())
assert a['all_model_input_hashes_match'] and a['observation_budget_respected']
print('Stage7 complete:',r/'results.html')
PY
