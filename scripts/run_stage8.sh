#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
run="${1:-outputs/sensor_pipeline/stage8_$(date +%Y%m%dT%H%M%S)}"
[[ ! -e "$run" ]] || { echo 'Use a new run directory'; exit 1; }
phase=scan
on_exit() {
 code=$?
 if [[ $code -ne 0 && -d "$run" ]]; then
  /usr/bin/python3 - "$run" "$phase" "$code" <<'PYERR'
import json,sys,datetime
from pathlib import Path
r=Path(sys.argv[1]);(r/'failure_pipeline.json').write_text(json.dumps({'phase':sys.argv[2],'exit_code':int(sys.argv[3]),'time':datetime.datetime.now().astimezone().isoformat(),'policy':'failure retained; no automatic retry'},indent=2))
PYERR
 fi
}
trap on_exit EXIT
export OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
isaac_python=/home/joy/robotics/isaac-env/bin/python
# The scan creates a fresh root; all following mutations are confined to this run.
timeout --signal=TERM --kill-after=10s 180 "$isaac_python" -u scripts/sensor_pipeline/stage8_no_training.py scan --out "$run"
for part in validation final_test; do
 phase="${part}_capture"
 mkdir -p "$run/${part}_capture"
 timeout --signal=TERM --kill-after=10s 180 scripts/run_go2w.sh --headless \
  --policy outputs/training/wheel_slope/policy_jit.pt --steps 12000 \
  --stage3-stereo-config "configs/sensors/stage8_${part}.json" \
  --stage3-stereo-output "$run/${part}_capture" > "$run/${part}_capture/simulator.log" 2>&1
 [[ -f "$run/${part}_capture/captures.json" ]]
 phase="${part}_evaluate"
 "$isaac_python" -u scripts/sensor_pipeline/stage8_no_training.py evaluate --out "$run" \
  --capture "$run/${part}_capture" --partition "$part" > "$run/${part}_evaluation.log" 2>&1
done
phase=report
"$isaac_python" -u scripts/sensor_pipeline/stage8_no_training.py report --out "$run" > "$run/report.log" 2>&1
echo "Stage8 complete: $run/results.html"
