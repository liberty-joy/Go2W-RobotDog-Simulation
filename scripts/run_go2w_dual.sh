#!/usr/bin/env bash
# Latest wheel controller plus experimental walking controller and stationary switching.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
for policy_file in outputs/training/wheel_slope/policy_jit.pt outputs/training/walk_clearance/policy_jit.pt; do
  if [[ ! -f "$policy_file" ]]; then
    echo "Missing policy: $policy_file" >&2
    exit 1
  fi
done
exec ./scripts/run_go2w_ros2.sh \
  --policy outputs/training/wheel_slope/policy_jit.pt \
  --walk-policy outputs/training/walk_clearance/policy_jit.pt "$@"
