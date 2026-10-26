#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export ROBOTDOG_ROS_RUNTIME=bundled
source scripts/ros2/env.sh
exec /home/joy/robotics/isaac-env/bin/python -u scripts/go2w_teleop.py --ros2 "$@"
