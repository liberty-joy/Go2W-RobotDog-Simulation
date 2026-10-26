#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
if [[ "${ROBOTDOG_ROS_RUNTIME:-auto}" != bundled && -f /opt/ros/jazzy/setup.bash ]]; then
  exec /usr/bin/python3 -u "$@"
fi
exec /home/joy/robotics/isaac-env/bin/python -u "$@"
