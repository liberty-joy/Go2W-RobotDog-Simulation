#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
source scripts/ros2/env.sh
exec /opt/ros/jazzy/bin/rviz2 -d ros2/config/go2w.rviz --ros-args -p use_sim_time:=true
