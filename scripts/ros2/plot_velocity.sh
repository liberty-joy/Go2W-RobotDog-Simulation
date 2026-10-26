#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
source scripts/ros2/env.sh
exec /usr/bin/python3 scripts/ros2/plot_velocity.py "$@"
