#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
exec /home/joy/robotics/isaac-env/bin/python -u scripts/go2w_teleop.py "$@"
