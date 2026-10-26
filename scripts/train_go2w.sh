#!/usr/bin/env bash
set -e
export OMNI_KIT_ACCEPT_EULA=YES
export ACCEPT_EULA=Y
# OpenBLAS worker teardown can race with Kit telemetry subprocess forks.
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export NUMEXPR_NUM_THREADS=4
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
exec /home/joy/robotics/isaaclab-env/bin/python -u training/go2w/run.py "$@"
