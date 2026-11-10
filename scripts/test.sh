#!/usr/bin/env bash
# Tests that need no running stack: scene, pick and motion-panel logic, the
# bootstrap script, and that the packaged Lichtblick panel matches its source.
# Needs a sourced ROS with moveit_msgs; the workspace need not be built.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
PYTHONPATH="$PWD/src/arm_bringup${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -m pytest -q -p no:cacheprovider src/arm_bringup/test scripts/test_bootstrap.py scripts/test_stop.py
python3 scripts/package_moveit_panel.py --check
