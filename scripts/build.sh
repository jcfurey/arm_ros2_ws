#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if ! command -v colcon >/dev/null; then
    echo 'colcon is missing. Use Docker, or source ROS and install colcon first.' >&2
    exit 1
fi
python3 scripts/bootstrap.py
# Recursive clones include the hardware SDK; the simulated arm does not use it.
colcon build --symlink-install --event-handlers console_direct+ --packages-ignore xarm_cplus_sdk "$@"
