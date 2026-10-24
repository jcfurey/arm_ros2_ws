#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if ! command -v colcon >/dev/null; then
    echo 'colcon is missing. Use Docker, or source ROS and install colcon first.' >&2
    exit 1
fi
colcon build --symlink-install --event-handlers console_direct+ "$@"
