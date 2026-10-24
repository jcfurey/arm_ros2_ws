#!/usr/bin/env bash
set -e
# Only these named volumes need ownership; never recursively chown the host repo.
chown ros:ros /ws/build /ws/install /ws/log
exec gosu ros /usr/local/bin/ros-shell.sh "$@"
