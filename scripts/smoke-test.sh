#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mode=${1:-gazebo}
case "$mode" in gazebo|mock) ;; *) echo 'Expected gazebo or mock' >&2; exit 2 ;; esac
bash scripts/build.sh
source install/setup.bash
launch_log=$(mktemp /tmp/arm-smoke.XXXXXX)
ros2 launch arm_bringup demo.launch.py mode:="$mode" gui:=false rviz:=false > "$launch_log" 2>&1 &
launch_pid=$!
cleanup() {
    trap - EXIT TERM INT
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' TERM INT
if ! ros2 run arm_bringup check_stack --mode "$mode"; then
    cat "$launch_log"
    exit 1
fi
echo "Launch log: $launch_log"
