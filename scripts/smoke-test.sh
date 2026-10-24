#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mode=${1:-gazebo}
robot=${2:-${ROBOT_MODEL:-xarm6}}
world=${3:-${TEST_WORLD:-workcell}}
case "$mode" in gazebo|mock) ;; *) echo 'Expected gazebo or mock' >&2; exit 2 ;; esac
bash scripts/build.sh
source install/setup.bash
launch_log=$(mktemp /tmp/arm-smoke.XXXXXX)
ros2 launch arm_bringup demo.launch.py robot:="$robot" mode:="$mode" world:="$world" \
    ousters:="${ENABLE_OUSTERS:-true}" perception:="${ENABLE_PERCEPTION:-false}" \
    cameras:="${ENABLE_CAMERAS:-true}" gripper:="${ENABLE_GRIPPER:-true}" \
    lidar_hz:="${LIDAR_HZ:-10.0}" bridge:=false gui:=false rviz:=false > "$launch_log" 2>&1 &
launch_pid=$!
cleanup() {
    trap - EXIT TERM INT
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' TERM INT
sensor_args=()
if [[ "$mode" == gazebo && ${ENABLE_OUSTERS:-true} == true ]]; then sensor_args+=(--ousters); fi
if ! ros2 run arm_bringup check_stack --robot "$robot" --mode "$mode" --world "$world" "${sensor_args[@]}"; then
    cat "$launch_log"
    exit 1
fi
if ((${#sensor_args[@]})); then
    if ! python3 scripts/check_registration.py --world "$world"; then
        cat "$launch_log"
        exit 1
    fi
fi
# Physics-level pick and place: the gripper must carry the part to its drop frame.
if [[ "$mode" == gazebo && "$robot" == xarm6 && "$world" == workcell && ${ENABLE_GRIPPER:-true} == true ]]; then
    if ! python3 scripts/check_pick.py --world "$world" --part pick_cube; then
        cat "$launch_log"
        exit 1
    fi
fi
echo "Launch log: $launch_log"
