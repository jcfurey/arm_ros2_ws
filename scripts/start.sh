#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
bash scripts/build.sh
source install/setup.bash
children=()
cleanup() {
    trap - EXIT TERM INT
    if ((${#children[@]})); then kill -TERM "${children[@]}" 2>/dev/null || true; fi
    wait || true
}
trap cleanup EXIT
trap 'exit 0' TERM INT
if [[ ${ENABLE_DESKTOP:-0} == 1 ]]; then
    /usr/local/bin/desktop.sh > /tmp/desktop.log 2>&1 &
    children+=("$!")
    for ((attempt=0; attempt<100; attempt++)); do
        if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then break; fi
        sleep 0.1
    done
    xdpyinfo -display "$DISPLAY" >/dev/null
    echo 'Desktop: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
fi
ros2 launch arm_bringup demo.launch.py \
    robot:="${ROBOT_MODEL:-xarm6}" mode:="${SIM_MODE:-gazebo}" \
    world:="${TEST_WORLD:-workcell}" gui:="${GUI:-false}" rviz:="${RVIZ:-false}" \
    ousters:="${ENABLE_OUSTERS:-true}" perception:="${ENABLE_PERCEPTION:-false}" \
    cameras:="${ENABLE_CAMERAS:-true}" gripper:="${ENABLE_GRIPPER:-true}" \
    wrist_imu:="${WRIST_IMU:-false}" imu_noise:="${IMU_NOISE:-true}" \
    lidar_hz:="${LIDAR_HZ:-10.0}" "$@" &
children+=("$!")
wait -n "${children[@]}"
