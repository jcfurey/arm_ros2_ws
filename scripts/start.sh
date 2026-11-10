#!/usr/bin/env bash
# Native or container workcell launcher. Lichtblick Desktop connects separately.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

usage() {
    cat <<'EOF'
Usage: bash scripts/start.sh [--build | --cuda] [ROS launch arguments...]

Start the headless xArm6 workcell, MoveIt, cameras, lidars, and Foxglove Bridge.
Connect Lichtblick Desktop to ws://localhost:8765. Ctrl-C stops the stack.
From another terminal, use bash scripts/stop.sh.

  --build     Rebuild with the default CPU profile (automatic on first run).
  --cuda      Rebuild with the native NVIDIA CUDA profile, then launch.
  -h, --help  Show this help.

Examples:
  bash scripts/start.sh
  bash scripts/start.sh --build
  bash scripts/start.sh --cuda perception:=true
  bash scripts/start.sh perception:=true
  bash scripts/start.sh world:=obstacle_course cameras:=false
  SIM_MODE=mock bash scripts/start.sh
EOF
}

rebuild=false
cuda=false
launch_args=()
while (($#)); do
    case "$1" in
        --build) rebuild=true ;;
        --cuda) cuda=true; rebuild=true ;;
        -h|--help) usage; exit 0 ;;
        --) shift; launch_args+=("$@"); break ;;
        *) launch_args+=("$1") ;;
    esac
    shift
done

ros_setup=${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash}
if [[ ! -r "$ros_setup" ]]; then
    echo "ROS setup not found: $ros_setup. Install ROS 2 Lyrical or set ROS_SETUP." >&2
    exit 1
fi
source "$ros_setup"

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$PWD/docker/cyclonedds.xml}
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
export OMP_WAIT_POLICY=${OMP_WAIT_POLICY:-PASSIVE}
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4}
export MAKEFLAGS=${MAKEFLAGS:--j4}
export ROS_LOG_DIR=${ROS_LOG_DIR:-$PWD/log/ros}
mkdir -p "$ROS_LOG_DIR"

if [[ "$rebuild" == true || ! -f install/setup.bash ]]; then
    build_args=(--packages-up-to arm_bringup --parallel-workers "${COLCON_WORKERS:-2}")
    if [[ "$cuda" == true ]]; then build_args=(--cuda "${build_args[@]}"); fi
    bash scripts/build.sh "${build_args[@]}"
fi
source install/setup.bash
if [[ "$cuda" == true ]]; then export GZ_OUSTER_BACKEND=cuda; fi
if ! command -v setsid >/dev/null; then
    echo 'setsid is missing; install util-linux to supervise the simulation processes.' >&2
    exit 1
fi

children=()
signal_children() {
    local signal=$1 pid
    for pid in "${children[@]}"; do
        # Also handle a stop arriving before setsid has created the group.
        kill -s "$signal" -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
}
children_running() {
    local pid
    for pid in "${children[@]}"; do
        if kill -0 -- "-$pid" 2>/dev/null || kill -0 "$pid" 2>/dev/null; then return 0; fi
    done
    return 1
}
wait_for_children() {
    local attempt
    for ((attempt=0; attempt<$1; attempt++)); do
        if ! children_running; then return 0; fi
        sleep 0.1
    done
    ! children_running
}
cleanup() {
    trap - EXIT TERM INT
    if ((${#children[@]})); then
        echo 'Stopping the workcell...'
        signal_children INT
        if ! wait_for_children 80; then
            signal_children TERM
            if ! wait_for_children 50; then signal_children KILL; fi
        fi
        local pid
        for pid in "${children[@]}"; do wait "$pid" 2>/dev/null || true; done
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ ${ENABLE_DESKTOP:-0} == 1 ]]; then
    setsid /usr/local/bin/desktop.sh > /tmp/desktop.log 2>&1 &
    children+=("$!")
    for ((attempt=0; attempt<100; attempt++)); do
        if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then break; fi
        sleep 0.1
    done
    xdpyinfo -display "$DISPLAY" >/dev/null
    echo 'Desktop: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
fi

echo "Starting the workcell on ROS domain $ROS_DOMAIN_ID."
echo 'Lichtblick Desktop: connect via Foxglove WebSocket at ws://localhost:8765'
echo 'Press Ctrl-C to stop the simulation and its ROS/Gazebo processes.'
echo 'Or run ./scripts/stop.sh from another terminal.'
setsid ros2 launch arm_bringup demo.launch.py \
    robot:="${ROBOT_MODEL:-xarm6}" mode:="${SIM_MODE:-gazebo}" \
    world:="${TEST_WORLD:-workcell}" gui:="${GUI:-false}" rviz:="${RVIZ:-false}" \
    ousters:="${ENABLE_OUSTERS:-true}" perception:="${ENABLE_PERCEPTION:-false}" \
    cameras:="${ENABLE_CAMERAS:-true}" gripper:="${ENABLE_GRIPPER:-true}" \
    wrist_imu:="${WRIST_IMU:-false}" imu_noise:="${IMU_NOISE:-true}" \
    lidar_hz:="${LIDAR_HZ:-10.0}" "${launch_args[@]}" &
children+=("$!")
wait -n "${children[@]}"
