#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
cuda=false
args=()
for argument in "$@"; do
    case "$argument" in
        --cuda) cuda=true ;;
        -h|--help)
            echo 'Usage: bash scripts/build.sh [--cuda] [colcon build arguments...]'
            echo 'CUDA selection: CUDACXX, CUDAToolkit_ROOT, /usr/local/cuda, installed toolkits, then PATH.'
            exit 0 ;;
        *) args+=("$argument") ;;
    esac
done
if ! command -v colcon >/dev/null; then
    echo 'colcon is missing. Use Docker, or source ROS and install colcon first.' >&2
    exit 1
fi

cuda_meta=()
if [[ "$cuda" == true ]]; then
    # Test C++20 and cuRAND headers with the actual host compiler, rather than
    # selecting an old nvcc merely because it happens to come first on PATH.
    probe_dir=$(mktemp -d /tmp/arm-cuda-toolchain.XXXXXX)
    trap 'rm -rf "$probe_dir"' EXIT
    cat > "$probe_dir/probe.cu" <<'EOF'
#include <cuda_runtime.h>
#include <curand_kernel.h>
__global__ void probe() {}
EOF
    cuda_nvcc=
    try_nvcc() {
        local candidate
        candidate=$(command -v "$1") || return 1
        candidate=$(readlink -f "$candidate")
        [[ -x "$candidate" ]] || return 1
        if "$candidate" -std=c++20 -x cu -c "$probe_dir/probe.cu" -o "$probe_dir/probe.o" \
                > "$probe_dir/probe.log" 2>&1; then
            cuda_nvcc=$candidate
            return 0
        fi
        return 1
    }
    if [[ -n ${CUDACXX:-} ]]; then
        try_nvcc "$CUDACXX" || true
    elif [[ -n ${CUDAToolkit_ROOT:-} ]]; then
        try_nvcc "$CUDAToolkit_ROOT/bin/nvcc" || true
    else
        try_nvcc /usr/local/cuda/bin/nvcc || true
        if [[ -z "$cuda_nvcc" ]]; then
            while IFS= read -r candidate; do
                if try_nvcc "$candidate"; then break; fi
            done < <(printf '%s\n' /usr/local/cuda-[0-9]*/bin/nvcc | sort -Vr)
        fi
        if [[ -z "$cuda_nvcc" ]]; then try_nvcc nvcc || true; fi
    fi
    if [[ -z "$cuda_nvcc" ]]; then
        echo 'No working CUDA C++20/cuRAND toolchain found. Set CUDACXX to a compatible nvcc.' >&2
        if [[ -f "$probe_dir/probe.log" ]]; then tail -12 "$probe_dir/probe.log" >&2; fi
        exit 1
    fi
    cuda_root=$(dirname "$(dirname "$cuda_nvcc")")
    if [[ -n ${CUDAToolkit_ROOT:-} && $(readlink -f "$CUDAToolkit_ROOT") != "$cuda_root" ]]; then
        echo "CUDACXX and CUDAToolkit_ROOT select different toolkits: $cuda_nvcc vs $CUDAToolkit_ROOT" >&2
        exit 1
    fi
    echo "CUDA compiler: $cuda_nvcc"
    echo "CUDA toolkit: $cuda_root (native GPU architecture)"
    cuda_meta=(--metas "$PWD/colcon.meta" "$PWD/colcon.cuda.meta")
    # Keep caller-supplied CMake arguments in the same argument group.
    cmake_position=${#args[@]}
    for ((index=0; index<${#args[@]}; index++)); do
        if [[ ${args[index]} == --cmake-args ]]; then cmake_position=$index; break; fi
    done
    if ((cmake_position == ${#args[@]})); then args+=(--cmake-args); fi
    args=("${args[@]:0:cmake_position+1}" "-DCMAKE_CUDA_COMPILER=$cuda_nvcc" \
          "-DCUDAToolkit_ROOT=$cuda_root" "${args[@]:cmake_position+1}")
fi

# A CUDA rebuild can reuse initialized simulation sources without changing
# the full xArm checkout's sparse-checkout selection or any submodule commits.
initialized=true
for source in src/external/xarm_ros2/xarm_description/package.xml \
              src/external/gz_sensors_ouster/package.xml \
              src/external/gz_sensors_ouster/third_party/ouster_sim_core/CMakeLists.txt \
              src/external/ouster-ros/ouster-ros/package.xml \
              src/external/ouster-ros/ouster-ros/ouster-sdk/CMakeLists.txt; do
    if [[ ! -f "$source" ]]; then initialized=false; fi
done
if [[ "$cuda" != true || "$initialized" != true ]]; then python3 scripts/bootstrap.py; fi
# Recursive clones include the hardware SDK; the simulated arm does not use it.
colcon build --symlink-install --event-handlers console_direct+ --packages-ignore xarm_cplus_sdk \
    "${cuda_meta[@]}" "${args[@]}"
