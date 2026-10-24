#!/usr/bin/env bash
set -euo pipefail
children=()
cleanup() {
    trap - EXIT TERM INT
    if ((${#children[@]})); then kill "${children[@]}" 2>/dev/null || true; fi
    wait || true
}
trap cleanup EXIT
trap 'exit 0' TERM INT
export XDG_RUNTIME_DIR=/tmp/runtime-ros
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
Xvfb "$DISPLAY" -screen 0 "${DESKTOP_SIZE:-1440x900x24}" -ac -nolisten tcp &
children+=("$!")
for ((attempt=0; attempt<100; attempt++)); do
    if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then break; fi
    sleep 0.1
done
xdpyinfo -display "$DISPLAY" >/dev/null
fluxbox &
children+=("$!")
x11vnc -display "$DISPLAY" -forever -shared -nopw -localhost -rfbport 5900 &
children+=("$!")
websockify --web=/usr/share/novnc/ 6080 localhost:5900 &
children+=("$!")
wait -n "${children[@]}"
