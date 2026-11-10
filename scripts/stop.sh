#!/usr/bin/env bash
# Stop this checkout's launcher, including its ROS and Gazebo descendants.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
case "${1:-}" in
    -h|--help)
        echo 'Usage: bash scripts/stop.sh'
        echo 'Stop this workspace simulation from another terminal.'
        exit 0 ;;
    '') ;;
    *) echo "Unexpected argument: $1" >&2; exit 2 ;;
esac

exec python3 - "$PWD" <<'PY'
import os
from pathlib import Path
import signal
import sys
import time

workspace = Path(sys.argv[1]).resolve()
launcher = workspace / "scripts/start.sh"


def snapshot():
    processes = {}
    for path in Path("/proc").iterdir():
        if not path.name.isdigit():
            continue
        try:
            fields = (path / "stat").read_text().rsplit(")", 1)[1].split()
            processes[int(path.name)] = {
                "parent": int(fields[1]), "state": fields[0], "start": fields[19],
            }
        except (OSError, ValueError, IndexError):
            continue
    return processes


processes = snapshot()
candidates = set()
for pid in processes:
    try:
        proc = Path("/proc") / str(pid)
        if Path(os.readlink(proc / "cwd")).resolve() != workspace:
            continue
        argv = [os.fsdecode(arg) for arg in (proc / "cmdline").read_bytes().split(b"\0") if arg]
        is_launcher = (len(argv) > 1 and Path(argv[0]).name == "bash"
                       and (workspace / argv[1]).resolve() == launcher)
        is_ros_launch = any(Path(argv[i]).name == "ros2"
                            and argv[i + 1:i + 4] == ["launch", "arm_bringup", "demo.launch.py"]
                            for i in range(len(argv) - 3))
        if is_launcher or is_ros_launch:
            candidates.add(pid)
    except (OSError, ValueError, IndexError):
        continue

if not candidates:
    print("No simulation is running for this workspace.")
    sys.exit(0)

# Signal the outer launcher once; it owns the process-group cleanup.
roots = set(candidates)
for pid in candidates:
    parent = processes[pid]["parent"]
    while parent in processes:
        if parent in candidates:
            roots.discard(pid)
            break
        parent = processes[parent]["parent"]

owned = {pid: processes[pid]["start"] for pid in candidates}


def remaining():
    current = snapshot()
    # Include children spawned while startup or shutdown is in progress.
    while True:
        children = {pid: p["start"] for pid, p in current.items()
                    if pid not in owned and p["parent"] in owned
                    and current.get(p["parent"], {}).get("start") == owned[p["parent"]]}
        if not children:
            break
        owned.update(children)
    return {pid for pid, start in owned.items()
            if pid in current and current[pid]["start"] == start and current[pid]["state"] != "Z"}


def send(pids, sig):
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


remaining()  # Record all descendants before any parent can exit.
print("Stopping this workspace's simulation...", flush=True)
send(roots, signal.SIGINT)
for sig, seconds in [(None, 18), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
    if sig is not None:
        send(remaining(), sig)
    deadline = time.monotonic() + seconds
    while remaining() and time.monotonic() < deadline:
        time.sleep(0.2)
    if not remaining():
        print("Simulation stopped.")
        sys.exit(0)
raise SystemExit(f"Simulation processes still running: {sorted(remaining())}")
PY
