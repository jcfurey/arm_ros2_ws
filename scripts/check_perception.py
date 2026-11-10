#!/usr/bin/env python3
"""Verify that `make perception` maps an unmodelled obstacle and that plans survive the map's updates.

MoveIt leaves out of its occupancy map every lidar point on the robot or on a
collision object it already knows, so a fully modelled workcell maps free
space only. And when its point-cloud updater plugin is missing, MoveIt logs an
error and plans against an empty map. This check therefore puts a box into
Gazebo that MoveIt does not know, requires occupied cells where it stands, and
requires them to clear once it is removed. It plans one motion without moving
the arm; the box has no collision and is removed at the end.
"""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from arm_bringup.scene import load_world

FILTERED = ["/perception/front_filtered", "/perception/rear_filtered"]
BOX, HALF = "perception_check_box", 0.15
# Clear of the arm, the bench and the rear screen, in view of both sensors.
CENTRE = (0.45, -0.3, 1.25)
SDF = f"""<sdf version="1.10"><model name="{BOX}"><static>true</static><link name="link">
<visual name="visual"><geometry><box><size>{2 * HALF} {2 * HALF} {2 * HALF}</size></box></geometry></visual>
</link></model></sdf>"""


def occupied_cells(octomap):
    """Centres (in the map's frame) and sizes of the occupied leaves of MoveIt's full-probability octree."""
    tree = octomap.octomap
    if tree.binary or tree.id != "OcTree":
        raise RuntimeError(f"Expected a full OcTree, got {tree.id} (binary={tree.binary})")
    data = bytes(value & 0xFF for value in tree.data)
    origin = octomap.origin.position
    cells, position = [], 0
    # Depth first: a float32 log-odds, then a byte with one bit per child; the root spans 2^16 cells.
    stack = [(origin.x, origin.y, origin.z, tree.resolution * 2**16)] if data else []
    while stack:
        x, y, z, size = stack.pop()
        log_odds, children = struct.unpack_from("<fB", data, position)
        position += 5
        if not children:
            if log_odds > 0.0:
                cells.append((x, y, z, size))
            continue
        quarter = size / 4
        for child in reversed(range(8)):
            if children >> child & 1:
                stack.append((x + (quarter if child & 1 else -quarter), y + (quarter if child & 2 else -quarter),
                              z + (quarter if child & 4 else -quarter), size / 2))
    return cells


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="workcell")
    parser.add_argument("--min-cells", type=int, default=30,
                        help="Occupied cells the box must produce (it spans about 420 at 4 cm)")
    parser.add_argument("--hold", type=float, default=5.0,
                        help="Seconds a planned preview must stay executable while the map updates")
    args = parser.parse_args()
    world = load_world(Path(get_package_share_directory("arm_bringup")) / "worlds" / (args.world + ".sdf")).name
    rclpy.init()
    node = Node("perception_check")
    filtered, status, sequence = set(), {}, [0]
    for topic in FILTERED:
        node.create_subscription(PointCloud2, topic, lambda _message, topic=topic: filtered.add(topic),
                                 qos_profile_sensor_data)

    def on_status(message):
        status.clear()
        status.update(json.loads(message.data))
        sequence[0] += 1

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/motion/status", on_status, latched)
    commands = node.create_publisher(String, "/motion/command", 10)
    scene = node.create_client(GetPlanningScene, "/get_planning_scene")

    def until(predicate, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"Timed out waiting for {what}")

    def occupancy():
        request = GetPlanningScene.Request(components=PlanningSceneComponents(components=PlanningSceneComponents.OCTOMAP))
        future = scene.call_async(request)
        until(future.done, 20, "MoveIt's planning scene")
        return future.result().scene.world.octomap

    def in_box(octomap):
        """Occupied cells that overlap the box, grown by one cell for the map's quantization."""
        reach = HALF + octomap.octomap.resolution
        return sum(all(abs(cell[axis] - CENTRE[axis]) < reach + cell[3] / 2 for axis in range(3))
                   for cell in occupied_cells(octomap))

    def remove_box():
        subprocess.run(["gz", "service", "-s", f"/world/{world}/remove", "--reqtype", "gz.msgs.Entity",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "2000",
                        "--req", f'name: "{BOX}" type: MODEL'], capture_output=True)

    try:
        until(scene.service_is_ready, 120, "MoveIt")
        until(lambda: set(FILTERED) <= filtered, 60,
              f"self-filtered clouds on {' and '.join(FILTERED)} (is moveit_ros_perception installed?)")
        print("PASS: both occupancy updaters process their clouds", flush=True)

        until(lambda: status.get("can_plan") and commands.get_subscription_count() > 0, 60, "the motion panel")
        before = occupancy()
        marker = sequence[0]
        commands.publish(String(data="plan ready"))
        until(lambda: sequence[0] > marker and status.get("state") in ("planned", "failed", "rejected", "stopped"),
              60, "a plan")
        if status.get("state") != "planned":
            raise RuntimeError(f"Planning against the live occupancy map failed: {status}")
        end = time.monotonic() + args.hold
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not status.get("can_execute"):
                raise RuntimeError(f"The preview was cleared while the occupancy map updated: {status}")
        if occupancy().octomap.data == before.octomap.data:
            raise RuntimeError(f"The occupancy map did not change in {args.hold:.0f} s; have its updaters stopped?")
        commands.publish(String(data="clear"))
        until(lambda: not status.get("can_execute"), 10, "the preview to clear")
        print(f"PASS: a planned preview stayed executable for {args.hold:.0f} s while the occupancy map updated",
              flush=True)

        empty = in_box(before)
        if empty >= args.min_cells / 4:
            raise RuntimeError(f"{empty} cells are already occupied where the test box goes")
        subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", world, "-name", BOX, "-string", SDF,
                        "-x", str(CENTRE[0]), "-y", str(CENTRE[1]), "-z", str(CENTRE[2])],
                       check=True, capture_output=True)
        seen = [0]

        def mapped():
            seen[0] = in_box(occupancy())
            return seen[0] >= args.min_cells

        try:
            until(mapped, 30, "the unmodelled box in the occupancy map")
        except RuntimeError as error:
            raise RuntimeError(f"{error}; last count {seen[0]}") from None
        # Both sensors keep adding cells for a few more scans.
        peak, end = seen[0], time.monotonic() + 3.0
        while time.monotonic() < end:
            peak = max(peak, in_box(occupancy()))
        print(f"PASS: an unmodelled {2 * HALF:.1f} m box fills {peak} occupied cells of the map", flush=True)
        remove_box()
        removed = time.monotonic()

        def cleared():
            seen[0] = in_box(occupancy())
            return seen[0] <= max(5, peak // 10)

        try:
            until(cleared, 40, "the removed box to clear from the occupancy map")
        except RuntimeError as error:
            raise RuntimeError(f"{error}; {seen[0]} of {peak} cells remain") from None
        print(f"PASS: the map cleared {time.monotonic() - removed:.0f} s after the box was removed "
              f"({seen[0]} of {peak} cells remain)", flush=True)
    finally:
        remove_box()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
