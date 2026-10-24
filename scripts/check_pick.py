#!/usr/bin/env python3
"""Pick a part through /motion/command and verify Gazebo moved it to its drop frame."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

from arm_bringup.scene import load_world


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="workcell")
    parser.add_argument("--part", default="pick_cube")
    args = parser.parse_args()
    scene = load_world(Path(get_package_share_directory("arm_bringup")) / "worlds" / (args.world + ".sdf"))
    drop = scene.frames[f"{args.part}_drop"][:2]
    home = next(item.pose[:2] for item in scene.movable if item.model == args.part)
    rclpy.init()
    node = Node("pick_check")
    status, pose = {}, {}
    node.create_subscription(String, "/motion/status", lambda m: status.update(json.loads(m.data)),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(PoseStamped, f"/parts/{args.part}/pose",
                             lambda m: pose.update(p=m.pose.position), qos_profile_sensor_data)
    command = node.create_publisher(String, "/motion/command", 10)

    def until(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"Timed out waiting for {description}")

    try:
        busy = ("planning", "executing", "cancelling")
        until(lambda: status.get("state") not in (None, *busy) and "p" in pose, 120,
              "a ready workcell UI and the part pose")
        if "picks" in status and args.part not in status["picks"]:
            raise RuntimeError(f"{args.part} is not a pick command: {status.get('picks')}")
        start = pose["p"]
        # Same rule as workcell_ui: a part at its SDF pose goes to its drop
        # frame, and anywhere else goes back home.
        target = drop if math.dist((start.x, start.y), home) < 0.03 else home
        # A transient status may describe the previous run of this same pick.
        # Wait for a new status after publishing instead of accepting that result.
        status.clear()
        command.publish(String(data=args.part))
        until(lambda: status.get("pose") == args.part, 10, "the pick to start")
        steps = []
        def finished():
            if status.get("step") and (not steps or steps[-1] != status["step"]):
                steps.append(status["step"])
                print(f"  {status['step']}", flush=True)
            return status.get("state") in ("succeeded", "failed", "stopped", "rejected")
        until(finished, 300, "the pick to finish")
        if status["state"] != "succeeded":
            raise RuntimeError(f"pick {status['state']} at {status.get('step')} (MoveIt code {status.get('error_code')})")
        time.sleep(1.0)
        until(lambda: True, 1.0, "the part to settle")
        end = pose["p"]
        error = math.dist((end.x, end.y), target)
        # Welded grasps land within 1-2 mm; held by friction alone
        # (weld_grasps:=false) a part can creep in the hand by ~2 cm.
        if error > 0.03 or abs(end.z - start.z) > 0.01:
            raise RuntimeError(f"{args.part} ended at {end.x:.3f} {end.y:.3f} {end.z:.3f}, "
                               f"not upright on the bench at {target}")
        print(f"PASS: picked {args.part} at ({start.x:.3f}, {start.y:.3f}) and placed it "
              f"{error * 1000:.0f} mm from ({target[0]:.3f}, {target[1]:.3f})", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
