#!/usr/bin/env python3
"""Exercise the Lichtblick plan/preview/execute protocol against a running workcell.

Moves the simulated arm, cancels a motion, and temporarily adds/removes one
far-away MoveIt collision object. Does not perform a pick.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class Check(Node):
    def __init__(self):
        super().__init__("moveit_panel_check")
        self.status, self.positions, self.markers, self.sequence = {}, {}, [], 0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/motion/status", self.on_status, qos)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, "/motion/preview", lambda m: setattr(self, "markers", m.markers), qos)
        self.commands = self.create_publisher(String, "/motion/command", 10)
        self.poses = self.create_publisher(PoseStamped, "/motion/goal_pose", 2)
        self.joints = self.create_publisher(JointState, "/motion/goal_joints", 2)
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def on_status(self, message):
        self.status = json.loads(message.data)
        self.sequence += 1

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))

    def until(self, predicate, timeout=60):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"Timed out; last status: {self.status}")

    def send(self, command, terminal):
        sequence = self.sequence
        self.commands.publish(String(data=command))
        self.until(lambda: self.sequence > sequence and self.status.get("state") in terminal)
        return self.status

    def plan(self, command):
        result = self.send(command, {"planned", "failed", "rejected", "stopped"})
        if result["state"] != "planned":
            raise RuntimeError(f"Planning failed: {result}")
        if not result["can_execute"] or result["plan_points"] < 1:
            raise RuntimeError(f"Preview is not executable: {result}")

    def change_scene(self, remove=False):
        obj = CollisionObject(id="moveit_panel_check/temporary", operation=CollisionObject.REMOVE if remove else CollisionObject.ADD)
        obj.header.frame_id = "world"
        if not remove:
            obj.pose.position.x, obj.pose.position.z = 10.0, 1.0
            obj.pose.orientation.w = 1.0
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.1, 0.1])]
            obj.primitive_poses = [Pose(orientation=Quaternion(w=1.0))]
        diff = PlanningScene(is_diff=True)
        diff.robot_state.is_diff = True
        diff.world.collision_objects = [obj]
        future = self.scene.call_async(ApplyPlanningScene.Request(scene=diff))
        self.until(future.done)
        if not future.result().success:
            raise RuntimeError("Cannot update the temporary planning obstacle")

    def run(self):
        # The latched status can arrive before this node's first joint state.
        self.until(lambda: self.status.get("can_plan") and self.commands.get_subscription_count() > 0
                   and all(name in self.positions for name in self.status["joints"]))
        joints = self.status["joints"]
        start = {name: self.positions[name] for name in joints}
        self.plan("plan ready")
        self.until(lambda: any(m.ns == "planned_robot" for m in self.markers))
        paths = [m for m in self.markers if m.type == Marker.LINE_STRIP]
        if not paths or len(paths[0].points) < 2 or any(m.header.frame_id != "world" for m in self.markers if m.action != Marker.DELETEALL):
            raise RuntimeError("Preview has no world-frame tool path or goal robot")
        if any(abs(self.positions[name] - start[name]) > 0.005 for name in joints):
            raise RuntimeError("Plan-only command moved the robot")
        print("PASS: plan-only preserves robot position and publishes a tool path and goal robot", flush=True)
        result = self.send("execute", {"succeeded", "failed", "rejected", "stopped"})
        if result["state"] != "succeeded" or result["can_execute"]:
            raise RuntimeError(f"Saved trajectory execution failed: {result}")
        print("PASS: saved plan executes once after collision validation", flush=True)

        self.plan("plan home")
        self.change_scene()
        try:
            self.until(lambda: not self.status.get("can_execute"))
            result = self.send("execute", {"rejected", "failed"})
            if result["busy"]:
                raise RuntimeError("Stale plan started executing")
        finally:
            self.change_scene(remove=True)
        print("PASS: collision-scene changes invalidate the saved plan", flush=True)

        self.commands.publish(String(data="speed 0.1"))
        self.until(lambda: self.status.get("velocity_scale") == 0.1)
        self.commands.publish(String(data="speed nan"))
        self.until(lambda: self.status.get("state") == "rejected")
        if self.status["velocity_scale"] != 0.1:
            raise RuntimeError("Invalid speed changed the settings")
        self.send("speed 0.25", {"idle"})
        if self.status["invalid_reason"]:
            raise RuntimeError(f"A speed change with no preview left a hint behind: {self.status['invalid_reason']}")
        print("PASS: speed settings are applied and malformed input is rejected", flush=True)

        self.joints.publish(JointState(name=["missing"], position=[0.0]))
        self.until(lambda: self.status.get("state") == "rejected" and self.status.get("pose") == "joint target")
        sequence = self.sequence
        targets = [self.positions[name] for name in joints]
        targets[0] += 0.03
        self.joints.publish(JointState(name=joints, position=targets))
        self.until(lambda: self.sequence > sequence and self.status.get("state") in ("planned", "failed", "rejected"))
        if self.status["state"] != "planned":
            raise RuntimeError(f"Joint target failed: {self.status}")
        self.send("clear", {"idle"})
        print("PASS: joint target planning and clear-preview controls", flush=True)

        self.until(lambda: self.status.get("tool_pose") is not None)
        target = list(self.status["tool_pose"])
        target[2] += 0.01
        message = PoseStamped()
        message.header.frame_id = "world"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = target[:3]
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = target[3:]
        sequence = self.sequence
        self.poses.publish(message)
        self.until(lambda: self.sequence > sequence and self.status.get("state") in ("planned", "failed", "rejected"))
        if self.status["state"] != "planned":
            raise RuntimeError(f"Tool target failed: {self.status}")
        self.send("clear", {"idle"})
        self.send("execute", {"rejected"})
        print("PASS: Cartesian tool target plans without moving; cleared plans cannot execute", flush=True)

        # The planner alone would sample for its whole 5 s and report only that it found no plan.
        message.pose.position.x += 3.0
        sequence, sent = self.sequence, time.monotonic()
        self.poses.publish(message)
        self.until(lambda: self.sequence > sequence and self.status.get("state") in ("planned", "failed", "rejected"))
        took = time.monotonic() - sent
        if self.status["state"] != "failed" or "out of the arm's reach" not in self.status.get("reason", "") or took > 3.0:
            raise RuntimeError(f"An unreachable tool target was not refused at once ({took:.1f} s): {self.status}")
        print(f"PASS: an unreachable tool target is refused with its reason in {took:.1f} s", flush=True)

        self.plan("plan home")
        self.send("execute", {"executing", "failed", "rejected"})
        if self.status["state"] != "executing":
            raise RuntimeError(f"Could not begin cancellation check: {self.status}")
        result = self.send("stop", {"stopped", "failed"})
        if result["busy"] or result["can_execute"]:
            raise RuntimeError(f"Stop left an operation or executable preview behind: {result}")
        if max(abs(self.positions[name]) for name in joints[:3]) < 0.02:
            raise RuntimeError("The arm reached home instead of stopping during the trajectory")
        result = self.send("home", {"succeeded", "failed", "rejected"})
        if result["state"] != "succeeded":
            raise RuntimeError(f"Could not restore home: {result}")
        print("PASS: stop cancels execution; legacy named-pose commands still work", flush=True)


def main():
    rclpy.init()
    node = Check()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
