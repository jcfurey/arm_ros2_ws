#!/usr/bin/env python3
"""Check live controllers and plan, execute, and verify two simulated arm motions."""
import argparse
import math
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, PointCloud2
from tf2_ros import Buffer, TransformListener
from arm_bringup.scene import load_world, quaternion


class StackCheck(Node):
    def __init__(self, mode, robot, world, ousters=False):
        super().__init__("arm_stack_check")
        self.mode = mode
        self.robot = robot
        self.world = world
        self.ousters = ousters
        self.cloud_samples = {name: [] for name in ["ouster_front", "ouster_rear"]}
        if ousters:
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self)
            for name in self.cloud_samples:
                self.create_subscription(PointCloud2, f"/sensor/lidar/{name}/points",
                                         lambda message, name=name: self.on_cloud(name, message),
                                         qos_profile_sensor_data)
        self.scene = load_world(Path(get_package_share_directory("arm_bringup")) / "worlds" / (world + ".sdf"))
        config_dir = Path(get_package_share_directory("arm_moveit_config")) / "config"
        if robot == "xarm6":
            config_dir /= "xarm6"
        semantic = ET.parse(config_dir / "arm.srdf").getroot()
        states = {state.attrib["name"]: state for state in semantic.findall("group_state")
                  if state.attrib["group"] == "arm"}
        self.joints = [joint.attrib["name"] for joint in states["home"].findall("joint")]
        self.targets = []
        self.named_states = states
        for name in ["ready", *(["sweep"] if "sweep" in states else []), "home"]:
            values = {joint.attrib["name"]: float(joint.attrib["value"])
                      for joint in states[name].findall("joint")}
            self.targets.append((name, [values[joint] for joint in self.joints]))
        self.positions = {}
        self.clock_samples = []
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.planning_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))

    def on_cloud(self, name, message):
        if message.width == 0 or message.height == 0:
            return
        offsets = {field.name: field.offset for field in message.fields}
        if not {"x", "y", "z"} <= offsets.keys():
            raise RuntimeError(f"{name} cloud has no XYZ fields")
        finite = 0
        for index in range(0, message.width * message.height, max(1, message.width * message.height // 2048)):
            row, column = divmod(index, message.width)
            start = row * message.row_step + column * message.point_step
            values = [struct.unpack_from(('>' if message.is_bigendian else '<') + 'f', message.data,
                                         start + offsets[axis])[0] for axis in 'xyz']
            finite += all(math.isfinite(value) for value in values) and sum(value*value for value in values) > 0.01
        if finite > 20:
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            self.cloud_samples[name].append((stamp, message.header.frame_id, finite))
            self.cloud_samples[name] = self.cloud_samples[name][-3:]

    def check_ousters(self):
        self.until(lambda: all(len({sample[0] for sample in samples}) >= 2
                               for samples in self.cloud_samples.values()),
                   120, "two advancing Ouster point clouds with finite XYZ returns")
        for name, samples in self.cloud_samples.items():
            frame = f"{name}/lidar_frame"
            if samples[-1][1] != frame:
                raise RuntimeError(f"Unexpected cloud frame: {samples[-1][1]}")
            self.until(lambda: self.tf_buffer.can_transform("world", frame, Time()), 15, f"{frame} TF")
            transform = self.tf_buffer.lookup_transform("world", frame, Time()).transform
            expected = self.scene.frames[f"{name}_mount"]
            actual = (transform.translation.x, transform.translation.y, transform.translation.z)
            if any(abs(a-b) > 1e-5 for a,b in zip(actual, expected[:3])):
                raise RuntimeError(f"{name} TF mount differs from SDF")
            q = transform.rotation
            alignment = sum(a*b for a,b in zip((q.x,q.y,q.z,q.w), quaternion(expected[3:])))
            if abs(abs(alignment)-1.0) > 1e-5:
                raise RuntimeError(f"{name} TF orientation differs from SDF")
        print("PASS: two Ouster packet-to-cloud streams, finite returns, advancing stamps, and world TF", flush=True)

    def on_clock(self, message):
        self.clock_samples.append(message.clock.sec * 1_000_000_000 + message.clock.nanosec)
        self.clock_samples = self.clock_samples[-2:]

    def until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f"Timed out waiting for {description}")

    def result(self, future, timeout, description):
        self.until(future.done, timeout, description)
        return future.result()

    def check_scene(self):
        if not self.planning_scene.wait_for_service(timeout_sec=30):
            raise RuntimeError("Planning scene service is unavailable")
        expected = {item.id: item for item in self.scene.objects}
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            query = GetPlanningScene.Request()
            query.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            scene = self.result(self.planning_scene.call_async(query), 15, "SDF planning scene").scene
            objects = {obj.id: obj for obj in scene.world.collision_objects}
            if expected.keys() <= objects.keys():
                break
            time.sleep(0.2)
        else:
            raise RuntimeError(f"Missing SDF collision objects: {expected.keys() - objects.keys()}")
        for name, item in expected.items():
            obj = objects[name]
            # MoveIt can normalize the primitive pose into CollisionObject.pose.
            if item.shape == "plane":
                dimensions = obj.planes[0].coef
            else:
                dimensions = obj.primitives[0].dimensions
            if len(dimensions) != len(item.dimensions) or any(abs(a-b) > 1e-6 for a,b in zip(dimensions, item.dimensions)):
                raise RuntimeError(f"MoveIt geometry differs from SDF: {name}")
        print(f"PASS: {len(expected)} SDF obstacles loaded into MoveIt ({self.world})", flush=True)

    def state_validity(self, target):
        if not self.validity.wait_for_service(timeout_sec=15):
            raise RuntimeError("State validity service is unavailable")
        query = GetStateValidity.Request()
        query.group_name = "arm"
        query.robot_state.joint_state.name = self.joints
        query.robot_state.joint_state.position = target
        return self.result(self.validity.call_async(query), 15, "collision check")

    def run(self):
        joints = self.joints
        self.until(lambda: all(j in self.positions for j in joints), 120, "six joint states")
        if self.mode == "gazebo":
            self.until(lambda: len(self.clock_samples) == 2 and self.clock_samples[1] > self.clock_samples[0],
                       30, "advancing Gazebo clock")
        if not self.controllers.wait_for_service(timeout_sec=30):
            raise RuntimeError("Controller manager is unavailable")
        # Bringup starts MoveIt after both controllers finish activating.
        if not self.move.wait_for_server(timeout_sec=60):
            raise RuntimeError("MoveIt move_action is unavailable")
        self.check_scene()
        if self.ousters:
            self.check_ousters()
        response = self.result(self.controllers.call_async(ListControllers.Request()), 15, "controller list")
        states = {c.name: c.state for c in response.controller}
        for name in ["joint_state_broadcaster", "arm_controller"]:
            if states.get(name) != "active":
                raise RuntimeError(f"{name} is not active: {states}")
        for name, target in self.targets:
            validity = self.state_validity(target)
            if not validity.valid:
                raise RuntimeError(f"{name} is in collision: {[(c.contact_body_1, c.contact_body_2) for c in validity.contacts]}")
            goal = MoveGroup.Goal()
            request = goal.request
            request.group_name = "arm"
            request.pipeline_id = "ompl"
            request.planner_id = "RRTConnectkConfigDefault"
            request.num_planning_attempts = 5
            request.allowed_planning_time = 10.0
            request.max_velocity_scaling_factor = 0.25
            request.max_acceleration_scaling_factor = 0.25
            request.start_state.is_diff = True
            request.workspace_parameters.header.frame_id = "world"
            request.workspace_parameters.min_corner.x = -1.0
            request.workspace_parameters.min_corner.y = -1.0
            request.workspace_parameters.min_corner.z = -0.1
            request.workspace_parameters.max_corner.x = 1.0
            request.workspace_parameters.max_corner.y = 1.0
            request.workspace_parameters.max_corner.z = self.scene.mount[2] + 1.5
            constraint = Constraints(name=name)
            for joint, position in zip(joints, target):
                constraint.joint_constraints.append(JointConstraint(
                    joint_name=joint, position=position, tolerance_above=0.001,
                    tolerance_below=0.001, weight=1.0))
            request.goal_constraints = [constraint]
            goal.planning_options.plan_only = False
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            handle = self.result(self.move.send_goal_async(goal), 15, "MoveIt goal acceptance")
            if not handle.accepted:
                raise RuntimeError(f"MoveIt rejected {name}")
            result = self.result(handle.get_result_async(), 90, f"{name} motion").result
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"{name} failed with MoveIt code {result.error_code.val}")
            self.until(lambda: all(abs(self.positions[j] - p) < 0.03 for j, p in zip(joints, target)),
                       10, f"measured joints at {name}")
            print(f"PASS: planned, executed, and measured {name}", flush=True)
        if self.world == "obstacle_course" and self.robot == "xarm6":
            blocked = {j.attrib["name"]: float(j.attrib["value"])
                       for j in self.named_states["blocked"].findall("joint")}
            target = [blocked[j] for j in joints]
            validity = self.state_validity(target)
            if validity.valid or not any("blocking_post" in c.contact_body_1 or "blocking_post" in c.contact_body_2
                                         for c in validity.contacts):
                raise RuntimeError("Expected the SDF blocking post to invalidate the blocked pose")
            # The last request is the verified home goal; replace only its target and plan without executing.
            for constraint, position in zip(goal.request.goal_constraints[0].joint_constraints, target):
                constraint.position = position
            goal.request.allowed_planning_time = 2.0
            goal.request.num_planning_attempts = 1
            goal.planning_options.plan_only = True
            handle = self.result(self.move.send_goal_async(goal), 15, "blocked goal acceptance")
            if not handle.accepted:
                raise RuntimeError("MoveIt did not accept the planning request for the negative check")
            result = self.result(handle.get_result_async(), 30, "blocked goal rejection").result
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                raise RuntimeError("MoveIt unexpectedly planned a goal inside the blocking post")
            print("PASS: actual SDF blocking post prevents the blocked pose and MoveIt rejects the plan", flush=True)
        print(f"PASS: {self.robot} {self.mode} {self.world} collision-aware execution", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["gazebo", "mock"], default="gazebo")
    parser.add_argument("--robot", choices=["xarm6", "demo"], default="xarm6")
    parser.add_argument("--world", choices=["workcell", "obstacle_course", "empty"], default="workcell")
    parser.add_argument("--ousters", action="store_true", help="Also verify both Ouster point clouds and TF mounts.")
    args = parser.parse_args()
    rclpy.init()
    node = StackCheck(args.mode, args.robot, args.world, args.ousters)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
