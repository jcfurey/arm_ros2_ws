#!/usr/bin/env python3
"""Check live controllers and plan, execute, and verify two simulated arm motions."""
import argparse
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


class StackCheck(Node):
    def __init__(self, mode):
        super().__init__("arm_stack_check")
        self.mode = mode
        self.positions = {}
        self.clock_samples = []
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.move = ActionClient(self, MoveGroup, "/move_action")

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))

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

    def run(self):
        joints = [f"joint_{i}" for i in range(1, 7)]
        self.until(lambda: all(j in self.positions for j in joints), 120, "six joint states")
        if self.mode == "gazebo":
            self.until(lambda: len(self.clock_samples) == 2 and self.clock_samples[1] > self.clock_samples[0],
                       30, "advancing Gazebo clock")
        if not self.controllers.wait_for_service(timeout_sec=30):
            raise RuntimeError("Controller manager is unavailable")
        # Bringup starts MoveIt after both controllers finish activating.
        if not self.move.wait_for_server(timeout_sec=60):
            raise RuntimeError("MoveIt move_action is unavailable")
        response = self.result(self.controllers.call_async(ListControllers.Request()), 15, "controller list")
        states = {c.name: c.state for c in response.controller}
        for name in ["joint_state_broadcaster", "arm_controller"]:
            if states.get(name) != "active":
                raise RuntimeError(f"{name} is not active: {states}")
        for name, target in [("ready", [0.3, -0.6, 1.2, 0.0, -0.6, 0.0]), ("home", [0.0] * 6)]:
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
            request.workspace_parameters.max_corner.z = 1.5
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
        print(f"PASS: {self.mode} controllers, joint states, and MoveIt execution", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["gazebo", "mock"], default="gazebo")
    args = parser.parse_args()
    rclpy.init()
    node = StackCheck(args.mode)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
