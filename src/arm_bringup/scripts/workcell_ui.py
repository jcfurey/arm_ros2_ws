#!/usr/bin/env python3
"""Lichtblick-friendly world/robot markers and named-pose motion commands."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from arm_bringup.scene import load_world, numbers, quaternion


class WorkcellUI(Node):
    def __init__(self):
        super().__init__("workcell_ui")
        self.scene = load_world(self.declare_parameter("world_file", "").value)
        robot = self.declare_parameter("robot", "xarm6").value
        config = Path(get_package_share_directory("arm_moveit_config")) / "config"
        if robot == "xarm6":
            config /= "xarm6"
        semantic = ET.parse(config / "arm.srdf").getroot()
        self.states = {state.attrib["name"]: [(j.attrib["name"], float(j.attrib["value"]))
                                             for j in state.findall("joint")]
                       for state in semantic.findall("group_state") if state.attrib["group"] == "arm"}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.world_pub = self.create_publisher(MarkerArray, "/scene/markers", qos)
        self.robot_pub = self.create_publisher(MarkerArray, "/robot/markers", qos)
        self.status_pub = self.create_publisher(String, "/motion/status", qos)
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.create_subscription(String, "/motion/command", self.command, 10)
        self.busy = False
        self.goal = None
        self.stop_requested = False
        self.robot_markers = MarkerArray()
        for parameter in ["robot_description", "sensor_description"]:
            description = self.declare_parameter(parameter, "").value
            if description:
                self.add_urdf_markers(description)
        self.create_timer(1.0, self.publish_markers)
        self.publish_markers()
        self.status("idle", poses=list(self.states), commands_topic="/motion/command")

    def status(self, state, **fields):
        self.status_pub.publish(String(data=json.dumps({"state": state, **fields})))

    def add_urdf_markers(self, description):
        root = ET.fromstring(description)
        materials = {m.attrib["name"]: numbers(m.find("color").attrib["rgba"], 4)
                     for m in root.findall("material") if m.find("color") is not None}
        for link in root.findall("link"):
            for visual in link.findall("visual"):
                marker = Marker(ns="robot", id=len(self.robot_markers.markers), action=Marker.ADD)
                marker.header.frame_id = link.attrib["name"]
                marker.frame_locked = True
                origin = visual.find("origin")
                xyz = numbers(origin.get("xyz", "0 0 0"), 3) if origin is not None else (0.0,)*3
                rpy = numbers(origin.get("rpy", "0 0 0"), 3) if origin is not None else (0.0,)*3
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
                marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = quaternion(rpy)
                material = visual.find("material")
                color = (0.7, 0.7, 0.7, 1.0)
                if material is not None:
                    color = numbers(material.find("color").attrib["rgba"], 4) if material.find("color") is not None else materials.get(material.get("name"), color)
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
                geometry = visual.find("geometry")[0]
                if geometry.tag == "mesh":
                    marker.type = Marker.MESH_RESOURCE
                    marker.mesh_resource = geometry.attrib["filename"]
                    marker.scale.x, marker.scale.y, marker.scale.z = numbers(geometry.get("scale", "1 1 1"), 3)
                elif geometry.tag == "box":
                    marker.type = Marker.CUBE
                    marker.scale.x, marker.scale.y, marker.scale.z = numbers(geometry.attrib["size"], 3)
                elif geometry.tag in ["cylinder", "sphere"]:
                    marker.type = Marker.CYLINDER if geometry.tag == "cylinder" else Marker.SPHERE
                    marker.scale.x = marker.scale.y = 2 * float(geometry.attrib["radius"])
                    marker.scale.z = float(geometry.attrib["length"]) if geometry.tag == "cylinder" else marker.scale.x
                else:
                    raise ValueError(f"Unsupported URDF visual: {geometry.tag}")
                self.robot_markers.markers.append(marker)

    def publish_markers(self):
        markers = MarkerArray()
        for index, item in enumerate(self.scene.objects):
            marker = Marker(ns="world", id=index, action=Marker.ADD)
            marker.header.frame_id = "world"
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = item.pose[:3]
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = item.pose[3:]
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = item.color
            if item.shape == "box":
                marker.type = Marker.CUBE
                marker.scale.x, marker.scale.y, marker.scale.z = item.dimensions
            elif item.shape == "cylinder":
                marker.type = Marker.CYLINDER
                marker.scale.x = marker.scale.y = 2 * item.dimensions[1]
                marker.scale.z = item.dimensions[0]
            elif item.shape == "sphere":
                marker.type = Marker.SPHERE
                marker.scale.x = marker.scale.y = marker.scale.z = 2 * item.dimensions[0]
            else:
                marker.type = Marker.CUBE
                marker.scale.x = marker.scale.y = 10.0
                marker.scale.z = 0.02
                marker.pose.position.z -= 0.01
            markers.markers.append(marker)
        self.world_pub.publish(markers)
        self.robot_pub.publish(self.robot_markers)

    def command(self, message):
        name = message.data.strip()
        if name == "stop":
            self.stop_requested = True
            if self.goal is not None:
                self.goal.cancel_goal_async()
            self.status("cancelling" if self.busy else "idle")
            return
        if self.busy or name not in self.states or not self.move.server_is_ready():
            self.status("rejected", pose=name, reason="busy, unknown pose, or MoveIt unavailable")
            return
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 3
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.start_state.is_diff = True
        constraints = Constraints(name=name)
        constraints.joint_constraints = [JointConstraint(joint_name=joint, position=value,
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0) for joint, value in self.states[name]]
        goal.request.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.busy, self.stop_requested = True, False
        self.status("planning", pose=name)

        def accepted(future):
            self.goal = future.result()
            if not self.goal.accepted:
                self.busy, self.goal = False, None
                self.status("rejected", pose=name)
                return
            self.status("executing", pose=name)
            if self.stop_requested:
                self.goal.cancel_goal_async()
            self.goal.get_result_async().add_done_callback(finished)

        def finished(future):
            code = future.result().result.error_code.val
            self.busy, self.goal = False, None
            self.status("succeeded" if code == 1 else "stopped" if self.stop_requested else "failed",
                        pose=name, error_code=code)

        self.move.send_goal_async(goal).add_done_callback(accepted)


def main():
    rclpy.init()
    node = WorkcellUI()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
