#!/usr/bin/env python3
"""Apply the selected SDF world's collision geometry to MoveIt.

Static models become fixed obstacles. Movable parts are added at their initial
poses (workcell_ui keeps them in step with Gazebo), and the gripper's fingers
may touch them, so closing on a part is a valid goal rather than a collision.
"""
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, ObjectColor, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import Plane, SolidPrimitive

from arm_bringup.scene import load_world

PRIMITIVES = {"box": SolidPrimitive.BOX, "sphere": SolidPrimitive.SPHERE, "cylinder": SolidPrimitive.CYLINDER}


def to_pose(values):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[:3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = values[3:]
    return pose


def collision_object(item, movable):
    obj = CollisionObject(id=item.id, operation=CollisionObject.ADD)
    obj.header.frame_id = "world"
    if item.shape == "plane":
        obj.planes = [Plane(coef=list(item.dimensions))]
        obj.plane_poses = [to_pose(item.pose)]
        return obj
    obj.primitives = [SolidPrimitive(type=PRIMITIVES[item.shape], dimensions=list(item.dimensions))]
    if movable:
        # Movable parts are moved by their object pose (CollisionObject.MOVE).
        obj.pose = to_pose(item.pose)
        obj.primitive_poses = [to_pose((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))]
    else:
        obj.primitive_poses = [to_pose(item.pose)]
    return obj


def group_links(semantic, group):
    root = ET.fromstring(semantic) if semantic else None
    if root is None:
        return []
    for element in root.findall("group"):
        if element.get("name") == group:
            return [link.attrib["name"] for link in element.findall("link")]
    return []


def allow(matrix, first, second):
    """Allow a pair in an AllowedCollisionMatrix message, adding names as needed."""
    for name in (first, second):
        if name not in matrix.entry_names:
            matrix.entry_names.append(name)
            for entry in matrix.entry_values:
                entry.enabled.append(False)
            matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
    i, j = matrix.entry_names.index(first), matrix.entry_names.index(second)
    matrix.entry_values[i].enabled[j] = matrix.entry_values[j].enabled[i] = True


def call(node, client, request, description):
    if not client.wait_for_service(timeout_sec=90):
        raise RuntimeError(f"MoveIt {description} service did not appear")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"MoveIt {description} did not respond")
    return future.result()


def main():
    rclpy.init()
    node = Node("scene_loader")
    try:
        scene = load_world(node.declare_parameter("world_file", "").value)
        semantic = node.declare_parameter("robot_description_semantic", "").value
        request = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        request.scene.robot_state.is_diff = True
        for movable, items in [(False, scene.objects), (True, scene.movable)]:
            for item in items:
                request.scene.world.collision_objects.append(collision_object(item, movable))
                color = ObjectColor(id=item.id)
                color.color.r, color.color.g, color.color.b, color.color.a = item.color
                request.scene.object_colors.append(color)
        apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not call(node, apply, request, "apply_planning_scene").success:
            raise RuntimeError("MoveIt did not accept the SDF collision scene")
        # Only the pads touch a grasped part; the gripper body stays collision-checked.
        fingers = [link for link in group_links(semantic, "gripper") if link.endswith("finger")]
        if scene.movable and fingers:
            query = GetPlanningScene.Request()
            query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            current = call(node, node.create_client(GetPlanningScene, "/get_planning_scene"), query,
                           "get_planning_scene")
            matrix = current.scene.allowed_collision_matrix
            for item in scene.movable:
                for link in fingers:
                    allow(matrix, item.id, link)
            update = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
            update.scene.robot_state.is_diff = True
            update.scene.allowed_collision_matrix = matrix
            if not call(node, apply, update, "apply_planning_scene").success:
                raise RuntimeError("MoveIt did not accept the gripper/part collision allowances")
        node.get_logger().info(
            f"Loaded {len(scene.objects)} static and {len(scene.movable)} movable SDF collision objects into MoveIt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
