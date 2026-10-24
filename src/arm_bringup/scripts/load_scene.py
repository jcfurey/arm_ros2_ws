#!/usr/bin/env python3
"""Apply the selected SDF world's static collision geometry to MoveIt."""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import Plane, SolidPrimitive

from arm_bringup.scene import load_world


def main():
    rclpy.init()
    node = Node("scene_loader")
    try:
        scene = load_world(node.declare_parameter("world_file", "").value)
        request = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        request.scene.robot_state.is_diff = True
        for item in scene.objects:
            obj = CollisionObject(id=item.id, operation=CollisionObject.ADD)
            obj.header.frame_id = "world"
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = item.pose[:3]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = item.pose[3:]
            if item.shape == "plane":
                obj.planes = [Plane(coef=list(item.dimensions))]
                obj.plane_poses = [pose]
            else:
                types = {"box": SolidPrimitive.BOX, "sphere": SolidPrimitive.SPHERE, "cylinder": SolidPrimitive.CYLINDER}
                obj.primitives = [SolidPrimitive(type=types[item.shape], dimensions=list(item.dimensions))]
                obj.primitive_poses = [pose]
            request.scene.world.collision_objects.append(obj)
            color = ObjectColor(id=item.id)
            color.color.r, color.color.g, color.color.b, color.color.a = item.color
            request.scene.object_colors.append(color)
        client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not client.wait_for_service(timeout_sec=90):
            raise RuntimeError("MoveIt apply_planning_scene service did not appear")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        if not future.done() or not future.result().success:
            raise RuntimeError("MoveIt did not accept the SDF collision scene")
        node.get_logger().info(f"Loaded {len(scene.objects)} static SDF collision objects into MoveIt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
