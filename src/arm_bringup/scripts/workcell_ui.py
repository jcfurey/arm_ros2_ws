#!/usr/bin/env python3
"""Lichtblick-friendly world/robot markers, named-pose motion commands, and part tracking."""
import json
import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, PlanningScene
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from arm_bringup.pick import (NO_IK_SOLUTION, SUCCESS, PickPlace, cartesian_request, grasp_yaws, half_extents,
                              ik_request, pose)
from arm_bringup.scene import compose, inverse, load_world, numbers, quaternion

PRIMITIVES = {"box": SolidPrimitive.BOX, "sphere": SolidPrimitive.SPHERE, "cylinder": SolidPrimitive.CYLINDER}


def lumped_link(description, link):
    """The link that carries `link` in Gazebo, which merges links joined by fixed joints."""
    parents = {joint.find("child").attrib["link"]: (joint.find("parent").attrib["link"], joint.attrib["type"])
               for joint in ET.fromstring(description).findall("joint")}
    while link in parents and parents[link][1] == "fixed":
        link = parents[link][0]
    return link


def set_pose(target, values):
    target.position.x, target.position.y, target.position.z = values[:3]
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = values[3:]


def shape_marker(marker, shape, dimensions):
    if shape == "box":
        marker.type = Marker.CUBE
        marker.scale.x, marker.scale.y, marker.scale.z = dimensions
    elif shape == "cylinder":
        marker.type = Marker.CYLINDER
        marker.scale.x = marker.scale.y = 2 * dimensions[1]
        marker.scale.z = dimensions[0]
    elif shape == "sphere":
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = 2 * dimensions[0]
    else:
        marker.type = Marker.CUBE
        marker.scale.x = marker.scale.y = 10.0
        marker.scale.z = 0.02
        marker.pose.position.z -= 0.01


class WorkcellUI(Node):
    def __init__(self):
        super().__init__("workcell_ui")
        self.scene = load_world(self.declare_parameter("world_file", "").value)
        semantic = ET.fromstring(self.declare_parameter("robot_description_semantic", "").value)
        # Command name -> (planning group, joint targets); names are unique across groups.
        self.states = {state.attrib["name"]: (state.attrib["group"],
                                              [(j.attrib["name"], float(j.attrib["value"]))
                                               for j in state.findall("joint")])
                       for state in semantic.findall("group_state")}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.world_pub = self.create_publisher(MarkerArray, "/scene/markers", qos)
        self.robot_pub = self.create_publisher(MarkerArray, "/robot/markers", qos)
        self.status_pub = self.create_publisher(String, "/motion/status", qos)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.tf = TransformBroadcaster(self)
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        # The hand's current pose comes from MoveIt when a move needs it; a TF
        # listener here would deserialize every /tf message for that.
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.apply_scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.gripper_links = [link.attrib["name"] for group in semantic.findall("group")
                              if group.attrib["name"] == "gripper" for link in group.findall("link")]
        self.create_subscription(String, "/motion/command", self.command, 10)
        self.busy = False
        self.goal = None
        self.stop_requested = False
        self.robot_markers = MarkerArray()
        for parameter in ["robot_description", "sensor_description"]:
            description = self.declare_parameter(parameter, "").value
            if description:
                self.add_urdf_markers(description)
        # Movable parts: model pose per part, starting from the SDF and then
        # following Gazebo's /parts/<model>/pose stream.
        self.parts = {}
        # Without Gazebo's pose stream (mock mode) the parts stay at their SDF poses.
        self.part_poses = self.declare_parameter("part_poses", False).value
        for item in self.scene.movable:
            self.parts.setdefault(item.model, {"pose": compose(item.pose, inverse(item.model_offset)),
                                               "items": [], "synced": None})["items"].append(item)
        for model in self.parts:
            self.create_subscription(PoseStamped, f"/parts/{model}/pose",
                                     lambda message, model=model: self.on_part(model, message),
                                     qos_profile_sensor_data)
        self.create_timer(0.1, self.sync_parts)
        gripper = {name: dict(joints) for name, (group, joints) in self.states.items() if group == "gripper"}
        bench = [item for item in self.scene.objects if item.id.startswith("sdf/bench_top/")]
        self.picker = None
        if {"open", "close"} <= gripper.keys() and bench:
            self.picker = PickPlace(bench[0].pose[2] + bench[0].dimensions[2] / 2, gripper)
        # "<model>::<link>" of the Gazebo link that carries the tip, to weld a
        # gripped part to (Gazebo's arm_bringup::GraspWeld, via gz_relay).
        self.weld_to = None
        model = self.declare_parameter("weld_model", "").value
        description = self.get_parameter("robot_description").value
        if model and description and self.picker:
            self.weld_to = f"{model}::{lumped_link(description, self.picker.tip)}"
            self.weld_pub = self.create_publisher(String, "/grasp_weld", QoSProfile(depth=10))
        # A part's name as a command picks it and sets it at the world's
        # <part>_drop frame, or back at its SDF pose when already there.
        self.picks = [model for model in self.parts if self.picker and f"{model}_drop" in self.scene.frames]
        self.create_timer(1.0, self.publish_markers)
        self.publish_markers()
        self.status("idle", poses=list(self.states), picks=self.picks, commands_topic="/motion/command")

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
                set_pose(marker.pose, xyz + quaternion(rpy))
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
        for index, item in enumerate(self.scene.objects + self.scene.movable):
            marker = Marker(ns="world", id=index, action=Marker.ADD)
            if item.model in self.parts:
                # Locked to the part's TF frame, so it moves with the part.
                marker.header.frame_id = f"parts/{item.model}"
                marker.frame_locked = True
                set_pose(marker.pose, item.model_offset)
            else:
                marker.header.frame_id = "world"
                set_pose(marker.pose, item.pose)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = item.color
            shape_marker(marker, item.shape, item.dimensions)
            markers.markers.append(marker)
        self.world_pub.publish(markers)
        self.robot_pub.publish(self.robot_markers)
        if not self.part_poses:
            self.broadcast_parts(self.get_clock().now().to_msg())

    def on_part(self, model, message):
        p, q = message.pose.position, message.pose.orientation
        self.parts[model]["pose"] = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if self.parts[model]["synced"] is None:
            self.parts[model]["synced"] = ()
        self.broadcast_parts(message.header.stamp, [model])

    def broadcast_parts(self, stamp, models=None):
        transforms = []
        for model in models or self.parts:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "world"
            transform.child_frame_id = f"parts/{model}"
            pose = self.parts[model]["pose"]
            t, r = transform.transform.translation, transform.transform.rotation
            t.x, t.y, t.z = pose[:3]
            r.x, r.y, r.z, r.w = pose[3:]
            transforms.append(transform)
        self.tf.sendTransform(transforms)

    def sync_parts(self):
        """Move each part's MoveIt collision objects to its Gazebo pose when it changes."""
        diff = PlanningScene(is_diff=True)
        diff.robot_state.is_diff = True
        for part in self.parts.values():
            pose, synced = part["pose"], part["synced"]
            if part.get("attached"):
                continue  # MoveIt carries an attached part with the gripper
            if synced is None or (synced and math.dist(pose[:3], synced[:3]) < 1e-3
                                  and abs(sum(a * b for a, b in zip(pose[3:], synced[3:]))) > 0.99999):
                continue
            part["synced"] = pose
            for item in part["items"]:
                obj = CollisionObject(id=item.id, operation=CollisionObject.MOVE)
                obj.header.frame_id = "world"
                set_pose(obj.pose, compose(pose, item.model_offset))
                diff.world.collision_objects.append(obj)
        if diff.world.collision_objects:
            self.scene_pub.publish(diff)

    def command(self, message):
        name = message.data.strip()
        if name == "stop":
            self.stop_requested = True
            if self.goal is not None:
                self.goal.cancel_goal_async()
            self.status("cancelling" if self.busy else "idle")
            return
        if self.busy or name not in [*self.states, *self.picks] or not self.move.server_is_ready():
            self.status("rejected", pose=name, reason="busy, unknown pose, or MoveIt unavailable")
            return
        held = [model for model, part in self.parts.items() if part.get("attached")]
        if name in self.picks:
            if held:
                self.status("rejected", pose=name, reason=f"holding {held[0]}; command open to release it")
                return
            self.pick(name, name)
            return
        group, joints = self.states[name]
        self.busy, self.stop_requested = True, False
        self.status("planning", pose=name)
        done = self.finish
        if name == "open" and held:
            # Opening lets go of a part that a failed pick left in the hand.
            self.weld(held[0], False)

            def done(_name, code):
                self.attach(held[0], False, lambda _model, detached: self.finish(
                    name, detached if code == SUCCESS else code))
        self.plan_and_execute(name, group, [Constraints(name=name, joint_constraints=[
            JointConstraint(joint_name=joint, position=value, tolerance_above=0.001, tolerance_below=0.001,
                            weight=1.0) for joint, value in joints])], done,
            on_accept=lambda: self.status("executing", pose=name))

    def finish(self, name, code, **fields):
        self.busy, self.goal = False, None
        self.status("succeeded" if code == SUCCESS else "stopped" if self.stop_requested else "failed",
                    pose=name, error_code=code, **fields)

    def pick(self, name, model):
        """Pick a part and set it in its drop zone, or back at its SDF pose."""
        home = compose(self.parts[model]["items"][0].pose, inverse(self.parts[model]["items"][0].model_offset))
        current = self.parts[model]["pose"]
        drop = self.scene.frames[f"{model}_drop"][:2]
        target = drop if math.dist(current[:2], home[:2]) < 0.03 else home[:2]
        items = self.parts[model]["items"]
        steps = self.picker.steps(current, *half_extents(items), target, grasp_yaws(items, current))
        self.busy, self.stop_requested = True, False

        def run(index, previous_code=SUCCESS):
            if previous_code != SUCCESS or self.stop_requested:
                # A stop between steps leaves the last step's success code.
                code = previous_code if previous_code != SUCCESS else -1
                self.finish(name, code, step=steps[index - 1][0] if index else None)
                return
            if index == len(steps):
                self.finish(name, SUCCESS)
                return
            step, kind, goal, ends = steps[index]
            self.status("executing", pose=name, step=step)
            done = lambda _name, code: run(index + 1, code)  # noqa: E731
            if kind == "cartesian":
                self.cartesian_move(step, goal, done)
            elif kind in ("attach", "detach"):
                self.attach(model, kind == "attach", done)
            elif ends:
                self.reachable(kind, goal, ends, lambda goals: self.plan_and_execute(step, kind, goals, done)
                               if goals else done(step, NO_IK_SOLUTION))
            else:
                self.plan_and_execute(step, kind, goal, done)

        run(0)

    def attach(self, model, attach, done):
        """Attach a grasped part to the gripper in MoveIt, or set it back in the world."""
        part = self.parts[model]
        if not attach:
            diff = PlanningScene(is_diff=True)
            diff.robot_state.is_diff = True
            for item in part["items"]:
                # MoveIt puts a removed attached object back into the world where it now is.
                held = AttachedCollisionObject(link_name=self.picker.tip)
                held.object.id, held.object.operation = item.id, CollisionObject.REMOVE
                diff.robot_state.attached_collision_objects.append(held)
            self.weld(model, False)
            self.apply_diff(diff, model, done, lambda: part.update(attached=False, synced=()))
            return
        # Hold the part where Gazebo has it in the hand. MoveIt's own world
        # copy lags Gazebo by up to a sync period and misses settling after a
        # release, and attaching that copy would carry the error into planning.
        part["attached"] = True

        def located(hand):
            if hand is None:
                part["attached"] = False
                done(model, -1)
                return
            in_hand = compose(inverse(hand), part["pose"])
            diff = PlanningScene(is_diff=True)
            diff.robot_state.is_diff = True
            for item in part["items"]:
                held = AttachedCollisionObject(link_name=self.picker.tip, touch_links=self.gripper_links)
                held.object.id, held.object.operation = item.id, CollisionObject.ADD
                held.object.header.frame_id = self.picker.tip
                held.object.primitives = [SolidPrimitive(type=PRIMITIVES[item.shape],
                                                         dimensions=list(item.dimensions))]
                set_pose(held.object.pose, compose(in_hand, item.model_offset))
                held.object.primitive_poses = [Pose(orientation=Quaternion(w=1.0))]
                diff.robot_state.attached_collision_objects.append(held)
            self.apply_diff(diff, model, done, lambda: self.weld(model, True), lambda: part.update(attached=False))

        self.hand_pose(located)

    def weld(self, model, attach):
        """Weld a gripped part to the hand in Gazebo, or release the weld."""
        if self.weld_to:
            self.weld_pub.publish(String(data=f"attach {model} {self.weld_to}" if attach else f"detach {model}"))

    def apply_diff(self, diff, model, done, on_success=None, on_failure=None):
        def applied(future):
            response = future.result()
            ok = response is not None and response.success
            callback = on_success if ok else on_failure
            if callback:
                callback()
            done(model, SUCCESS if ok else -1)

        self.apply_scene.call_async(ApplyPlanningScene.Request(scene=diff)).add_done_callback(applied)

    def hand_pose(self, then):
        """Call `then` with MoveIt's current world pose of the tip as a tuple, or None."""
        query = GetPositionFK.Request(fk_link_names=[self.picker.tip], robot_state=RobotState(is_diff=True))
        query.header.frame_id = "world"

        def located(future):
            response = future.result()
            if response is None or response.error_code.val != SUCCESS or not response.pose_stamped:
                self.get_logger().error(f"MoveIt has no current {self.picker.tip} pose")
                then(None)
                return
            p = response.pose_stamped[0].pose
            then((p.position.x, p.position.y, p.position.z,
                  p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

        self.fk.call_async(query).add_done_callback(located)

    def reachable(self, group, goals, ends, then):
        """Call `then` with the goals whose paired end pose has a collision-free IK solution."""
        clear = [None] * len(goals)

        def checked(index, future):
            response = future.result()
            clear[index] = response is not None and response.error_code.val == SUCCESS
            if None not in clear:
                then([goal for goal, ok in zip(goals, clear) if ok])

        for index, end in enumerate(ends):
            request = ik_request(group, self.picker.tip, end, RobotState(is_diff=True))
            self.ik.call_async(request).add_done_callback(lambda future, index=index: checked(index, future))

    def cartesian_move(self, name, positions, done):
        """Move link_tcp in straight lines through `positions`, keeping its orientation."""
        def located(hand):
            if hand is None:
                done(name, -1)
                return
            waypoints = [pose(xyz, hand[3:]) for xyz in positions]
            request = cartesian_request("arm", self.picker.tip, waypoints, RobotState(is_diff=True))
            self.cartesian.call_async(request).add_done_callback(planned)

        def planned(future):
            response = future.result()
            if response is None or response.error_code.val != SUCCESS or response.fraction < 0.99:
                done(name, response.error_code.val if response and response.error_code.val != SUCCESS else -2)
                return
            sent = self.execute.send_goal_async(ExecuteTrajectory.Goal(trajectory=response.solution))
            sent.add_done_callback(lambda f: self.track(name, f, done))

        self.hand_pose(located)

    def track(self, name, future, done, on_accept=None):
        self.goal = future.result()
        if not self.goal.accepted:
            done(name, -1)
            return
        if on_accept:
            on_accept()
        if self.stop_requested:
            self.goal.cancel_goal_async()
        self.goal.get_result_async().add_done_callback(lambda f: done(name, f.result().result.error_code.val))

    def plan_and_execute(self, name, group, goals, done, on_accept=None, attempts=3):
        """Plan to any one of `goals` (alternative Constraints) and execute.

        A plan that fails before anything moves is planned again, up to
        `attempts` times: OMPL is randomized, and MoveIt rejects a path that
        smoothing pushed into collision instead of planning another.
        """
        def finished(_name, code):
            if (code in (MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.INVALID_MOTION_PLAN)
                    and attempts > 1 and not self.stop_requested):
                self.get_logger().warning(f"{name}: planning failed with code {code}; planning again")
                self.plan_and_execute(name, group, goals, done, on_accept, attempts - 1)
                return
            done(_name, code)

        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 3
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = goals
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.move.send_goal_async(goal).add_done_callback(lambda future: self.track(name, future, finished, on_accept))


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
