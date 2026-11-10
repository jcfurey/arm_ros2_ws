"""Plan, preview and execute controls for the built-in Lichtblick panels."""
import copy
import math
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetStateValidity
from octomap_msgs.msg import OctomapWithPose
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from arm_bringup.pick import NO_IK_SOLUTION, SUCCESS, ik_request, joint_goal, pose_goal
from arm_bringup.scene import compose

# A joint has moved once it is this far (rad) from where it last counted as
# moving. The slowest motion the panel offers, 1% speed, covers it in under
# 0.1 s; a resting simulated arm drifts by about 1e-11.
MOVING = 1e-3
# Seconds allowed for the collision-checked reachability query of a tool
# target, and for the query that tells out of reach from in collision.
REACH_TIMEOUT, DIAGNOSE_TIMEOUT = 0.5, 0.25


def scale_value(text):
    value = float(text)
    if not math.isfinite(value) or not 0.01 <= value <= 1.0:
        raise ValueError("Speed must be between 0.01 and 1.0 (1–100%)")
    return value


def joint_targets(names, positions, expected, limits):
    if len(names) != len(positions) or len(set(names)) != len(names) or set(names) != set(expected):
        raise ValueError("Specify each arm joint exactly once, with one position in radians")
    targets = dict(zip(names, positions))
    for name, value in targets.items():
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        low, high = limits[name]
        if not low <= value <= high:
            raise ValueError(f"{name} is outside its limits [{low:.3f}, {high:.3f}] rad")
    return [(name, targets[name]) for name in expected]


def normalize_pose(message):
    if message.header.frame_id not in ("", "world"):
        raise ValueError("Tool targets must use the world frame")
    target = copy.deepcopy(message.pose)
    p, q = target.position, target.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError("Tool position and quaternion must be finite")
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("Tool orientation needs a nonzero quaternion")
    q.x, q.y, q.z, q.w = (v / norm for v in (q.x, q.y, q.z, q.w))
    return target


def matches_start(expected, current, tolerance=0.01):
    return bool(expected) and all(math.isfinite(value) and name in current and math.isfinite(current[name])
                                  and abs(current[name] - value) <= tolerance
                                  for name, value in expected.items())


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def scene_signature(source):
    scene = copy.deepcopy(source)
    scene.robot_state.joint_state = JointState()
    # A live occupancy map (make perception) changes with every lidar update,
    # so no saved path would outlive a comparison that included it. Execute
    # checks every waypoint against the scene as it is then, map included.
    scene.world.octomap = OctomapWithPose()
    frames = {obj.header.frame_id for obj in scene.world.collision_objects}
    frames.discard("world")
    frames.discard("")
    # MoveIt also exports unrelated TF frames, including parts that jitter by
    # micrometres in contact. Only transforms referenced by collision geometry
    # can change the meaning of this saved path.
    scene.fixed_frame_transforms = [transform for transform in scene.fixed_frame_transforms
                                    if transform.child_frame_id in frames]
    for obj in scene.world.collision_objects:
        obj.header.stamp.sec = obj.header.stamp.nanosec = 0
    for obj in scene.robot_state.attached_collision_objects:
        obj.object.header.stamp.sec = obj.object.header.stamp.nanosec = 0
    for transform in scene.fixed_frame_transforms:
        transform.header.stamp.sec = transform.header.stamp.nanosec = 0
    return serialize_message(scene)


class LatestJointStates:
    """The newest joint positions, read when asked for.

    An rclpy callback for every /joint_states message costs about 9% of a core
    at the broadcaster's 100 Hz. This subscription belongs to a node that no
    executor spins, so messages only queue in the middleware and read() takes
    what is waiting, as rclpy.wait_for_message does.
    """

    def __init__(self, ui, topic="/joint_states"):
        self.node = rclpy.create_node(f"{ui.get_name()}_joint_states", context=ui.context,
                                      use_global_arguments=False, enable_rosout=False,
                                      start_parameter_services=False)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Resolved by the UI's node, so a remapping of its /joint_states applies.
        self.subscription = self.node.create_subscription(JointState, ui.resolve_topic_name(topic),
                                                          lambda _message: None, qos)
        self.positions = {}

    def read(self):
        handle = self.subscription.handle
        with handle:
            while (taken := handle.take_message(JointState, False)) is not None:
                self.positions.update(zip(taken[0].name, taken[0].position))
        return self.positions

    def destroy(self):
        self.node.destroy_node()


class MotionPanel:
    def __init__(self, ui, semantic, description):
        self.ui = ui
        self.tip = semantic.find("group[@name='arm']/chain").attrib["tip_link"]
        self.joints = [name for name, _ in next(joints for group, joints in ui.states.values() if group == "arm")]
        robot = ET.fromstring(description)
        self.links = {link.attrib["name"] for link in robot.findall("link")}
        self.limits = {}
        for joint in robot.findall("joint"):
            if joint.attrib["name"] in self.joints:
                limit = joint.find("limit")
                self.limits[joint.attrib["name"]] = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
        self.velocity_scale = self.acceleration_scale = 0.25
        self.joint_states = LatestJointStates(ui)
        self.positions = self.joint_states.positions
        self.moved_from = {}
        self.last_motion = time.monotonic()
        self.generation = 0
        self.saved = None
        self.invalid_reason = ""
        self.pose_sent = -math.inf
        self.tool_pose = None
        self.scene_sent = -math.inf
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.preview_pub = ui.create_publisher(MarkerArray, "/motion/preview", qos)
        self.pose_pub = ui.create_publisher(PoseStamped, "/motion/tool_pose", 2)
        self.validity = ui.create_client(GetStateValidity, "/check_state_validity")
        self.get_scene = ui.create_client(GetPlanningScene, "/get_planning_scene")
        ui.create_subscription(PoseStamped, "/motion/goal_pose", self.pose_target, 2)
        ui.create_subscription(JointState, "/motion/goal_joints", self.joints_target, 2)
        ui.create_timer(0.5, self.refresh)
        self.clear()

    def ready(self):
        return (self.ui.move.server_is_ready() and self.ui.fk.service_is_ready() and self.ui.ik.service_is_ready()
                and self.get_scene.service_is_ready() and self.validity.service_is_ready())

    def fields(self):
        return {"busy": self.ui.busy, "can_execute": self.saved is not None and not self.ui.busy,
                "can_plan": self.ready() and all(name in self.positions for name in self.joints),
                "velocity_scale": self.velocity_scale, "acceleration_scale": self.acceleration_scale,
                "tool_link": self.tip, "poses": list(self.ui.states), "picks": self.ui.picks,
                "arm_poses": [name for name, (group, _) in self.ui.states.items() if group == "arm"],
                "joints": self.joints, "joint_limits": self.limits,
                # NaN is not JSON; a joint without a reading is left out.
                "joint_positions": {name: value for name, value in self.positions.items() if math.isfinite(value)},
                "tool_pose": self.tool_pose,
                "held_parts": [name for name, part in self.ui.parts.items() if part.get("attached")],
                "planned_pose": self.saved["name"] if self.saved else "",
                "plan_duration": self.saved["duration"] if self.saved else 0.0,
                "plan_points": len(self.saved["trajectory"].joint_trajectory.points) if self.saved else 0,
                "invalid_reason": self.invalid_reason}

    def clear(self, reason=""):
        self.generation += 1
        self.saved = None
        self.invalid_reason = reason
        self.preview_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def discard(self, reason):
        """Clear for another operation; `reason` only explains a preview that existed."""
        self.clear(reason if self.saved else "")

    def invalidate(self, reason):
        if self.saved:
            self.clear(reason)
            self.ui.status("invalidated", reason=reason)
        else:
            self.generation += 1

    def poll_joints(self):
        """Take the joint states that arrived since the last call (the UI calls this at 10 Hz)."""
        positions = self.joint_states.read()
        if any(math.isfinite(value) and not abs(value - self.moved_from.get(name, math.inf)) < MOVING
               for name, value in positions.items()):
            self.moved_from, self.last_motion = dict(positions), time.monotonic()
        if self.saved and not self.ui.busy and not matches_start(self.saved["start"], positions):
            self.invalidate("Robot moved away from the planned start; plan again")

    def check_scene(self):
        """Clear a saved preview whose collision objects changed, from perception or other ROS clients."""
        now = time.monotonic()
        # An unanswered query is given up after 5 s, so a restarted MoveIt is asked again.
        if not self.saved or self.ui.busy or now - self.scene_sent < 5.0:
            return
        saved, token = self.saved, self.generation
        self.scene_sent = now

        def checked(signature):
            self.scene_sent = -math.inf
            # An answer for a preview that has since been replaced says nothing about the new one.
            if self.saved is saved and token == self.generation and signature != saved["scene"]:
                self.invalidate("Collision scene changed; plan again")

        def failed(reason):
            self.scene_sent = -math.inf
            if self.saved is saved and token == self.generation:
                self.invalidate(reason)

        self.snapshot(saved["name"], checked, on_error=failed, guarded=False)

    def reject(self, reason, name=""):
        self.ui.status("rejected", pose=name, reason=reason)

    def available(self):
        if self.ui.busy:
            self.reject("A motion or plan is already in progress")
            return False
        if not self.ready():
            self.reject("MoveIt is not ready")
            return False
        self.poll_joints()
        if not all(name in self.positions for name in self.joints):
            self.reject("Waiting for all arm joint states")
            return False
        return True

    def command(self, name):
        if name == "execute":
            self.execute_saved()
        elif name == "clear":
            if not self.ui.busy:
                self.clear()
                self.ui.status("idle")
            else:
                self.reject("Stop the active operation before clearing its preview")
        elif name.startswith("speed "):
            if not self.ui.busy:
                try:
                    value = scale_value(name[6:])
                except ValueError as error:
                    self.reject(str(error))
                    return True
                self.discard("Speed changed; plan again")
                self.velocity_scale = self.acceleration_scale = value
                self.ui.status("idle")
            else:
                self.reject("Stop the active operation before changing speed")
        elif name.startswith("plan "):
            target = name[5:].strip()
            if target not in self.ui.states or self.ui.states[target][0] != "arm":
                self.reject("Choose an available arm pose", target)
            elif self.available():
                joints = self.ui.states[target][1]
                self.plan(target, [joint_goal(joints)], self.collision_free(joints))
        elif name.startswith("pick_to "):
            values = name.split()
            if len(values) != 3 or values[1] not in self.ui.picks or values[2] not in ("auto", "home", "drop"):
                self.reject("Choose an available part and destination")
            elif self.available():
                held = [model for model, part in self.ui.parts.items() if part.get("attached")]
                if held:
                    self.reject(f"Holding {held[0]}; open the gripper to release it first")
                else:
                    self.discard("A pick started; plan again")
                    self.ui.pick(values[1], values[1], destination=values[2])
        else:
            return False
        return True

    def pose_target(self, message):
        if not self.available():
            return
        try:
            target = normalize_pose(message)
        except ValueError as error:
            self.reject(str(error), "tool target")
            return
        self.plan("tool target", [pose_goal(self.tip, target)], self.reachable(target))

    def joints_target(self, message):
        if not self.available():
            return
        try:
            targets = joint_targets(message.name, message.position, self.joints, self.limits)
        except ValueError as error:
            self.reject(str(error), "joint target")
            return
        self.plan("joint target", [joint_goal(targets)], self.collision_free(targets))

    def active(self, token):
        return token == self.generation and not self.ui.stop_requested

    def failed(self, name, reason, code=MoveItErrorCodes.FAILURE):
        self.clear(reason)
        self.ui.finish(name, code, reason=reason)

    def response(self, future, name):
        try:
            return future.result()
        except Exception as error:
            self.failed(name, f"MoveIt request failed: {error}")
            return None

    def reachable(self, target):
        """A check for plan(): name the reason at once when no arm configuration reaches a tool pose.

        The planner would otherwise sample for its whole time allowance and
        report only that it found no plan.
        """
        def query(collisions, timeout):
            request = ik_request("arm", self.tip, target, RobotState(is_diff=True))
            request.ik_request.avoid_collisions = collisions
            request.ik_request.timeout.sec, request.ik_request.timeout.nanosec = 0, int(timeout * 1e9)
            return request

        def check(name, token, proceed):
            def solved(future):
                result = self.response(future, name)
                if result is None:
                    return
                if result.error_code.val == SUCCESS or not self.active(token):
                    proceed()  # which reports a stop
                    return
                self.ui.ik.call_async(query(False, DIAGNOSE_TIMEOUT)).add_done_callback(self.ui.guard(diagnosed))

            def diagnosed(future):
                result = self.response(future, name)
                if result is None:
                    return
                if result.error_code.val == SUCCESS:
                    self.failed(name, "The tool target is reachable only in collision with the scene or the robot",
                                MoveItErrorCodes.GOAL_IN_COLLISION)
                else:
                    self.failed(name, "The tool target is out of the arm's reach", NO_IK_SOLUTION)

            self.ui.ik.call_async(query(True, REACH_TIMEOUT)).add_done_callback(self.ui.guard(solved))

        return check

    def collision_free(self, targets):
        """A check for plan(): name the reason at once when a joint goal itself collides."""
        def check(name, token, proceed):
            state = RobotState(is_diff=True)
            state.joint_state = JointState(name=[joint for joint, _ in targets],
                                           position=[float(value) for _, value in targets])

            def valid(future):
                result = self.response(future, name)
                if result is None:
                    return
                if result.valid or not self.active(token):
                    proceed()
                else:
                    self.failed(name, "The target collides with the scene or the robot itself",
                                MoveItErrorCodes.GOAL_IN_COLLISION)

            query = GetStateValidity.Request(robot_state=state, group_name="arm")
            self.validity.call_async(query).add_done_callback(self.ui.guard(valid))

        return check

    def plan(self, name, goals, check=None):
        """Plan to `goals` and preview the path without moving.

        `check(name, token, proceed)` may fail the request before the planner runs.
        """
        self.clear()
        token = self.generation
        self.ui.begin(name)
        self.ui.status("planning", pose=name)
        planning_scene = [None]
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name, request.pipeline_id, request.planner_id = "arm", "ompl", "RRTConnectkConfigDefault"
        request.allowed_planning_time, request.num_planning_attempts = 5.0, 3
        request.max_velocity_scaling_factor = self.velocity_scale
        request.max_acceleration_scaling_factor = self.acceleration_scale
        request.start_state.is_diff = True
        request.goal_constraints = goals
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        def accepted(future):
            handle = self.response(future, name)
            if handle is None:
                return
            self.ui.goal = handle
            if not handle.accepted:
                self.failed(name, "MoveIt rejected the planning request")
                return
            if not self.active(token):
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(self.ui.guard(planned))

        def planned(future):
            wrapped = self.response(future, name)
            if wrapped is None:
                return
            if not self.active(token):
                self.failed(name, "Planning stopped or the collision scene changed; plan again")
                return
            result = wrapped.result
            trajectory = result.planned_trajectory
            if result.error_code.val != SUCCESS or not trajectory.joint_trajectory.points:
                self.failed(name, "MoveIt could not find a collision-free plan", result.error_code.val)
                return
            state = result.trajectory_start.joint_state
            start = {joint: value for joint, value in zip(state.name, state.position) if joint in self.joints}
            if set(start) != set(self.joints) or not matches_start(start, self.joint_states.read()):
                self.failed(name, "Robot moved during planning; plan again")
                return
            last = trajectory.joint_trajectory.points[-1].time_from_start
            saved = {"name": name, "trajectory": trajectory, "state": result.trajectory_start,
                     "start": start, "duration": last.sec + last.nanosec / 1e9,
                     "planning_time": result.planning_time}

            def captured(signature):
                if signature != planning_scene[0]:
                    self.failed(name, "Collision scene changed during planning; plan again")
                else:
                    self.preview({**saved, "scene": signature}, token)

            self.snapshot(name, captured)

        def begin(signature):
            if not self.active(token):
                self.failed(name, "Planning stopped before the request was sent")
                return
            planning_scene[0] = signature
            self.ui.move.send_goal_async(goal).add_done_callback(self.ui.guard(accepted))

        def proceed():
            self.snapshot(name, begin)

        if check:
            check(name, token, proceed)
        else:
            proceed()

    def snapshot(self, name, then, on_error=None, guarded=True):
        """Read the collision scene's signature directly from MoveIt.

        The occupancy map is not requested: a megabyte per query that the
        signature would leave out anyway.
        """
        components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                      | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                      | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                      | PlanningSceneComponents.LINK_PADDING_AND_SCALING
                      | PlanningSceneComponents.TRANSFORMS)
        query = GetPlanningScene.Request(components=PlanningSceneComponents(components=components))

        def received(future):
            try:
                response = future.result()
                if response is None:
                    raise RuntimeError("Empty planning-scene response")
            except Exception as error:
                reason = f"Cannot verify the collision scene: {error}"
                if on_error:
                    on_error(reason)
                else:
                    self.failed(name, reason)
                return
            then(scene_signature(response.scene))

        self.get_scene.call_async(query).add_done_callback(self.ui.guard(received) if guarded else received)

    def state_at(self, saved, point):
        state = copy.deepcopy(saved["state"])
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        values.update(zip(saved["trajectory"].joint_trajectory.joint_names, point.positions))
        state.joint_state = JointState(name=list(values), position=list(values.values()))
        state.is_diff = True
        return state

    def preview(self, saved, token):
        points = saved["trajectory"].joint_trajectory.points
        # At most 40 FK calls, then one for the final ghost. This is work per
        # plan; displaying an idle preview performs no repeated kinematics.
        indices = sorted({round(i * (len(points) - 1) / min(39, len(points) - 1))
                          for i in range(min(40, len(points)))}) if len(points) > 1 else [0]
        path = []

        def next_point(index):
            if not self.active(token):
                self.failed(saved["name"], "Preview stopped or the collision scene changed; plan again")
                return
            if index == len(indices):
                links = sorted({marker.header.frame_id for marker in self.ui.robot_markers.markers} & self.links)
                query = GetPositionFK.Request(fk_link_names=links, robot_state=self.state_at(saved, points[-1]))
                query.header.frame_id = "world"
                self.ui.fk.call_async(query).add_done_callback(
                    self.ui.guard(lambda future: completed(future, links)))
                return
            query = GetPositionFK.Request(fk_link_names=[self.tip], robot_state=self.state_at(saved, points[indices[index]]))
            query.header.frame_id = "world"

            def located(future):
                result = self.response(future, saved["name"])
                if result is None:
                    return
                if result.error_code.val != SUCCESS or not result.pose_stamped:
                    self.failed(saved["name"], "Cannot compute the planned tool path")
                    return
                path.append(result.pose_stamped[0].pose.position)
                next_point(index + 1)

            self.ui.fk.call_async(query).add_done_callback(self.ui.guard(located))

        def completed(future, links):
            result = self.response(future, saved["name"])
            if result is None:
                return
            if not self.active(token) or not matches_start(saved["start"], self.joint_states.read()):
                self.failed(saved["name"], "Robot or collision scene changed; plan again")
                return
            if result.error_code.val != SUCCESS or len(result.pose_stamped) != len(links):
                self.failed(saved["name"], "Cannot compute the goal robot preview")
                return
            transforms = dict(zip(result.fk_link_names, [pose_values(p.pose) for p in result.pose_stamped]))
            markers = [Marker(action=Marker.DELETEALL)]
            line = Marker(ns="planned_tool_path", id=0, type=Marker.LINE_STRIP)
            line.header.frame_id = "world"
            line.pose.orientation.w = 1.0
            line.scale.x = 0.008
            line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.9, 1.0, 1.0
            line.points = path
            if len(path) > 1:  # a plan to where the arm already is has no path to draw
                markers.append(line)
            for original in self.ui.robot_markers.markers:
                if original.header.frame_id not in transforms:
                    continue
                marker = copy.deepcopy(original)
                target = compose(transforms[marker.header.frame_id], pose_values(marker.pose))
                marker.header.frame_id, marker.frame_locked, marker.ns = "world", False, "planned_robot"
                marker.pose.position = Point(x=target[0], y=target[1], z=target[2])
                marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = target[3:]
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.65, 1.0, 0.35
                marker.mesh_use_embedded_materials = False
                markers.append(marker)
            self.saved = saved
            self.preview_pub.publish(MarkerArray(markers=markers))
            self.ui.busy, self.ui.goal = False, None
            self.ui.status("planned", pose=saved["name"], planning_time=saved["planning_time"])

        next_point(0)

    def execute_saved(self):
        if not self.available():
            return
        saved = self.saved
        if saved is None:
            self.reject(self.invalid_reason or "Plan a target before executing")
            return
        if not matches_start(saved["start"], self.positions):
            self.invalidate("Robot moved away from the planned start; plan again")
            return
        if not self.ui.execute.server_is_ready():
            self.reject("MoveIt's execution service is unavailable")
            return
        self.ui.begin(saved["name"])
        self.ui.status("checking", pose=saved["name"])
        token = self.generation
        points = saved["trajectory"].joint_trajectory.points

        def checked(index):
            if not self.active(token) or not matches_start(saved["start"], self.joint_states.read()):
                self.failed(saved["name"], "Robot or collision scene changed before execution; plan again")
                return
            if index == len(points):
                self.snapshot(saved["name"], execute_if_unchanged)
                return
            query = GetStateValidity.Request(robot_state=self.state_at(saved, points[index]), group_name="arm")

            def valid(future):
                result = self.response(future, saved["name"])
                if result is not None:
                    if not result.valid:
                        self.failed(saved["name"], "The preview now collides with the scene; plan again",
                                    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)
                    else:
                        checked(index + 1)

            self.validity.call_async(query).add_done_callback(self.ui.guard(valid))

        def execute_if_unchanged(signature):
            if (signature != saved["scene"] or not self.active(token)
                    or not matches_start(saved["start"], self.joint_states.read())):
                self.failed(saved["name"], "Robot or collision scene changed during validation; plan again")
                return
            self.saved = None  # A preview is single-use, including failed executions.
            future = self.ui.execute.send_goal_async(ExecuteTrajectory.Goal(trajectory=saved["trajectory"]))

            def finished(name, code):
                self.clear()
                self.ui.finish(name, code)

            future.add_done_callback(self.ui.guard(lambda f: self.ui.track(
                saved["name"], f, finished, lambda: self.ui.status("executing", pose=saved["name"]))))

        def initial_scene(signature):
            if signature != saved["scene"]:
                self.failed(saved["name"], "Collision scene changed since planning; plan again")
            else:
                checked(0)

        self.snapshot(saved["name"], initial_scene)

    def refresh(self):
        """Twice a second: re-check a saved preview's scene and, while watched, republish status and tool pose."""
        self.check_scene()
        watching = self.pose_pub.get_subscription_count() or self.ui.status_pub.get_subscription_count()
        if not watching:
            return
        self.ui.publish_status()
        now = time.monotonic()
        # As in check_scene, an unanswered query does not block the next one for ever.
        if now - self.pose_sent < 5.0 or not self.ui.fk.service_is_ready():
            return
        self.pose_sent = now
        query = GetPositionFK.Request(fk_link_names=[self.tip], robot_state=RobotState(is_diff=True))
        query.header.frame_id = "world"

        def located(future):
            self.pose_sent = -math.inf
            try:
                result = future.result()
            except Exception as error:
                self.ui.get_logger().warning(f"Cannot read tool pose: {error}")
                return
            if result and result.error_code.val == SUCCESS and result.pose_stamped:
                self.pose_pub.publish(result.pose_stamped[0])
                self.tool_pose = list(pose_values(result.pose_stamped[0].pose))

        self.ui.fk.call_async(query).add_done_callback(located)
