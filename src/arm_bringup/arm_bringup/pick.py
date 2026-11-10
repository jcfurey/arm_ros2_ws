"""Top-down pick and place of a movable part through MoveIt.

The gripper's link_tcp z axis is its approach direction and the fingers close
along its y axis. A top-down grasp points z at the floor and turns y to a face
normal of the part, so the pads close on two opposite faces. Two yaws, 180 deg
apart, close a box across its narrow faces; a square box or a cylinder can
be closed from four. Pose goals offer each yaw whose pose at the bottom of
the vertical move is reachable and clear, and the planner takes one of them
(the wrist camera body sticks out 8 cm along link_tcp's x axis and can meet
a wall on the way down, and joint6's +-178 deg range leaves some yaws
unreachable). Cartesian moves keep the hand's actual orientation. The gripper closes to the part's width less a squeeze,
as a real xArm Gripper is commanded, and stalls on the part at its force
limit. Every motion goes
through MoveIt: pose goals for the free-space moves, Cartesian paths for the
vertical approach and lift, and the gripper's named states. Between grasp and
release the part is attached to the gripper in MoveIt's planning scene, so it
moves with the hand while planning and is checked against the world.
"""
import math

from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from shape_msgs.msg import SolidPrimitive

from arm_bringup.scene import multiply

# xArm Gripper four-bar: a pad's inner face sits PAD_GAP(theta) from the
# centre line, where theta is drive_joint (UFACTORY kinematics, finger pad
# 26 mm inside the finger origin): 9 mm + 35.465 cos(theta) - 42.039 sin(theta).
FINGER_REACH = (0.035465, 0.042039)
PAD_OFFSET = 0.035 - 0.026
# Command each pad this far inside the part. gz_ros2_control servos position
# through velocity; aimed at a point the pads can reach by compressing the
# contact, the servo reverses and chatters, so a grasp never reads as a stall.
# Aimed deeper, the drive pushes steadily at its effort limit (the grip force)
# and the controller reports the stall within ~0.4 s.
SQUEEZE = 0.008

# Pointing down, the fingertips sit 10 mm above link_tcp and the pads span
# 10-56 mm above it. Open, the inner knuckles start 50 mm above it; commanded
# to a 40 mm part less the squeeze they swing in to ~40 mm above it over the
# part. A TCP 8 mm above the bench keeps the tips clear, and a tall part is
# taken near its top so the knuckles stay above it (only the pads may touch a
# free part in MoveIt).
TCP_ABOVE_BENCH = 0.008
KNUCKLE_CLEARANCE = 0.035
APPROACH = 0.12


def drive_for_gap(half_gap):
    """drive_joint angle at which each pad is `half_gap` from the centre line."""
    a, b = FINGER_REACH
    reach = math.hypot(a, b)
    ratio = max(-1.0, min(1.0, (half_gap - PAD_OFFSET) / reach))
    return min(0.85, max(0.0, math.acos(ratio) - math.atan2(b, a)))


def half_extents(items):
    """Half the height and the half-width across the gripped faces of an upright part."""
    height = {"box": lambda d: d[2] / 2, "cylinder": lambda d: d[0] / 2, "sphere": lambda d: d[0]}
    width = {"box": lambda d: min(d[0], d[1]) / 2, "cylinder": lambda d: d[1], "sphere": lambda d: d[0]}
    return (max(item.model_offset[2] + height[item.shape](item.dimensions) for item in items),
            max(width[item.shape](item.dimensions) for item in items))


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def top_down(yaw):
    """Quaternion pointing link_tcp z down, with its x axis at `yaw` (y at `yaw` - 90 deg)."""
    half = yaw / 2
    flip = (1.0, 0.0, 0.0, 0.0)  # 180 deg about x
    return multiply((0.0, 0.0, math.sin(half), math.cos(half)), flip)


def grasp_yaws(items, part_pose):
    """link_tcp yaws whose pads close across the part's narrow faces.

    The pads close along link_tcp's y axis, which lies 90 deg from its x axis.
    """
    yaw = yaw_of(part_pose[3:])
    boxes = [item for item in items if item.shape == "box"]
    if boxes and abs(boxes[0].dimensions[0] - boxes[0].dimensions[1]) > 1e-3:
        box = boxes[0]
        normal = yaw + yaw_of(box.model_offset[3:]) + (0.0 if box.dimensions[0] < box.dimensions[1] else math.pi / 2)
        return [normal - math.pi / 2, normal + math.pi / 2]
    return [yaw + k * math.pi / 2 for k in range(4)]


def pose(xyz, q):
    p = Pose()
    p.position.x, p.position.y, p.position.z = xyz
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


def pose_goals(link, targets):
    """Alternative pose goals; MoveIt plans to whichever it can reach."""
    return [pose_goal(link, target) for target in targets]


def pose_goal(link, target, tolerance=0.005, angle_tolerance=0.02):
    constraints = Constraints(name="pose")
    position = PositionConstraint(link_name=link, weight=1.0)
    position.header.frame_id = "world"
    region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance])
    position.constraint_region.primitives = [region]
    position.constraint_region.primitive_poses = [pose((target.position.x, target.position.y, target.position.z),
                                                       (0.0, 0.0, 0.0, 1.0))]
    orientation = OrientationConstraint(link_name=link, orientation=target.orientation, weight=1.0)
    orientation.header.frame_id = "world"
    orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = angle_tolerance
    orientation.absolute_z_axis_tolerance = angle_tolerance
    constraints.position_constraints = [position]
    constraints.orientation_constraints = [orientation]
    return constraints


def joint_goal(joints):
    constraints = Constraints(name="joints")
    constraints.joint_constraints = [JointConstraint(joint_name=j, position=v, tolerance_above=0.001,
                                                     tolerance_below=0.001, weight=1.0) for j, v in joints]
    return constraints


class PickPlace:
    """Plans a pick and place as a list of steps run by a caller-owned executor.

    `move(group, constraints)` must plan and execute through MoveGroup;
    `cartesian(waypoints)` must compute and execute a Cartesian path and
    return the achieved fraction. Both return a MoveIt error code.
    """

    def __init__(self, bench_top, gripper_states, tip="link_tcp"):
        self.bench_top = bench_top
        self.states = gripper_states
        self.tip = tip

    def steps(self, part_pose, half_height, half_width, place_xy, yaws):
        """Steps for an upright part, `half_height` from its centre to its top,
        gripped at any of the hand `yaws` and set down at the same yaws.

        Each step is (name, kind, goal, ends). Kinds: a planning group name
        with a list of alternative goal constraints, "cartesian" with link_tcp
        positions to reach in a straight line at the current orientation, and
        "attach" / "detach" (the part to or from link_tcp in the planning
        scene). For the moves above the part and the place, `ends` holds, per
        alternative, the pose at the bottom of the vertical move that follows:
        the planner checks only the goal itself, so the caller keeps the
        alternatives whose bottom pose has a collision-free IK solution.
        """
        x, y, part_z = part_pose[:3]
        z = max(self.bench_top + TCP_ABOVE_BENCH, part_z + half_height - KNUCKLE_CLEARANCE)
        grasp, place = (x, y, z), (place_xy[0], place_xy[1], z + 0.002)

        def over(xyz):
            """Alternative goals above `xyz`, and the hand's pose at `xyz` for each."""
            raised = (xyz[0], xyz[1], z + APPROACH)
            return (pose_goals(self.tip, [pose(raised, top_down(yaw)) for yaw in yaws]),
                    [pose(xyz, top_down(yaw)) for yaw in yaws])

        grip = {joint: drive_for_gap(half_width - SQUEEZE) for joint in self.states["close"]}
        return [
            ("open", "gripper", [joint_goal(self.states["open"].items())], None),
            ("approach", "arm", *over(grasp)),
            ("descend", "cartesian", [grasp], None),
            ("grasp", "gripper", [joint_goal(grip.items())], None),
            ("attach", "attach", None, None),
            ("lift", "cartesian", [(x, y, z + APPROACH)], None),
            ("carry", "arm", *over(place)),
            ("lower", "cartesian", [place], None),
            ("detach", "detach", None, None),
            ("release", "gripper", [joint_goal(self.states["open"].items())], None),
            ("retreat", "cartesian", [(place[0], place[1], z + APPROACH)], None),
        ]


def cartesian_request(group, tip, waypoints, start_state):
    request = GetCartesianPath.Request()
    request.header.frame_id = "world"
    request.group_name = group
    request.link_name = tip
    request.waypoints = waypoints
    request.max_step = 0.005
    request.avoid_collisions = True
    request.start_state = start_state
    request.max_velocity_scaling_factor = 0.1
    request.max_acceleration_scaling_factor = 0.1
    return request


def ik_request(group, tip, target, start_state):
    """A collision-checked IK query for `tip` at `target` (a world-frame Pose)."""
    request = GetPositionIK.Request()
    request.ik_request.group_name = group
    request.ik_request.ik_link_name = tip
    request.ik_request.robot_state = start_state
    request.ik_request.avoid_collisions = True
    request.ik_request.pose_stamped.header.frame_id = "world"
    request.ik_request.pose_stamped.pose = target
    request.ik_request.timeout.nanosec = 500_000_000
    return request


SUCCESS = MoveItErrorCodes.SUCCESS
NO_IK_SOLUTION = MoveItErrorCodes.NO_IK_SOLUTION
__all__ = ["PickPlace", "cartesian_request", "drive_for_gap", "grasp_yaws", "half_extents", "ik_request", "pose",
           "NO_IK_SOLUTION", "SUCCESS"]
