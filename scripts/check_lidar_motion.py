#!/usr/bin/env python3
"""Check that both Ousters see a moving object where it was at each point's timestamp.

Spawns a 0.3 m box without collision in the workcell, drives it back and
forth along y at 1 m/s through both sensors' view, and records its pose every
physics step. A cloud point on a face across the motion lies off that face,
at the point's own time (header stamp plus its t field), by the box's
velocity times how much later than that time the sensor captured the box.
That lag is reported per scan, beside the lag of one capture at the end of
the scan, and should stay within half the plugin's capture spacing plus a
physics step (4 ms at 10 Hz). The box is removed again at the end.
"""
import argparse
import subprocess
import threading
import time

import numpy as np
import rclpy
from gz.msgs.pose_pb2 import Pose
from gz.msgs.twist_pb2 import Twist
from gz.transport import Node as GzNode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import tf2_ros

WORLD, BOX = "arm_world", "lidar_motion_box"
SENSORS = ["ouster_front", "ouster_rear"]
HALF, SPEED = 0.15, 1.0
# Clear of the arm, the bench and the rear screen, in view of both sensors.
X, Z, Y_FROM, Y_TO = 0.45, 1.25, -1.0, 0.4
# Bullet applies gravity whatever the link's <gravity> says, and VelocityControl
# resets the velocity before every 1 ms step: rising by one step of gravity holds z.
LIFT = 9.81 * 0.001
SDF = f"""<sdf version="1.10"><model name="{BOX}">
<link name="link"><gravity>false</gravity>
<inertial><mass>1</mass><inertia><ixx>0.015</ixx><iyy>0.015</iyy><izz>0.015</izz></inertia></inertial>
<visual name="visual"><geometry><box><size>{2 * HALF} {2 * HALF} {2 * HALF}</size></box></geometry></visual>
</link>
<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
<initial_linear>0 {SPEED} {LIFT}</initial_linear></plugin>
<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
<publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose>
<static_publisher>false</static_publisher></plugin>
</model></sdf>"""


def matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def field(message, name, dtype):
    offset = next(f.offset for f in message.fields if f.name == name)
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(-1, message.point_step)
    width = np.dtype(dtype).itemsize
    return raw[:, offset:offset + width].copy().view(dtype).ravel()


class MotionCheck(Node):
    def __init__(self):
        super().__init__("lidar_motion_check")
        self.lock = threading.Lock()
        self.truth = []                                # (sim ns, x, y, z)
        self.scans = {sensor: [] for sensor in SENSORS}  # (stamp ns, columns, world xyz, t ns)
        self.transforms = {}
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        for sensor in SENSORS:
            self.create_subscription(PointCloud2, f"/sensor/lidar/{sensor}/points",
                                     lambda m, s=sensor: self.on_cloud(s, m),
                                     QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.gz = GzNode()
        self.gz.subscribe(Pose, f"/{BOX}/pose", self.on_pose)
        self.command = self.gz.advertise(f"/model/{BOX}/cmd_vel", Twist)

    def on_pose(self, message):
        with self.lock:
            self.truth.append((message.header.stamp.sec * 10**9 + message.header.stamp.nsec,
                               message.position.x, message.position.y, message.position.z))

    def on_cloud(self, sensor, message):
        if sensor not in self.transforms:
            try:
                transform = self.tf.lookup_transform("world", message.header.frame_id, Time()).transform
            except tf2_ros.TransformException:
                return
            t = transform.translation
            self.transforms[sensor] = (matrix(transform.rotation), np.array([t.x, t.y, t.z]))
        rotation, translation = self.transforms[sensor]
        valid = field(message, "range", np.uint32) > 0
        xyz = np.stack([field(message, axis, np.float32) for axis in "xyz"], axis=1)[valid]
        world = xyz.astype(np.float64) @ rotation.T + translation
        near = ((np.abs(world[:, 0] - X) < 0.4) & (np.abs(world[:, 2] - Z) < 0.4)
                & (world[:, 1] > Y_FROM - 0.5) & (world[:, 1] < Y_TO + 0.5))
        offsets = field(message, "t", np.uint32)[valid][near].astype(np.int64)
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        with self.lock:
            self.scans[sensor].append((stamp, message.width, world[near], offsets))

    def spin_for(self, seconds, until=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not until():
            rclpy.spin_once(self, timeout_sec=0.01)
        return until()

    def drive(self, passes):
        """Run the box back and forth between Y_FROM and Y_TO."""
        with self.lock:
            self.truth.clear()
            for scans in self.scans.values():
                scans.clear()
        direction, turns = 1.0, 0
        while turns < 2 * passes:
            self.spin_for(0.005)
            with self.lock:
                y = self.truth[-1][2] if self.truth else Y_FROM
            if (direction > 0 and y >= Y_TO) or (direction < 0 and y <= Y_FROM):
                direction = -direction
                twist = Twist()
                twist.linear.y, twist.linear.z = direction * SPEED, LIFT
                self.command.publish(twist)
                turns += 1


def lags(truth, scans):
    """Per scan: median lag of the box capture behind the point times, and that of one capture at scan end (ms)."""
    velocity = np.diff(truth[:, 2]) / np.diff(truth[:, 0]) * 1e9
    turns = truth[1:-1, 0][np.sign(velocity[1:]) != np.sign(velocity[:-1])]
    period = np.median(np.diff([scan[0] for scan in scans]))
    measured, at_end, points = [], [], []
    for stamp, columns, world, offsets in scans:
        start, end = stamp - 5e6, stamp + period + 5e6
        if start < truth[0, 0] or end > truth[-1, 0] or np.any((turns >= start) & (turns <= end)):
            continue  # needs the box's poses throughout, at one velocity
        when = (stamp + offsets).astype(np.float64)
        centre = np.stack([np.interp(when, truth[:, 0], truth[:, k]) for k in (1, 2, 3)], axis=1)
        q = world - centre
        # Faces across the motion: inside the box's cross-section, either side.
        face = ((np.abs(q[:, 0]) < HALF - 0.05) & (np.abs(q[:, 2]) < HALF - 0.05)
                & (np.abs(q[:, 1]) > 0.02) & (np.abs(q[:, 1]) < HALF + 0.14))
        if face.sum() < 30:
            continue
        speed = np.median(velocity[(truth[1:, 0] >= stamp) & (truth[1:, 0] <= stamp + period)])
        measured.append(np.median((q[face, 1] - np.sign(q[face, 1]) * HALF) / speed * 1e3))
        at_end.append(np.median(stamp + (columns - 1) * period / columns - when[face]) / 1e6)
        points.append(face.sum())
    return np.array(measured), np.array(at_end), points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--passes", type=int, default=8, help="Round trips of the box.")
    parser.add_argument("--max-mean-ms", type=float, default=2.0)
    parser.add_argument("--max-scan-ms", type=float, default=5.0,
                        help="Allowed lag on a single scan; 5%% of scans may exceed it.")
    args = parser.parse_args()
    rclpy.init()
    node = MotionCheck()
    subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", WORLD, "-name", BOX, "-string", SDF,
                    "-x", str(X), "-y", str(Y_FROM), "-z", str(Z)], check=True, capture_output=True)
    try:
        if not node.spin_for(30, lambda: len(node.truth) > 100 and len(node.transforms) == len(SENSORS)):
            raise SystemExit(f"FAIL: {len(node.truth)} box poses and clouds from {sorted(node.transforms)} "
                             "in 30 s (are ENABLE_OUSTERS and the workcell running?)")
        node.drive(args.passes)
    finally:
        subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/remove", "--reqtype", "gz.msgs.Entity",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "2000",
                        "--req", f'name: "{BOX}" type: MODEL'], capture_output=True)
    with node.lock:
        truth = np.array(sorted(node.truth), dtype=np.float64)
        scans = {sensor: list(node.scans[sensor]) for sensor in SENSORS}
    node.destroy_node()
    rclpy.shutdown()
    failures = []
    for sensor in SENSORS:
        measured, at_end, points = lags(truth, scans[sensor])
        if len(measured) < 20:
            failures.append(f"{sensor} saw the box's moving faces in {len(measured)} scans")
            continue
        print(f"{sensor}: {len(measured)} scans, {int(np.median(points))} points on the moving faces per scan; "
              f"capture lag behind point time ms: mean {measured.mean():+.2f}, "
              f"range {measured.min():+.2f}..{measured.max():+.2f} "
              f"(one capture at scan end: mean {at_end.mean():+.2f}, up to {at_end.max():+.2f})", flush=True)
        if abs(measured.mean()) > args.max_mean_ms or np.mean(np.abs(measured) > args.max_scan_ms) > 0.05:
            failures.append(sensor)
    if failures:
        raise SystemExit("FAIL: " + ", ".join(failures))
    print("PASS: both Ousters see the moving box where it was at each point's timestamp", flush=True)


if __name__ == "__main__":
    main()
