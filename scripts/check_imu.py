#!/usr/bin/env python3
"""Check the simulated D435i IMU against the arm's motion in TF.

Moves the arm through the workcell UI's "sweep" and back home, and compares
/camera/wrist/imu with the motion of its frame that robot_state_publisher
derives from the joint states: angular velocity from the change in
orientation, and specific force from gravity and the frame's acceleration.
Those differences average the motion over 40 ms windows, so the IMU is
averaged over the same windows before comparing. As on a real unit, the
turn-on biases are estimated at rest first. Gravity dominates the specific
force, so the frame's own acceleration is also checked with gravity removed,
averaged over 0.1 s.
Needs the IMU (WRIST_IMU=true); subscribing starts its bridge.
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import tf2_ros

FRAME = "camera_imu_optical_frame"
GRAVITY = np.array([0.0, 0.0, -9.81])


def matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def smooth(values, kernel):
    kernel = np.asarray(kernel, dtype=np.float64) / np.sum(kernel)
    padded = np.pad(values, ((len(kernel) // 2, len(kernel) // 2), (0, 0)), mode="edge")
    return np.stack([np.convolve(column, kernel, mode="valid") for column in padded.T], axis=1)


def rotation_vector(rotation):
    angle = math.acos(max(-1.0, min(1.0, (np.trace(rotation) - 1) / 2)))
    axis = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]])
    return axis * (0.5 if angle < 1e-9 else angle / (2 * math.sin(angle)))


class ImuCheck(Node):
    def __init__(self):
        super().__init__("imu_check")
        self.samples = []
        self.frame_ids = set()
        self.state = None
        self.tf = tf2_ros.Buffer(cache_time=Duration(seconds=120))
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(Imu, "/camera/wrist/imu", self.on_imu, 500)
        self.create_subscription(String, "/motion/status", self.on_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.command = self.create_publisher(String, "/motion/command", 10)

    def on_imu(self, message):
        self.frame_ids.add(message.header.frame_id)
        w, a = message.angular_velocity, message.linear_acceleration
        self.samples.append((message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                             w.x, w.y, w.z, a.x, a.y, a.z))

    def on_status(self, message):
        self.state = json.loads(message.data).get("state")

    def spin_for(self, seconds, until=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not until():
            rclpy.spin_once(self, timeout_sec=0.02)
        return until()

    def move(self, pose, timeout):
        self.state = None
        self.command.publish(String(data=pose))
        if not self.spin_for(timeout, lambda: self.state in ("succeeded", "failed", "rejected", "stopped")):
            raise SystemExit(f"FAIL: {pose} did not finish within {timeout:.0f} s")
        if self.state != "succeeded":
            raise SystemExit(f"FAIL: {pose} {self.state}")

    def pose(self, stamp):
        transform = self.tf.lookup_transform("world", FRAME, Time(nanoseconds=int(stamp))).transform
        t = transform.translation
        return np.array([t.x, t.y, t.z]), matrix(transform.rotation)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rest", type=float, default=3.0, help="Seconds at rest before moving.")
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    rclpy.init()
    node = ImuCheck()
    try:
        if not node.spin_for(20, lambda: len(node.samples) > 100 and node.state is not None):
            raise SystemExit(f"FAIL: {len(node.samples)} IMU messages and no /motion/status in 20 s "
                             "(is WRIST_IMU=true?)")
        node.spin_for(args.rest)
        moving_from = node.samples[-1][0]
        node.move("sweep", args.timeout)
        node.move("home", args.timeout)
        node.spin_for(1.0)
        data = np.array(node.samples, dtype=np.float64)
        stamps = data[:, 0]
        if node.frame_ids != {FRAME}:
            raise SystemExit(f"FAIL: IMU frame_id {sorted(node.frame_ids)}, expected {FRAME}")
        period = np.median(np.diff(stamps)) * 1e-9
        print(f"{len(data)} samples at {1 / period:.0f} Hz (sim time) in {FRAME}", flush=True)
        if abs(period - 0.005) > 0.0005:
            raise SystemExit(f"FAIL: IMU period {period * 1000:.2f} ms, expected 5 ms (200 Hz)")

        # Reference motion from TF: orientation change over +-h, a box average of
        # the angular velocity, and the second difference over +-2h, a triangular
        # average of the acceleration. Average the 5 ms IMU samples the same way.
        h = 0.02e9
        averaged = np.concatenate([smooth(data[:, 1:4], np.ones(9)),
                                   smooth(data[:, 4:7], np.r_[1:9, 7:0:-1])], axis=1)
        rows = []
        for row, imu in zip(data, averaged):
            stamp = row[0]
            try:
                (p0, _), (p1, r1), (p2, _) = (node.pose(stamp - 2 * h), node.pose(stamp), node.pose(stamp + 2 * h))
                _, ra = node.pose(stamp - h)
                _, rb = node.pose(stamp + h)
            except tf2_ros.TransformException:
                continue
            omega = rotation_vector(ra.T @ rb) / (2 * h * 1e-9)
            accel = (p2 - 2 * p1 + p0) / (2 * h * 1e-9) ** 2
            rows.append((imu, omega, r1.T @ accel, -r1.T @ GRAVITY, stamp < moving_from))
        measured, omega_ref, motion_ref, gravity_ref = (np.array([r[i] for r in rows]) for i in range(4))
        at_rest = np.array([r[4] for r in rows])
        moving = ~at_rest
        if at_rest.sum() < 100 or moving.sum() < 400:
            raise SystemExit(f"FAIL: {at_rest.sum()} samples at rest and {moving.sum()} moving have TF")
        # Turn-on biases, estimated at rest as on a real unit.
        gyro_bias = measured[at_rest, :3].mean(0)
        accel_bias = (measured[at_rest, 3:] - gravity_ref[at_rest]).mean(0)
        force = measured[moving, 3:] - accel_bias
        # The frame's own acceleration is small beside gravity; compare it over 0.1 s.
        motion = smooth(measured[:, 3:] - accel_bias - gravity_ref, np.ones(21))[moving]
        failures = []
        # name, measured, reference, unit, minimum correlation, allowed scale error
        for name, value, truth, unit, min_correlation, max_scale_error in [
                ("gyro", measured[moving, :3] - gyro_bias, omega_ref[moving], "rad/s", 0.998, 0.03),
                ("specific force", force, motion_ref[moving] + gravity_ref[moving], "m/s^2", 0.998, 0.03),
                ("motion acceleration", motion, smooth(motion_ref, np.ones(21))[moving], "m/s^2", 0.85, 0.2)]:
            error = value - truth
            correlation = np.corrcoef(value.ravel(), truth.ravel())[0, 1]
            scale = np.dot(value.ravel(), truth.ravel()) / np.dot(truth.ravel(), truth.ravel())
            print(f"{name}: reference RMS {np.sqrt((truth ** 2).mean()):.4f} {unit}, error RMS "
                  f"{np.sqrt((error ** 2).mean()):.4f} (max {np.abs(error).max():.4f}), correlation "
                  f"{correlation:.4f}, scale {scale:.3f}", flush=True)
            if not correlation >= min_correlation or not abs(scale - 1) <= max_scale_error:
                failures.append(name)
        rest_force = measured[at_rest, 3:].mean(0)
        rest_gravity = gravity_ref[at_rest].mean(0)
        tilt = math.degrees(math.acos(np.clip(np.dot(rest_force, rest_gravity) / (
            np.linalg.norm(rest_force) * np.linalg.norm(rest_gravity)), -1, 1)))
        print(f"at rest: gyro bias {np.round(gyro_bias, 4)} rad/s, accel bias {np.round(accel_bias, 3)} m/s^2 "
              f"({tilt:.1f} deg from gravity in TF)", flush=True)
        # A turn-on bias tilts the reading by a few degrees; tens mean a frame error.
        if tilt > 20:
            failures.append("gravity direction")
        if failures:
            raise SystemExit("FAIL: IMU does not follow the arm's motion: " + ", ".join(failures))
        print(f"PASS: wrist IMU follows the arm through sweep over {moving.sum()} samples", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
