#!/usr/bin/env python3
"""Measure sensor rates, scan completeness, and the real-time factor of a running stack."""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2

CLOUDS = ["/sensor/lidar/ouster_front/points", "/sensor/lidar/ouster_rear/points"]


def camera_image_topic(topic):
    """Accept the old RealSense CameraInfo arguments, but exercise rendering."""
    for stream, image in [("color", "image_raw"), ("depth", "image_rect_raw")]:
        suffix = f"/{stream}/camera_info"
        if topic.endswith(suffix):
            return topic[:-len(suffix)] + f"/{stream}/{image}"
    if topic.endswith("/camera_info"):
        raise ValueError(f"Specify the Image topic corresponding to {topic}")
    return topic


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class RateProbe(Node):
    def __init__(self, clouds, cameras):
        super().__init__("rate_probe")
        self.samples = {topic: [] for topic in clouds + cameras}
        self.points = {topic: [] for topic in clouds}
        self.clock = []
        for topic in clouds:
            self.create_subscription(PointCloud2, topic, lambda m, t=topic: self.on_cloud(t, m),
                                     QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
        for topic in cameras:
            # CameraInfo alone does not cause Gazebo to render an image. Keep
            # real image subscriptions alive for the entire measurement window.
            self.create_subscription(Image, topic, lambda m, t=topic: self.on_stamp(t, m),
                                     qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)

    def on_stamp(self, topic, message):
        self.samples[topic].append((time.monotonic(), stamp_ns(message)))

    def on_cloud(self, topic, message):
        self.on_stamp(topic, message)
        # Organized clouds always hold width x height slots; count finite
        # returns so packets lost between the plugin and os_cloud show up.
        x = next(field.offset for field in message.fields if field.name == "x")
        data = np.frombuffer(message.data, dtype=np.uint8).reshape(-1, message.point_step)
        xs = data[:, x:x + 4].copy().view("<f4" if not message.is_bigendian else ">f4").ravel()
        self.points[topic].append(int(np.count_nonzero(np.isfinite(xs) & (xs != 0.0))))

    def on_clock(self, message):
        self.clock.append((time.monotonic(), message.clock.sec * 1_000_000_000 + message.clock.nanosec))


def rate(samples):
    if len(samples) < 3:
        return 0.0, 0.0
    wall = (samples[-1][0] - samples[0][0])
    sim = (samples[-1][1] - samples[0][1]) / 1e9
    count = len(samples) - 1
    return count / wall if wall > 0 else 0.0, count / sim if sim > 0 else 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0, help="Measurement window in seconds.")
    parser.add_argument("--warmup", type=float, default=5.0, help="Seconds to discard before measuring.")
    parser.add_argument("--min-hz", type=float, default=0.0,
                        help="Fail unless every cloud reaches this rate in both wall and sim time "
                             "(within 1%%: a window that cuts a scan short reads as 9.97 Hz).")
    parser.add_argument("--min-rtf", type=float, default=0.0, help="Fail below this real-time factor.")
    parser.add_argument("--cameras", nargs="*", default=[],
                        help="Image topics to render and measure. RealSense color/depth CameraInfo "
                             "arguments also work and select their corresponding Image topics.")
    args = parser.parse_args()
    try:
        cameras = list(dict.fromkeys(camera_image_topic(topic) for topic in args.cameras))
    except ValueError as error:
        parser.error(str(error))
    rclpy.init()
    node = RateProbe(CLOUDS, cameras)
    try:
        end = time.monotonic() + args.warmup
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        for values in [*node.samples.values(), *node.points.values(), node.clock]:
            values.clear()
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        failures = []
        wall, sim = rate(node.clock)
        rtf = (node.clock[-1][1] - node.clock[0][1]) / 1e9 / (node.clock[-1][0] - node.clock[0][0]) \
            if len(node.clock) > 2 else 0.0
        print(f"real-time factor: {rtf:.3f}  (/clock {wall:.0f} msgs/s)", flush=True)
        if len(node.clock) < 3 or node.clock[-1][1] <= node.clock[0][1]:
            failures.append("no advancing /clock stream")
        if rtf < args.min_rtf:
            failures.append(f"real-time factor {rtf:.3f} < {args.min_rtf}")
        for topic, samples in node.samples.items():
            wall, sim = rate(samples)
            extra = ""
            if topic in node.points and node.points[topic]:
                sizes = node.points[topic]
                extra = f"  finite returns/scan min={min(sizes)} median={int(np.median(sizes))} max={max(sizes)}"
            print(f"{topic}: {wall:.2f} Hz wall, {sim:.2f} Hz sim, {len(samples)} msgs{extra}", flush=True)
            if len(samples) < 3:
                failures.append(f"{topic} did not deliver enough messages")
            if topic in CLOUDS and min(wall, sim) < 0.99 * args.min_hz:
                failures.append(f"{topic} below {args.min_hz} Hz")
        if failures:
            raise SystemExit("FAIL: " + "; ".join(failures))
        print("PASS: measured rates", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
