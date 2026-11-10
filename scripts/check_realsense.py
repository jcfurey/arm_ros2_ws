#!/usr/bin/env python3
"""Check a simulated RealSense's depth stream against its ideal depth and Intel's error model.

realsense_depth publishes depth/image_rect_raw (16UC1 millimetres, 0 = no
depth) from Gazebo's ideal depth/image_meters. Frames with the same stamp are
compared: the error must be unbiased, its spread must follow
RMS = z^2 * subpixel / (focal_px * baseline) in each well-populated range bin,
and out-of-range pixels must read 0. Subscribing renders the camera.
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

CAMERAS = {  # baseline (m), depth horizontal field of view (rad), min and max depth (m)
    "wrist": (0.050, 1.5184, 0.17, 10.0),
    "overhead": (0.095, 1.5184, 0.4, 6.0),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=sorted(CAMERAS), default="wrist")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--subpixel", type=float, default=0.08, help="realsense_depth's subpixel RMS.")
    args = parser.parse_args()
    baseline, hfov, min_depth, max_depth = CAMERAS[args.camera]
    rclpy.init()
    node = Node("realsense_check")
    ideal, noisy = {}, {}
    stamp = lambda message: (message.header.stamp.sec, message.header.stamp.nanosec)  # noqa: E731
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, f"/camera/{args.camera}/depth/image_meters",
                             lambda m: ideal.__setitem__(stamp(m), m), qos)
    node.create_subscription(Image, f"/camera/{args.camera}/depth/image_rect_raw",
                             lambda m: noisy.__setitem__(stamp(m), m), qos)
    try:
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        pairs = [(ideal[key], noisy[key]) for key in sorted(set(ideal) & set(noisy))]
        if len(pairs) < 5:
            raise SystemExit(f"FAIL: only {len(pairs)} matched frames ({len(ideal)} ideal, {len(noisy)} noisy)")
        if pairs[0][1].encoding != "16UC1":
            raise SystemExit(f"FAIL: depth/image_rect_raw is {pairs[0][1].encoding}, not 16UC1")
        errors, depths = [], []
        for truth, measured in pairs:
            z = np.frombuffer(truth.data, np.float32).reshape(truth.height, truth.width)
            d = np.frombuffer(measured.data, np.uint16).reshape(measured.height, measured.width) / 1000.0
            outside = ~np.isfinite(z) | (z < min_depth) | (z > max_depth)
            if np.any(d[outside] != 0):
                raise SystemExit("FAIL: pixels outside the depth range are not marked 0")
            inside = ~outside
            errors.append(d[inside] - z[inside])
            depths.append(z[inside])
        error, depth = np.concatenate(errors), np.concatenate(depths)
        focal = 0.5 * pairs[0][0].width / math.tan(0.5 * hfov)
        failures, checked = [], 0
        for low in np.arange(min_depth, max_depth, 0.25):
            band = (depth >= low) & (depth < low + 0.25)
            if band.sum() < 20000:
                continue
            checked += 1
            # Expected spread: the model's RMS over the band's depths, plus 1 mm rounding.
            model = np.sqrt(np.mean((depth[band] ** 2 * args.subpixel / (focal * baseline)) ** 2) + 1e-6 / 12)
            spread, bias = error[band].std(), error[band].mean()
            print(f"z {low:.2f}-{low + 0.25:.2f} m: bias {bias * 1000:+.2f} mm, RMS {spread * 1000:.2f} mm "
                  f"(model {model * 1000:.2f} mm), {band.sum()} px", flush=True)
            if abs(spread / model - 1) > 0.15 or abs(bias) > max(0.0005, 0.1 * model):
                failures.append(f"{low:.2f} m band")
        if not checked:
            raise SystemExit(f"FAIL: too little of the view lies within {min_depth}-{max_depth} m to check; "
                             "point the camera at the scene")
        if failures:
            raise SystemExit("FAIL: depth error does not match the model in the " + ", ".join(failures))
        print(f"PASS: {args.camera} depth is 16UC1 mm with D4xx stereo noise over {len(pairs)} frames", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
