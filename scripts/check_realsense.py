#!/usr/bin/env python3
"""Check a simulated RealSense's depth stream against its ideal depth and Intel's error model.

realsense_depth publishes depth/image_rect_raw (16UC1 millimetres, 0 = no
depth) from Gazebo's ideal depth/image_meters. Frames with the same stamp are
compared: the error must be unbiased, its spread must follow
RMS = z^2 * subpixel / (focal_px * baseline) in each well-populated range bin,
and out-of-range pixels must read 0. The depth cloud, transformed into the
image's optical frame, must also agree with the ideal depth and CameraInfo.
Subscribing renders the camera.
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener

CAMERAS = {  # baseline (m), depth horizontal field of view (rad), min and max depth (m)
    "wrist": (0.050, 1.5184, 0.17, 10.0),
    "overhead": (0.095, 1.5184, 0.4, 6.0),
}


def check_cloud(cloud, image, info, transform):
    """Compare organized XYZ, through its declared TF frame, with depth pixels."""
    if (cloud.height, cloud.width) != (image.height, image.width):
        raise SystemExit("FAIL: cloud and ideal depth image dimensions differ")
    if image.header.frame_id != info.header.frame_id:
        raise SystemExit("FAIL: depth image and calibration use different optical frames")
    fields = {field.name: field for field in cloud.fields}
    if any(axis not in fields or fields[axis].datatype != 7 for axis in "xyz"):
        raise SystemExit("FAIL: expected float32 XYZ cloud fields")
    dtype = np.dtype({"names": list("xyz"),
                      "formats": [(">" if cloud.is_bigendian else "<") + "f4"] * 3,
                      "offsets": [fields[axis].offset for axis in "xyz"], "itemsize": cloud.point_step})
    data = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                      strides=(cloud.row_step, cloud.point_step))
    xyz = np.stack([data[axis] for axis in "xyz"], axis=-1)
    depth = np.ndarray((image.height, image.width), dtype=">f4" if image.is_bigendian else "<f4",
                       buffer=image.data, strides=(image.step, 4))
    valid = np.isfinite(xyz).all(axis=-1) & np.isfinite(depth) & (depth > 0)
    if valid.sum() < 100:
        raise SystemExit("FAIL: too few finite cloud points for projection check")
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    optical = xyz[valid] @ rotation.T + np.array([t.x, t.y, t.z])
    rows, columns = np.nonzero(valid)
    d = depth[valid]
    # Gazebo's render rays sample pixel centers.
    expected = np.column_stack(((columns + 0.5 - info.k[2]) * d / info.k[0],
                                (rows + 0.5 - info.k[5]) * d / info.k[4], d))
    error = np.linalg.norm(optical - expected, axis=1)
    maximum = float(error.max())
    if maximum > 0.005:
        raise SystemExit(f"FAIL: cloud in {cloud.header.frame_id} disagrees with optical depth/calibration: "
                         f"max XYZ error {maximum:.3f} m (check the cloud frame convention)")
    print(f"PASS: {cloud.header.frame_id} cloud projects into {image.header.frame_id}; "
          f"{len(error)} points, max XYZ error {maximum * 1000:.3f} mm", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=sorted(CAMERAS), default="wrist")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--subpixel", type=float, default=0.08, help="realsense_depth's subpixel RMS.")
    args = parser.parse_args()
    baseline, hfov, min_depth, max_depth = CAMERAS[args.camera]
    rclpy.init()
    node = Node("realsense_check")
    ideal, noisy, clouds, calibration = {}, {}, {}, {}
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    stamp = lambda message: (message.header.stamp.sec, message.header.stamp.nanosec)  # noqa: E731
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, f"/camera/{args.camera}/depth/image_meters",
                             lambda m: ideal.__setitem__(stamp(m), m), qos)
    node.create_subscription(Image, f"/camera/{args.camera}/depth/image_rect_raw",
                             lambda m: noisy.__setitem__(stamp(m), m), qos)

    def on_cloud(message):
        clouds[stamp(message)] = message
        if len(clouds) > 5:
            del clouds[next(iter(clouds))]

    node.create_subscription(PointCloud2, f"/camera/{args.camera}/depth/points", on_cloud, qos)
    node.create_subscription(CameraInfo, f"/camera/{args.camera}/depth/camera_info",
                             lambda m: calibration.__setitem__("latest", m), qos)
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
        cloud_stamps = sorted(clouds.keys() & ideal.keys())
        if not cloud_stamps or not calibration:
            raise SystemExit("FAIL: no matched depth cloud/image and calibration")
        key = cloud_stamps[-1]
        cloud, image = clouds[key], ideal[key]
        transform = buffer.lookup_transform(image.header.frame_id, cloud.header.frame_id,
                                            Time.from_msg(cloud.header.stamp)).transform
        check_cloud(cloud, image, calibration["latest"], transform)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
