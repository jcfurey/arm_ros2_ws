#!/usr/bin/env python3
"""Check decoded Ouster world points against the actual static SDF surfaces."""
import argparse
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from arm_bringup.scene import load_world


def rotation(q):
    x,y,z,w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def points(message):
    fields = {field.name: field for field in message.fields}
    if any(fields[axis].datatype != 7 for axis in 'xyz'):
        raise ValueError("Expected float32 XYZ cloud fields")
    dtype = np.dtype({'names': list('xyz'), 'formats': [('>' if message.is_bigendian else '<')+'f4']*3,
                      'offsets': [fields[axis].offset for axis in 'xyz'], 'itemsize': message.point_step})
    cloud = np.ndarray(shape=(message.height, message.width), dtype=dtype,
                       buffer=message.data, strides=(message.row_step, message.point_step))
    xyz = np.column_stack([cloud[axis].reshape(-1) for axis in 'xyz'])
    valid = np.isfinite(xyz).all(axis=1) & ((xyz*xyz).sum(axis=1) > 0.01)
    xyz = xyz[valid]
    return xyz[::max(1, len(xyz)//8000)]


def surface_distances(world_points, objects):
    distances = np.full(len(world_points), np.inf)
    for item in objects:
        local = (world_points - np.array(item.pose[:3])) @ rotation(item.pose[3:])
        if item.shape == 'box':
            d = np.abs(local) - np.array(item.dimensions)/2
            distance = np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(np.max(d, axis=1), 0)
        elif item.shape == 'cylinder':
            d = np.column_stack([np.linalg.norm(local[:,:2], axis=1)-item.dimensions[1],
                                 np.abs(local[:,2])-item.dimensions[0]/2])
            distance = np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(np.max(d, axis=1), 0)
        elif item.shape == 'sphere':
            distance = np.linalg.norm(local, axis=1)-item.dimensions[0]
        else:
            normal = np.array(item.dimensions[:3])
            distance = (local @ normal + item.dimensions[3]) / np.linalg.norm(normal)
        distances = np.minimum(distances, np.abs(distance))
    return distances


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', choices=['workcell','obstacle_course','empty'], default='workcell')
    args = parser.parse_args()
    scene = load_world(Path(get_package_share_directory('arm_bringup')) / 'worlds' / (args.world+'.sdf'))
    rclpy.init()
    node = Node('ouster_registration_check')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    clouds = {}
    names = ['ouster_front', 'ouster_rear']
    for name in names:
        node.create_subscription(PointCloud2, f'/sensor/lidar/{name}/points',
                                 lambda msg, name=name: clouds.__setitem__(name, msg), qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+90
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if all(name in clouds and buffer.can_transform('world', clouds[name].header.frame_id, Time()) for name in names):
                break
        else:
            raise RuntimeError('Missing clouds or TF for registration check')
        failed = []
        for name in names:
            message = clouds[name]
            transform = buffer.lookup_transform('world', message.header.frame_id, Time()).transform
            q,t = transform.rotation, transform.translation
            xyz = points(message) @ rotation((q.x,q.y,q.z,q.w)).T + np.array((t.x,t.y,t.z))
            if len(xyz) < 100:
                raise RuntimeError(f'{name}: too few finite returns')
            distance = surface_distances(xyz, scene.objects)
            matched = float((distance < 0.06).mean())
            print(f'{name}: {matched:.1%} within 6 cm of SDF surfaces; median={np.median(distance):.4f} m; p95={np.percentile(distance,95):.4f} m', flush=True)
            # A few hits are on the moving robot, which is not part of the static SDF world.
            if matched < 0.90:
                failed.append(name)
        if failed:
            raise RuntimeError(f'Clouds do not align with the world: {failed}')
        print('PASS: both Ouster clouds geometrically register to the SDF world', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
