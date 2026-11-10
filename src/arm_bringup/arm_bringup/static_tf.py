#!/usr/bin/env python3
"""Retain the complete static frame tree in a single TFMessage."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


# Hidden so visualizers do not also consume the individual, partial snapshots.
STATIC_TF_SOURCES = "/_tf_static_sources"


class StaticTransforms(Node):
    def __init__(self):
        super().__init__("static_transforms")
        # The broadcaster merges updates by child frame and retains the full
        # snapshot. Clients that cache only the last message on /tf_static can
        # then recover every fixed edge when a new panel subscribes.
        self.broadcaster = StaticTransformBroadcaster(self)
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, STATIC_TF_SOURCES, self.on_transforms, qos)

    def on_transforms(self, message):
        if message.transforms:
            self.broadcaster.sendTransform(message.transforms)


def main():
    rclpy.init()
    node = StaticTransforms()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
