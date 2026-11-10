"""Static snapshots must survive clients retaining only one message per topic."""
import os
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from arm_bringup.static_tf import STATIC_TF_SOURCES, StaticTransforms


def transform(parent, child, x=0.0):
    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = x
    message.transform.rotation.w = 1.0
    return message


class StaticTransformTests(unittest.TestCase):
    def test_late_subscriber_gets_all_sources_in_one_retained_snapshot(self):
        # Running the suite beside a live stack must never publish test
        # mounts into that stack, even when ROS_DOMAIN_ID is the same.
        scope = f"/_static_tf_test_{os.getpid()}"
        rclpy.init(args=["--ros-args", "-r", f"__ns:={scope}",
                         "-r", f"{STATIC_TF_SOURCES}:={scope}/sources",
                         "-r", f"/tf_static:={scope}/tf_static"])
        executor = SingleThreadedExecutor()
        nodes = []

        def add(node):
            nodes.append(node)
            executor.add_node(node)
            return node

        def until(predicate):
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if predicate():
                    return
            self.fail("Timed out waiting for a complete static TF snapshot")

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        try:
            # Each source retains a different fragment, published before the
            # consolidator joins, just as the two URDFs and rig mount do.
            groups = [[transform("world", "link_base")],
                      [transform("sensor_rig", "lidar"), transform("sensor_rig", "overhead")],
                      [transform("world", "sensor_rig")]]
            publishers = [add(Node(f"static_source_{i}")).create_publisher(TFMessage, STATIC_TF_SOURCES, qos)
                          for i in range(len(groups))]
            for publisher, group in zip(publishers, groups):
                publisher.publish(TFMessage(transforms=group))
            add(StaticTransforms())
            reader = add(Node("static_snapshot_reader"))
            received = []
            subscription = reader.create_subscription(TFMessage, "/tf_static", received.append, qos)
            until(lambda: received and len(received[-1].transforms) == 4)
            reader.destroy_subscription(subscription)

            # A newly mounted panel only needs the latest retained message.
            late = []
            reader.create_subscription(TFMessage, "/tf_static", late.append, qos)
            until(lambda: bool(late))
            self.assertEqual({t.child_frame_id: t.header.frame_id for t in late[0].transforms},
                             {"link_base": "world", "sensor_rig": "world",
                              "lidar": "sensor_rig", "overhead": "sensor_rig"})

            # Updating one source replaces its edge without dropping others.
            publishers[0].publish(TFMessage(transforms=[transform("world", "link_base", 0.5)]))
            until(lambda: any(t.child_frame_id == "link_base" and t.transform.translation.x == 0.5
                              for t in late[-1].transforms))
            self.assertEqual(len(late[-1].transforms), 4)
        finally:
            executor.shutdown()
            for node in reversed(nodes):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
