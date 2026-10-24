#!/usr/bin/env python3
"""Verify Foxglove schemas, streamed ROS data, and remote robot-mesh retrieval."""
import argparse
import json
import struct
import time

import websocket
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://127.0.0.1:8765")
    parser.add_argument("--no-ousters", action="store_true")
    parser.add_argument("--pose", choices=["ready", "sweep", "home"],
                        help="Also publish a named pose through the bridge and await execution")
    args = parser.parse_args()
    required = {"/clock", "/tf", "/tf_static", "/scene/markers", "/robot/markers", "/motion/status"}
    if not args.no_ousters:
        required |= {"/sensor/lidar/ouster_front/points", "/sensor/lidar/ouster_rear/points"}
    connect_deadline = time.monotonic() + 60
    while True:
        try:
            connection = websocket.create_connection(args.url,
                subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"], timeout=5)
            break
        except (OSError, websocket.WebSocketException):
            if time.monotonic() >= connect_deadline:
                raise
            time.sleep(0.5)
    subscriptions, received = {}, set()
    # The robot description references the simplified copies of UFACTORY's meshes.
    assets = {
        1: "package://arm_description/meshes/sim/xarm6/visual/link_base.stl",
        2: "file://" + get_package_share_directory("arm_description") + "/meshes/sim/xarm6/visual/link_base.stl",
    }
    assets_received = set()
    command_sent = motion_started = False
    if args.pose:
        from rclpy.serialization import deserialize_message, serialize_message
        from std_msgs.msg import String
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            try:
                message = connection.recv()
            except websocket.WebSocketTimeoutException:
                continue
            if isinstance(message, str):
                message = json.loads(message)
                if message.get("op") == "serverInfo":
                    if "assets" not in message.get("capabilities", []):
                        raise RuntimeError("Bridge does not support remote mesh assets")
                    for request, uri in assets.items():
                        connection.send(json.dumps({"op": "fetchAsset", "requestId": request, "uri": uri}))
                    if args.pose:
                        if "clientPublish" not in message.get("capabilities", []):
                            raise RuntimeError("Bridge does not support publishing motion commands")
                        connection.send(json.dumps({"op": "advertise", "channels": [{
                            "id": 1, "topic": "/motion/command", "encoding": "cdr",
                            "schemaName": "std_msgs/msg/String", "schemaEncoding": "ros2msg",
                            "schema": "string data\n"}]}))
                elif message.get("op") == "advertise":
                    for channel in message["channels"]:
                        if channel["topic"] in required and channel["topic"] not in subscriptions.values():
                            if channel["encoding"] != "cdr" or not channel.get("schema"):
                                raise RuntimeError(f"Missing ROS 2 schema: {channel['topic']}")
                            subscription = len(subscriptions) + 1
                            subscriptions[subscription] = channel["topic"]
                            connection.send(json.dumps({"op": "subscribe", "subscriptions": [
                                {"id": subscription, "channelId": channel["id"]}]}))
                elif message.get("op") == "status" and message.get("level") == 2:
                    raise RuntimeError(message["message"])
            elif message:
                if message[0] == 1 and len(message) > 13:
                    subscription = struct.unpack_from("<I", message, 1)[0]
                    if subscription in subscriptions:
                        received.add(subscriptions[subscription])
                        if args.pose and command_sent and subscriptions[subscription] == "/motion/status":
                            status = json.loads(deserialize_message(message[13:], String).data)
                            if status.get("pose") == args.pose:
                                motion_started |= status["state"] in ("planning", "executing")
                                if status["state"] in ("failed", "rejected", "stopped"):
                                    raise RuntimeError(f"Motion command failed: {status}")
                                if motion_started and status["state"] == "succeeded":
                                    print(f"PASS: bridge-published {args.pose} command planned and executed")
                                    return
                elif message[0] == 4:
                    request, status, error_length = struct.unpack_from("<IBI", message, 1)
                    if request in assets:
                        if status or len(message) <= 10 + error_length:
                            raise RuntimeError("Robot mesh retrieval failed: " + message[10:10+error_length].decode())
                        assets_received.add(request)
            if required <= received and assets_received == assets.keys():
                if not command_sent:
                    print("PASS: Foxglove WebSocket schemas, clock, TF, scene, robot, status, and sensor streams")
                    print("PASS: package:// and file:// robot mesh assets fetched through the bridge")
                    if not args.pose:
                        return
                    connection.send_binary(struct.pack("<BI", 1, 1) + serialize_message(String(data=args.pose)))
                    command_sent = True
                    deadline = time.monotonic() + 60
        if command_sent:
            raise RuntimeError(f"No successful motion status for bridge-published {args.pose} command")
        raise RuntimeError(f"Missing bridge streams: {required - received}; mesh assets: {assets_received}")
    finally:
        connection.close()


if __name__ == "__main__":
    main()
