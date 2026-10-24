"""One robot and MoveIt configuration, backed by Gazebo or mock ros2_control."""
from pathlib import Path
import re
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from arm_bringup.scene import load_world
import xacro
import yaml


# Packages whose meshes the robot description references.
MESH_PACKAGES = ["arm_description", "xarm_description"]
# realsense_depth parameters per camera: stereo baseline and the depth imager's
# field of view and range at 640x360 (see cameras.xacro).
WRIST = {"name": "wrist", "baseline": 0.050, "horizontal_fov": 1.5184, "min_depth": 0.17, "max_depth": 10.0}
OVERHEAD = {"name": "overhead", "baseline": 0.095, "horizontal_fov": 1.5184, "min_depth": 0.4, "max_depth": 6.0}


def camera_bridges(name):
    """Lazy ros_gz_bridge entries for one RealSense-style camera, named like realsense-ros."""
    gz, ros = f"/camera/{name}", f"/camera/{name}"
    topics = [
        ("color/image", "color/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        ("color/camera_info", "color/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        # Ideal 32FC1 metres. realsense_depth publishes the driver's noisy
        # 16UC1 millimetres on depth/image_rect_raw from Gazebo's topic.
        ("depth/image", "depth/image_meters", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        ("depth/camera_info", "depth/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        ("depth/image/points", "depth/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked"),
    ]
    # Lazy entries subscribe to Gazebo only while a ROS subscriber exists, and
    # Gazebo renders a camera only while it has a subscriber.
    return [{"gz_topic_name": f"{gz}/{g}", "ros_topic_name": f"{ros}/{r}", "ros_type_name": rt,
             "gz_type_name": gt, "direction": "GZ_TO_ROS", "lazy": True} for g, r, rt, gt in topics]


def launch_setup(context):
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    mode = LaunchConfiguration("mode").perform(context)
    robot = LaunchConfiguration("robot").perform(context)
    gui = flag("gui")
    rviz = flag("rviz")
    sim = mode == "gazebo"
    ousters = sim and flag("ousters")
    perception = flag("perception")
    if perception and not ousters:
        raise ValueError("perception requires Gazebo mode and ousters:=true")
    gripper = robot == "xarm6" and flag("gripper")
    cameras = sim and flag("cameras")
    weld = sim and gripper and flag("weld_grasps")
    wrist_camera = cameras and robot == "xarm6"
    wrist_imu = wrist_camera and flag("wrist_imu")
    anti_aliasing = LaunchConfiguration("camera_anti_aliasing").perform(context)
    config_dir = Path(get_package_share_directory("arm_moveit_config")) / "config"
    model_config = config_dir / "xarm6" if robot == "xarm6" else config_dir
    description_dir = Path(get_package_share_directory("arm_description")) / "urdf"
    bringup_dir = Path(get_package_share_directory("arm_bringup"))
    world_file = bringup_dir / "worlds" / (LaunchConfiguration("world").perform(context) + ".sdf")
    scene = load_world(world_file)
    overhead_camera = cameras and "overhead_camera_mount" in scene.frames
    controllers = str(model_config / "ros2_controllers.yaml")
    description_mappings = {"hardware": mode, "controllers_file": controllers,
                            "mount_xyz": " ".join(map(str, scene.mount[:3])),
                            "mount_rpy": " ".join(map(str, scene.mount[3:]))}
    if robot == "xarm6":
        description_mappings.update({
            "gripper": str(gripper).lower(), "wrist_camera": str(wrist_camera).lower(),
            "camera_hz": LaunchConfiguration("wrist_camera_hz").perform(context),
            "camera_anti_aliasing": anti_aliasing, "wrist_imu": str(wrist_imu).lower(),
            "imu_noise": LaunchConfiguration("imu_noise").perform(context)})
    semantic = ((model_config / "arm.srdf.xacro", {"gripper": str(gripper).lower()}) if robot == "xarm6"
                else (model_config / "arm.srdf", {}))
    config = (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        .robot_description(
            file_path=str(description_dir / ("xarm6.urdf.xacro" if robot == "xarm6" else "arm.urdf.xacro")),
            mappings=description_mappings,
        )
        .robot_description_semantic(file_path=str(semantic[0]), mappings=semantic[1])
        .joint_limits(file_path=str(model_config / "joint_limits.yaml"))
        .trajectory_execution(file_path=str(model_config / "moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    if not perception:
        config.sensors_3d.clear()
    if not gripper:
        manager = config.trajectory_execution["moveit_simple_controller_manager"]
        manager["controller_names"] = [name for name in manager["controller_names"] if name != "gripper_controller"]
        manager.pop("gripper_controller", None)
    physics_description = dict(config.robot_description)
    # Gazebo gets local file URIs; ROS visualizers get portable package URIs.
    portable = config.robot_description["robot_description"]
    for package in MESH_PACKAGES:
        portable = portable.replace("file://" + get_package_share_directory(package) + "/", f"package://{package}/")
    config.robot_description = {"robot_description": portable}
    common = {"use_sim_time": sim}
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[config.robot_description, common], output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        parameters=[config.to_dict(), {"octomap_frame": "world", "octomap_resolution": 0.04}, common], output="screen",
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", str(config_dir / "moveit.rviz")],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.planning_pipelines,
                    config.joint_limits, common],
        output="screen",
    )
    scene_loader = Node(
        package="arm_bringup", executable="load_scene",
        parameters=[{"world_file": str(world_file)}, config.robot_description_semantic, common], output="screen",
    )
    spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "arm_controller", *(["gripper_controller"] if gripper else []),
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120",
                   # On a cold host the cameras' first shader compile can stall
                   # Gazebo for seconds; a timed-out switch call that is retried
                   # finds its controllers already active and fails the launch.
                   "--switch-timeout", "60", "--service-call-timeout", "60",
                   "--param-file", controllers],
        output="screen",
    )

    def after_controllers(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Controller activation failed"))]
        return [move_group, scene_loader]

    def after_scene(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="SDF planning scene loading failed"))]
        return [ui_node, *([rviz_node] if rviz else [])]

    actions = [
        state_publisher,
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_controllers)),
        RegisterEventHandler(OnProcessExit(target_action=scene_loader, on_exit=after_scene)),
    ]
    if flag("bridge"):
        meshes = r"/meshes/.*\.(stl|dae|obj|png|jpg|mtl)$"
        actions.append(Node(
            package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
            parameters=[common, {
                "address": "0.0.0.0", "port": 8765,
                "asset_uri_allowlist": [pattern for package in MESH_PACKAGES for pattern in [
                    f"^package://{package}{meshes}",
                    "^file://" + re.escape(get_package_share_directory(package)) + meshes]],
            }], output="screen",
        ))
    sensor_description = ""
    if ousters or overhead_camera:
        sensor_share = Path(get_package_share_directory("gz_sensors_ouster"))
        mappings = {"ousters": str(ousters).lower(), "overhead_camera": str(overhead_camera).lower(),
                    "metadata_path": str(sensor_share / "config/metadata/os1_64_rev7.json"),
                    "lidar_hz": LaunchConfiguration("lidar_hz").perform(context),
                    "overhead_hz": LaunchConfiguration("overhead_camera_hz").perform(context),
                    "camera_anti_aliasing": anti_aliasing}
        for frame, prefix in [("ouster_front_mount", "front"), ("ouster_rear_mount", "rear"),
                              ("overhead_camera_mount", "overhead")]:
            if frame in scene.frames:
                mappings[f"{prefix}_xyz"] = " ".join(map(str, scene.frames[frame][:3]))
                mappings[f"{prefix}_rpy"] = " ".join(map(str, scene.frames[frame][3:]))
        sensor_description = xacro.process_file(str(description_dir / "world_sensors.urdf.xacro"),
                                                mappings=mappings).toxml()
        actions.extend([
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 name="sensor_rig_state_publisher",
                 parameters=[{"robot_description": sensor_description}, common],
                 remappings=[("robot_description", "/sensor_rig/robot_description")], output="screen"),
            Node(package="tf2_ros", executable="static_transform_publisher", name="sensor_rig_mount",
                 arguments=["--frame-id", "world", "--child-frame-id", "sensor_rig"],
                 parameters=[common], output="screen"),
            Node(package="ros_gz_sim", executable="create", name="spawn_sensor_rig",
                 arguments=["-name", "workcell_sensors", "-topic", "/sensor_rig/robot_description", "-world", scene.name],
                 output="screen"),
        ])
    if ousters:
        actions.append(AppendEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH", str(Path(get_package_prefix("gz_sensors_ouster")) / "lib")))
        for name in ["ouster_front", "ouster_rear"]:
            actions.append(Node(
                package="ouster_ros", executable="os_cloud", name="os_cloud",
                namespace=f"/sensor/lidar/{name}", output="screen",
                parameters=[common, {
                    "proc_mask": "PCL", "point_cloud_frame": f"{name}/lidar_frame",
                    # cam_wip's ranges target the identity XYZ LUT. Distinct sensor_frame
                    # prevents os_cloud applying the real housing's 180deg + 36mm transform.
                    "pub_static_tf": False, "sensor_frame": f"{name}/os_sensor",
                    "lidar_frame": f"{name}/lidar_frame", "imu_frame": f"{name}/os_imu",
                    # The simulated sensor writes simulation time into its packets, as a
                    # PTP-synced unit writes its synced clock, so each scan keeps its exact
                    # start. TIME_FROM_ROS_TIME re-stamps scans from packet arrival on
                    # os_cloud's clock, which put them up to 30 ms off (6.6 ms standard
                    # deviation), on a 1 kHz /clock as well as the 200 Hz one.
                    "timestamp_mode": "TIME_FROM_INTERNAL_OSC",
                    # A 3.1 MB best-effort cloud is lost whole when any UDP
                    # fragment is; reliable delivery retransmits the fragment
                    # (see docker/cyclonedds.xml). Packets are reliable too,
                    # so no scan is assembled with missing columns.
                    "lidar_packet_reliable": True,
                    f"qos_overrides./sensor/lidar/{name}/points.publisher.reliability": "reliable",
                }],
            ))
    # In Gazebo the UI stamps part transforms with Gazebo's pose stamps and
    # needs no clock of its own; under use_sim_time rclpy would decode /clock
    # at 200 Hz (~9% of a core).
    ui_node = Node(
        package="arm_bringup", executable="workcell_ui", output="screen",
        parameters=[config.robot_description, config.robot_description_semantic, {
            "world_file": str(world_file), "sensor_description": sensor_description, "part_poses": sim,
            "weld_model": robot if weld else "",
        }],
    )
    if sim:
        gazebo_dir = Path(get_package_share_directory("ros_gz_sim"))
        # Cameras render through EGL without an X display unless the GUI runs.
        gz_args = f'-r {"" if gui else "-s --headless-rendering "}{world_file}'
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_dir / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args}.items(),
        )
        # create re-sends its request every 5 s until Gazebo answers, and Gazebo
        # later receives the unanswered ones too. Renaming stays off (the default)
        # so it drops those repeats; gflags would read "-allow_renaming false" as
        # true, which spawned a renamed copy of the robot per retry on slow starts.
        spawn_robot = Node(
            package="ros_gz_sim", executable="create",
            arguments=["-name", robot, "-param", "robot_description", "-world", scene.name],
            parameters=[physics_description],
            output="screen",
        )

        def after_spawn(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason="Gazebo robot spawn failed"))]
            return [spawner]

        actions.extend([
            AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", str(Path(get_package_prefix("arm_bringup")) / "lib")),
            gazebo,
            # Gazebo publishes its clock every 1 ms step; relaying every one
            # wakes each use_sim_time node 1000 times a second. Part poses come
            # from SceneBroadcaster rather than a gz system per part.
            Node(package="arm_bringup", executable="gz_relay",
                 parameters=[{"world": scene.name,
                              "clock_rate": float(LaunchConfiguration("clock_hz").perform(context)),
                              "parts": list(dict.fromkeys(item.model for item in scene.movable)) or [""]}],
                 output="screen"),
            RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=after_spawn)),
            spawn_robot,
        ])
        cameras_on = [camera for camera, enabled in [(WRIST, wrist_camera), (OVERHEAD, overhead_camera)] if enabled]
        bridges = [entry for camera in cameras_on for entry in camera_bridges(camera["name"])]
        if wrist_imu:
            bridges.append({"gz_topic_name": "/camera/wrist/imu", "ros_topic_name": "/camera/wrist/imu",
                            "ros_type_name": "sensor_msgs/msg/Imu", "gz_type_name": "gz.msgs.IMU",
                            "direction": "GZ_TO_ROS", "lazy": True})
        # RealSense depth (stereo noise, 1 mm units, 16UC1) from Gazebo's ideal depth.
        actions.extend(Node(package="arm_bringup", executable="realsense_depth", name="realsense_depth",
                            namespace=f"/camera/{camera['name']}", output="screen",
                            parameters=[{"gz_topic": f"/camera/{camera['name']}/depth/image",
                                         **{key: value for key, value in camera.items() if key != "name"}}])
                       for camera in cameras_on)
        if bridges:
            bridge_config = tempfile.NamedTemporaryFile("w", prefix="arm_gz_bridge_", suffix=".yaml", delete=False)
            with bridge_config:
                yaml.safe_dump(bridges, bridge_config)
            actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
                                parameters=[{"config_file": bridge_config.name}, common], output="screen"))
    else:
        actions.extend([
            Node(package="controller_manager", executable="ros2_control_node",
                 parameters=[controllers, common],
                 remappings=[("~/robot_description", "/robot_description")], output="screen"),
            spawner,
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bridge", default_value="true", choices=["true", "false"],
                              description="Serve Foxglove WebSocket data and robot mesh assets for Lichtblick."),
        DeclareLaunchArgument("ousters", default_value="true", choices=["true", "false"],
                              description="Spawn two fixed OS1-64 sensors in Gazebo using cam_wip's CPU raycaster."),
        DeclareLaunchArgument("lidar_hz", default_value="10.0",
                              description="Scan rate per Ouster sensor; the OS1-64 metadata's 1024x10 mode is 10 Hz."),
        DeclareLaunchArgument("perception", default_value="false", choices=["true", "false"],
                              description="Integrate both Ouster point clouds into MoveIt's occupancy map."),
        DeclareLaunchArgument("gripper", default_value="true", choices=["true", "false"],
                              description="Mount the UFACTORY xArm Gripper on the xArm6."),
        DeclareLaunchArgument("weld_grasps", default_value="true", choices=["true", "false"],
                              description="Weld a part to the hand in Gazebo once the gripper has closed on "
                                          "it, instead of holding it by contact friction alone."),
        DeclareLaunchArgument("cameras", default_value="true", choices=["true", "false"],
                              description="Add the wrist D435i and overhead D455 RGB-D cameras (Gazebo mode)."),
        DeclareLaunchArgument("wrist_camera_hz", default_value="15.0",
                              description="Wrist D435i frame rate (native modes: 6, 15, 30, 60)."),
        DeclareLaunchArgument("overhead_camera_hz", default_value="15.0",
                              description="Overhead D455 frame rate (native modes: 5, 15, 30, 60). Equal to "
                                          "the wrist rate, both cameras render together."),
        DeclareLaunchArgument("camera_anti_aliasing", default_value="0",
                              description="MSAA samples for color images (Gazebo's default is 4). Under "
                                          "software rendering 4x MSAA multiplies each image's cost."),
        DeclareLaunchArgument("wrist_imu", default_value="false", choices=["true", "false"],
                              description="Simulate the D435i's IMU on /camera/wrist/imu (200 Hz)."),
        DeclareLaunchArgument("imu_noise", default_value="true", choices=["true", "false"],
                              description="Apply the IMU's noise and turn-on bias (false for validation)."),
        DeclareLaunchArgument("world", default_value="workcell", choices=["workcell", "obstacle_course", "empty"],
                              description="SDF scene used by both Gazebo and MoveIt's collision world."),
        DeclareLaunchArgument("robot", default_value="xarm6", choices=["xarm6", "demo"],
                              description="Use the official UFACTORY xArm6 or the primitive demo arm."),
        DeclareLaunchArgument("mode", default_value="gazebo", choices=["gazebo", "mock"],
                              description="Use Gazebo physics or ros2_control mock hardware."),
        DeclareLaunchArgument("gui", default_value="false", choices=["true", "false"],
                              description="Start the Gazebo GUI (Gazebo mode only)."),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("clock_hz", default_value="200.0",
                              description="Rate of ROS /clock updates relayed from Gazebo's 1 kHz clock."),
        OpaqueFunction(function=launch_setup),
    ])
