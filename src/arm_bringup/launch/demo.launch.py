"""One robot and MoveIt configuration, backed by Gazebo or mock ros2_control."""
from pathlib import Path
import re

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


def launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    robot = LaunchConfiguration("robot").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    sim = mode == "gazebo"
    ousters = sim and LaunchConfiguration("ousters").perform(context).lower() == "true"
    perception = LaunchConfiguration("perception").perform(context).lower() == "true"
    if perception and not ousters:
        raise ValueError("perception requires Gazebo mode and ousters:=true")
    config_dir = Path(get_package_share_directory("arm_moveit_config")) / "config"
    model_config = config_dir / "xarm6" if robot == "xarm6" else config_dir
    description_dir = Path(get_package_share_directory("arm_description")) / "urdf"
    bringup_dir = Path(get_package_share_directory("arm_bringup"))
    world_file = bringup_dir / "worlds" / (LaunchConfiguration("world").perform(context) + ".sdf")
    scene = load_world(world_file)
    controllers = str(model_config / "ros2_controllers.yaml")
    config = (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        .robot_description(
            file_path=str(description_dir / ("xarm6.urdf.xacro" if robot == "xarm6" else "arm.urdf.xacro")),
            mappings={"hardware": mode, "controllers_file": controllers,
                      "mount_xyz": " ".join(map(str, scene.mount[:3])),
                      "mount_rpy": " ".join(map(str, scene.mount[3:]))},
        )
        .robot_description_semantic(file_path=str(model_config / "arm.srdf"))
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
    physics_description = dict(config.robot_description)
    mesh_package = get_package_share_directory("xarm_description")
    # Gazebo gets local file URIs; ROS visualizers get portable package URIs.
    config.robot_description = {"robot_description": config.robot_description["robot_description"].replace(
        "file://" + mesh_package + "/", "package://xarm_description/")}
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
        parameters=[{"world_file": str(world_file)}, common], output="screen",
    )
    spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "arm_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120",
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
    if LaunchConfiguration("bridge").perform(context).lower() == "true":
        mesh_path = Path(get_package_share_directory("xarm_description")) / "meshes"
        actions.append(Node(
            package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
            parameters=[common, {
                "address": "0.0.0.0", "port": 8765,
                "asset_uri_allowlist": [
                    r"^package://xarm_description/meshes/.*\.(stl|dae|obj|png|jpg|mtl)$",
                    "^file://" + re.escape(str(mesh_path)) + r"/.*\.(stl|dae|obj|png|jpg|mtl)$",
                ],
            }], output="screen",
        ))
    sensor_description = ""
    if ousters:
        sensor_share = Path(get_package_share_directory("gz_sensors_ouster"))
        front = scene.frames["ouster_front_mount"]
        rear = scene.frames["ouster_rear_mount"]
        sensor_description = xacro.process_file(str(description_dir / "world_sensors.urdf.xacro"), mappings={
            "metadata_path": str(sensor_share / "config/metadata/os1_64_rev7.json"),
            "front_xyz": " ".join(map(str, front[:3])), "front_rpy": " ".join(map(str, front[3:])),
            "rear_xyz": " ".join(map(str, rear[:3])), "rear_rpy": " ".join(map(str, rear[3:])),
            "lidar_hz": LaunchConfiguration("lidar_hz").perform(context),
        }).toxml()
        actions.extend([
            AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", str(Path(get_package_prefix("gz_sensors_ouster")) / "lib")),
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
                    "timestamp_mode": "TIME_FROM_ROS_TIME",
                }],
            ))
    ui_node = Node(
        package="arm_bringup", executable="workcell_ui", output="screen",
        parameters=[common, config.robot_description, {
            "world_file": str(world_file), "robot": robot, "sensor_description": sensor_description,
        }],
    )
    if sim:
        gazebo_dir = Path(get_package_share_directory("ros_gz_sim"))
        gz_args = f'-r {"" if gui else "-s "}{world_file}'
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_dir / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args}.items(),
        )
        spawn_robot = Node(
            package="ros_gz_sim", executable="create",
            arguments=["-name", robot, "-param", "robot_description",
                       "-world", scene.name, "-allow_renaming", "false"],
            parameters=[physics_description],
            output="screen",
        )

        def after_spawn(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason="Gazebo robot spawn failed"))]
            return [spawner]

        actions.extend([
            gazebo,
            Node(package="ros_gz_bridge", executable="parameter_bridge",
                 arguments=[f"/world/{scene.name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                 remappings=[(f"/world/{scene.name}/clock", "/clock")], output="screen"),
            RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=after_spawn)),
            spawn_robot,
        ])
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
        DeclareLaunchArgument("lidar_hz", default_value="3.0", description="Scan rate per Ouster sensor."),
        DeclareLaunchArgument("perception", default_value="false", choices=["true", "false"],
                              description="Integrate both Ouster point clouds into MoveIt's occupancy map."),
        DeclareLaunchArgument("world", default_value="workcell", choices=["workcell", "obstacle_course", "empty"],
                              description="SDF scene used by both Gazebo and MoveIt's collision world."),
        DeclareLaunchArgument("robot", default_value="xarm6", choices=["xarm6", "demo"],
                              description="Use the official UFACTORY xArm6 or the primitive demo arm."),
        DeclareLaunchArgument("mode", default_value="gazebo", choices=["gazebo", "mock"],
                              description="Use Gazebo physics or ros2_control mock hardware."),
        DeclareLaunchArgument("gui", default_value="false", choices=["true", "false"],
                              description="Start the Gazebo GUI (Gazebo mode only)."),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=launch_setup),
    ])
