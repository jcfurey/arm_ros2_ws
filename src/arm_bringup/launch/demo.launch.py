"""One robot and MoveIt configuration, backed by Gazebo or mock ros2_control."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    sim = mode == "gazebo"
    config_dir = Path(get_package_share_directory("arm_moveit_config")) / "config"
    bringup_dir = Path(get_package_share_directory("arm_bringup"))
    controllers = str(config_dir / "ros2_controllers.yaml")
    config = (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        .robot_description(mappings={"hardware": mode, "controllers_file": controllers})
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    common = {"use_sim_time": sim}
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[config.robot_description, common], output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        parameters=[config.to_dict(), common], output="screen",
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", str(config_dir / "moveit.rviz")],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.planning_pipelines,
                    config.joint_limits, common],
        output="screen",
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
        return [move_group, *([rviz_node] if rviz else [])]

    actions = [
        state_publisher,
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_controllers)),
    ]
    if sim:
        gazebo_dir = Path(get_package_share_directory("ros_gz_sim"))
        gz_args = f'-r {"" if gui else "-s "}{bringup_dir / "worlds" / "empty.sdf"}'
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_dir / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args}.items(),
        )
        spawn_robot = Node(
            package="ros_gz_sim", executable="create",
            arguments=["-name", "demo_arm", "-topic", "/robot_description",
                       "-world", "arm_world", "-allow_renaming", "false"],
            output="screen",
        )

        def after_spawn(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason="Gazebo robot spawn failed"))]
            return [spawner]

        actions.extend([
            gazebo,
            Node(package="ros_gz_bridge", executable="parameter_bridge",
                 arguments=["/world/arm_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                 remappings=[("/world/arm_world/clock", "/clock")], output="screen"),
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
        DeclareLaunchArgument("mode", default_value="gazebo", choices=["gazebo", "mock"],
                              description="Use Gazebo physics or ros2_control mock hardware."),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"],
                              description="Start the Gazebo GUI (Gazebo mode only)."),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=launch_setup),
    ])
