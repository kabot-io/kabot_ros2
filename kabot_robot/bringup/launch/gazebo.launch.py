import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
    RegisterEventHandler, Shutdown, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def launch_gazebo(context):
    package_directory = PathSubstitution(FindPackageShare("kabot_robot"))
    controller_parameters = ParameterFile(
        package_directory / "config" / "kabot_gazebo_controllers.yaml", allow_substs=True
    )
    controller_parameter_path = str(controller_parameters.evaluate(context))
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), ' "',
            package_directory / "urdf" / "kabot_gazebo.urdf.xacro",
            '" prefix:="', LaunchConfiguration("prefix"), '" controller_config:="',
            controller_parameter_path, '"',
        ]),
        value_type=str,
    )
    world = (package_directory / "worlds" / "kabot.world.sdf").perform(context)
    plugin_directory = (PathSubstitution(FindPackagePrefix("gz_ros2_control")) / "lib").perform(context)
    gazebo = ExecuteProcess(
        cmd=[FindExecutable(name="gz"), "sim", "-r", "-s", world],
        name="gazebo_server", output="screen", on_exit=Shutdown(),
        additional_env={"GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join(filter(None, [
            os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH"), plugin_directory,
        ]))},
    )
    create = Node(
        package="ros_gz_sim", executable="create", name="spawn_kabot",
        arguments=["-world", "kabot_world", "-topic", "/simulation/robot_description",
                   "-name", "kabot", "-allow_renaming", "false", "-z", "0.02"],
        output="both",
    )
    spawner = Node(
        package="controller_manager", executable="spawner", name="controller_spawner",
        arguments=[
            "joint_state_broadcaster", "kabot_base_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--param-file", controller_parameter_path, "--activate-as-group",
            "--controller-ros-args", "-r ~/cmd_vel:=/cmd_vel",
        ],
        output="both",
    )
    spawn_finished = False

    def after_spawn(event, context):
        nonlocal spawn_finished
        spawn_finished = True
        if event.returncode != 0:
            return [Shutdown(reason="Failed to create Kabot in Gazebo")]
        return [spawner]

    def spawn_timeout(context):
        if not spawn_finished:
            return [Shutdown(reason="Timed out creating Kabot in Gazebo (45 seconds)")]
        return []

    def after_controllers(event, context):
        if event.returncode != 0:
            return [Shutdown(reason="Failed to activate Kabot controllers")]
        return []

    return [
        RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: controller_parameters.cleanup()
        )),
        RegisterEventHandler(OnProcessExit(target_action=create, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_controllers)),
        gazebo,
        ExecuteProcess(
            cmd=[FindExecutable(name="gz"), "sim", "-g" ],
            name="gazebo_gui", output="screen", on_exit=Shutdown(),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="simulation_robot_state_publisher",
            remappings=[("robot_description", "/simulation/robot_description")],
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            output="both",
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="gazebo_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/model/kabot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            ],
            remappings=[("/model/kabot/pose", "/simulation/ground_truth")],
            parameters=[{"use_sim_time": True}], output="both",
        ),
        create,
        TimerAction(period=45.0, actions=[OpaqueFunction(function=spawn_timeout)]),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", package_directory / "rviz" / "kabot_mock.rviz",
                       "-f", [LaunchConfiguration("prefix"), "odom"]],
            parameters=[{"use_sim_time": True}],
            remappings=[("/robot_description", "/simulation/robot_description")],
            condition=IfCondition(LaunchConfiguration("rviz")), output="log",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI."),
        DeclareLaunchArgument("rviz", default_value="false", description="Start RViz2."),
        DeclareLaunchArgument("prefix", default_value="", description="Joint and TF frame prefix."),
        OpaqueFunction(function=launch_gazebo),
    ])