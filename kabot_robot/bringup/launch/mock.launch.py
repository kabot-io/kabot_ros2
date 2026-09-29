import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_mock(context):
    package_directory = PathSubstitution(FindPackageShare("kabot_robot"))
    controller_parameters = ParameterFile(
        package_directory / "config" / "kabot_mock_controllers.yaml", allow_substs=True
    )
    controller_parameter_path = str(controller_parameters.evaluate(context))
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                ' "',
                package_directory / "urdf" / "kabot_mock.urdf.xacro",
                '" prefix:=',
                LaunchConfiguration("prefix"),
            ]
        ),
        value_type=str,
    )
    return [
        RegisterEventHandler(
            OnShutdown(on_shutdown=lambda event, context: controller_parameters.cleanup())
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="both",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            parameters=[{"update_rate": 100}],
            output="both",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="controller_spawner",
            arguments=[
                "joint_state_broadcaster",
                "kabot_base_controller",
                "--controller-manager", "/controller_manager",
                "--param-file", controller_parameter_path,
                "--activate-as-group",
                "--controller-ros-args", "-r ~/cmd_vel:=/cmd_vel",
            ],
            output="both",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            additional_env={
                "QT_ENABLE_HIGHDPI_SCALING": os.environ.get("QT_ENABLE_HIGHDPI_SCALING", "0"),
            } if os.name == "nt" else {},
            arguments=[
                "-d", package_directory / "rviz" / "kabot_mock.rviz",
                "-f", [LaunchConfiguration("prefix"), "odom"],
            ],
            condition=IfCondition(LaunchConfiguration("gui")),
            output="log",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Start RViz2."),
            DeclareLaunchArgument(
                "prefix", default_value="", description="Prefix for joint and TF frame names."
            ),
            OpaqueFunction(function=launch_mock),
        ]
    )
