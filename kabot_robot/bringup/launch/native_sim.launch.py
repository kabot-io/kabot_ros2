from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_native_sim(context):
    package_directory = Path(get_package_share_directory("kabot_robot"))
    schema_path = Path(LaunchConfiguration("schema_path").perform(context)).expanduser().resolve()
    if not schema_path.is_file():
        raise FileNotFoundError(f"Native simulator schema not found: {schema_path}")
    robot_description = xacro.process_file(
        str(package_directory / "urdf" / "kabot_native_sim.urdf.xacro"),
        mappings={
            "schema_path": str(schema_path),
            "zenoh_endpoint": LaunchConfiguration("zenoh_endpoint").perform(context),
        },
    ).toxml()
    calibrated = LaunchConfiguration("calibrated").perform(context).lower() == "true"
    controllers = str(package_directory / "config" / (
        "kabot_calibrated_controllers.yaml" if calibrated else "kabot_native_sim_controllers.yaml"))
    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[controllers],
        output="both",
    )
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "kabot_state_broadcaster",
            "kabot_motion_model" if calibrated else "kabot_wheel_pid",
            "kabot_base_controller",
            "--activate-as-group",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers,
            "--controller-ros-args", "-r ~/cmd_vel:=/cmd_vel",
        ],
        output="both",
    )

    def check_spawner(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Native simulator controllers failed"))]
        return []

    return [
        RegisterEventHandler(OnProcessExit(
            target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason="Controller manager stopped"))],
        )),
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=check_spawner)),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="both",
        ),
        manager,
        spawner,
        Node(
            package="twist_stamper",
            executable="twist_stamper",
            name="twist_stamper",
            parameters=[{"frame_id": "base_link"}],
            remappings=[("cmd_vel_in", "/cmd_vel_unstamped"), ("cmd_vel_out", "/cmd_vel")],
            output="both",
        ),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", str(package_directory / "rviz/kabot_mock.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("schema_path", description="Tier 2 YAML used to build the firmware."),
        DeclareLaunchArgument("zenoh_endpoint", default_value="tcp/127.0.0.1:7447"),
        DeclareLaunchArgument("calibrated", default_value="true", choices=["true", "false"],
                      description="Physical Twist units with model odometry; false for historical raw calibration."),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=launch_native_sim),
    ])