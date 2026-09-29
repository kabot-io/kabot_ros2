# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_directory = PathSubstitution(
        FindPackageShare(LaunchConfiguration("description_package"))
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                ' "',
                description_directory / "urdf" / LaunchConfiguration("description_file"),
                '" prefix:=',
                LaunchConfiguration("prefix"),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "description_package",
                default_value="kabot_robot",
                description=(
                    "Package containing the robot URDF/Xacro and RViz configuration."
                ),
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value="kabot.urdf.xacro",
                description="URDF/XACRO description file with the robot.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description=(
                    "Start Rviz2 and Joint State Publisher gui automatically "
                    "with this launch file."
                ),
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="Prefix applied to all link and joint names.",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(LaunchConfiguration("gui")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                # Qt/OGRE can flicker with fractional Windows display scaling.
                additional_env={
                    "QT_ENABLE_HIGHDPI_SCALING": os.environ.get("QT_ENABLE_HIGHDPI_SCALING", "0"),
                } if os.name == "nt" else {},
                output="log",
                arguments=[
                    "-d",
                    description_directory / "rviz" / "kabot_view.rviz",
                    "-f",
                    [LaunchConfiguration("prefix"), "base_link"],
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
        ]
    )
