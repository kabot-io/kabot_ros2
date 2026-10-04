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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_directory = PathSubstitution(FindPackageShare("kabot_robot"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=package_directory / "rviz" / "kabot_mock.rviz",
                description="RViz configuration for observing the running controller.",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="odom",
                description="Fixed world frame supplied by the controller TF tree.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=[
                    "-d",
                    LaunchConfiguration("rviz_config"),
                    "-f",
                    LaunchConfiguration("fixed_frame"),
                ],
            ),
        ]
    )
