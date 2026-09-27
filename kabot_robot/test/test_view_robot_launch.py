import math
import os
from pathlib import Path
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def generate_test_description():
    prefix = os.environ.get("KABOT_TEST_PREFIX", "")
    launch_file = (
        Path(get_package_share_directory("kabot_robot")) / "launch/view_robot.launch.py"
    )
    view_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments={"gui": "false", "prefix": prefix}.items(),
    )
    return LaunchDescription([view_robot, ReadyToTest()]), {"prefix": prefix}


class TestRobotDescription(unittest.TestCase):
    def test_description_joint_states_and_transforms(self, prefix):
        rclpy.init()
        node = rclpy.create_node("kabot_description_test")
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        descriptions = []
        joint_states = []
        node.create_subscription(
            String,
            "/robot_description",
            descriptions.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        node.create_subscription(JointState, "/joint_states", joint_states.append, 10)
        child_links = [
            "chassis", "back_slider", "front_slider", "top", "left_wheel", "right_wheel"
        ]
        base_frame = prefix + "base_link"
        try:
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if descriptions and joint_states and all(
                    buffer.can_transform(base_frame, prefix + child, Time())
                    for child in child_links
                ):
                    break

            self.assertTrue(descriptions, "No robot_description received")
            self.assertTrue(joint_states, "No joint_states received")
            robot = ET.fromstring(descriptions[-1].data)
            self.assertEqual(robot.attrib["name"], "Kabot")
            self.assertEqual(
                {link.attrib["name"] for link in robot.findall("link")},
                {base_frame, *(prefix + child for child in child_links)},
            )
            self.assertIsNone(robot.find("ros2_control"))
            states = joint_states[-1]
            self.assertEqual(
                set(states.name),
                {prefix + "left_wheel_joint", prefix + "right_wheel_joint"},
            )
            self.assertEqual(len(states.name), 2)
            self.assertEqual(len(states.position), 2)
            self.assertTrue(all(math.isfinite(value) for value in states.position))
            for position in states.position:
                self.assertAlmostEqual(position, 0.0)
            self.assertEqual(node.count_publishers("/joint_states"), 1)
            self.assertNotIn("controller_manager", node.get_node_names())

            for joint in robot.findall("joint"):
                child = joint.find("child").attrib["link"]
                self.assertTrue(
                    buffer.can_transform(base_frame, child, Time()),
                    f"Missing transform {base_frame} -> {child}",
                )
                transform = buffer.lookup_transform(base_frame, child, Time()).transform
                origin = joint.find("origin")
                expected_position = [float(value) for value in origin.attrib["xyz"].split()]
                actual_position = [
                    transform.translation.x, transform.translation.y, transform.translation.z
                ]
                for actual, expected in zip(actual_position, expected_position):
                    self.assertAlmostEqual(actual, expected)
                roll = float(origin.get("rpy", "0 0 0").split()[0])
                actual_rotation = [
                    transform.rotation.x, transform.rotation.y,
                    transform.rotation.z, transform.rotation.w,
                ]
                expected_rotation = [math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2)]
                for actual, expected in zip(actual_rotation, expected_rotation):
                    self.assertAlmostEqual(actual, expected)
        finally:
            listener.unregister()
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)