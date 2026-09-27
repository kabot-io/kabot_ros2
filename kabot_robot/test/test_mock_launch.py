from collections import deque
import math
import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
from nav_msgs.msg import Odometry
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def generate_test_description():
    prefix = os.environ.get("KABOT_TEST_PREFIX", "")
    launch_file = Path(get_package_share_directory("kabot_robot")) / "launch/mock.launch.py"
    mock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments={"gui": "false", "prefix": prefix}.items(),
    )
    return LaunchDescription([mock, ReadyToTest()]), {"prefix": prefix}


class TestMockControl(unittest.TestCase):
    def wait_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail("Timed out waiting for mock control data")

    def command_for(self, linear, angular, duration=1.0):
        def publish():
            command = TwistStamped()
            command.header.stamp = self.node.get_clock().now().to_msg()
            command.twist.linear.x = linear
            command.twist.angular.z = angular
            self.publisher.publish(command)

        publish()
        timer = self.node.create_timer(0.05, publish)
        try:
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        finally:
            self.node.destroy_timer(timer)

    def wheel_states(self, attribute):
        states = self.states[-1]
        values = dict(zip(states.name, getattr(states, attribute)))
        return (
            values[self.prefix + "left_wheel_joint"],
            values[self.prefix + "right_wheel_joint"],
        )

    def assert_velocity(self, linear, angular):
        left, right = self.wheel_states("velocity")
        self.assertAlmostEqual(left, (linear - angular * 0.102 / 2) / 0.016, delta=0.02)
        self.assertAlmostEqual(right, (linear + angular * 0.102 / 2) / 0.016, delta=0.02)
        odometry = self.odometry[-1]
        self.assertAlmostEqual(odometry.twist.twist.linear.x, linear, delta=0.003)
        self.assertAlmostEqual(odometry.twist.twist.angular.z, angular, delta=0.02)

    def test_motion_and_command_timeout(self, prefix):
        self.prefix = prefix
        rclpy.init()
        self.node = rclpy.create_node("kabot_mock_test")
        self.states = deque(maxlen=1)
        self.odometry = deque(maxlen=1)
        self.node.create_subscription(JointState, "/joint_states", self.states.append, 10)
        self.node.create_subscription(
            Odometry, "/kabot_base_controller/odom", self.odometry.append, 10
        )
        self.publisher = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        buffer = Buffer()
        listener = TransformListener(buffer, self.node)
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=15))
            deadline = time.monotonic() + 20.0
            controllers = {}
            while time.monotonic() < deadline:
                future = client.call_async(ListControllers.Request())
                self.wait_until(future.done)
                controllers = {item.name: item.state for item in future.result().controller}
                if controllers == {
                    "joint_state_broadcaster": "active", "kabot_base_controller": "active"
                }:
                    break
            self.assertEqual(
                controllers,
                {"joint_state_broadcaster": "active", "kabot_base_controller": "active"},
            )
            child_frames = [
                "base_link", "chassis", "front_slider", "back_slider", "top",
                "left_wheel", "right_wheel",
            ]
            self.wait_until(
                lambda: self.states and self.odometry
                and self.publisher.get_subscription_count() == 1
                and all(
                    buffer.can_transform(prefix + "odom", prefix + child, Time())
                    for child in child_frames
                )
            )
            self.assertEqual(self.node.count_publishers("/joint_states"), 1)
            self.assertNotIn("joint_state_publisher", self.node.get_node_names())
            self.assertNotIn("joint_state_publisher_gui", self.node.get_node_names())
            self.assertEqual(
                set(self.states[-1].name),
                {prefix + "left_wheel_joint", prefix + "right_wheel_joint"},
            )
            self.assertEqual(self.odometry[-1].header.frame_id, prefix + "odom")
            self.assertEqual(self.odometry[-1].child_frame_id, prefix + "base_link")
            self.assert_velocity(0.0, 0.0)

            initial_position = self.odometry[-1].pose.pose.position.x
            initial_wheels = self.wheel_states("position")
            self.command_for(0.05, 0.0)
            self.assert_velocity(0.05, 0.0)
            self.assertGreater(self.odometry[-1].pose.pose.position.x, initial_position + 0.025)
            self.assertAlmostEqual(self.odometry[-1].pose.pose.position.y, 0.0, delta=0.001)
            for current, initial in zip(self.wheel_states("position"), initial_wheels):
                self.assertGreater(current, initial + 1.5)
            forward_position = self.odometry[-1].pose.pose.position.x
            self.command_for(-0.05, 0.0)
            self.assert_velocity(-0.05, 0.0)
            self.assertLess(self.odometry[-1].pose.pose.position.x, forward_position - 0.025)

            self.command_for(0.0, 0.5)
            self.assert_velocity(0.0, 0.5)
            rotation = self.odometry[-1].pose.pose.orientation
            self.assertGreater(2 * math.atan2(rotation.z, rotation.w), 0.25)

            self.wait_until(
                lambda: all(abs(value) < 1e-6 for value in self.wheel_states("velocity"))
                and abs(self.odometry[-1].twist.twist.angular.z) < 1e-6,
                timeout=2.0,
            )
            self.assert_velocity(0.0, 0.0)
            stopped_wheels = self.wheel_states("position")
            stopped_pose = self.odometry[-1].pose.pose
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            for actual, expected in zip(self.wheel_states("position"), stopped_wheels):
                self.assertAlmostEqual(actual, expected, delta=1e-6)
            self.assertEqual(self.odometry[-1].pose.pose, stopped_pose)
        finally:
            listener.unregister()
            self.node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)