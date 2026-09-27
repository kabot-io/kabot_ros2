from collections import deque
import math
import os
from pathlib import Path
import time
import unittest
import uuid

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
import xacro


def generate_test_description():
    prefix = os.environ.get("KABOT_TEST_PREFIX", "")
    launch_file = Path(get_package_share_directory("kabot_robot")) / "launch/gazebo.launch.py"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments={"gui": "false", "rviz": "false", "prefix": prefix}.items(),
    )
    actions = [
        SetEnvironmentVariable("GZ_PARTITION", "kabot_test_" + uuid.uuid4().hex),
    ]
    if os.environ.get("KABOT_TEST_FOREIGN_DESCRIPTION") == "1":
        description_file = launch_file.parent.parent / "urdf" / "kabot.urdf.xacro"
        actions.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="foreign_description_publisher",
            parameters=[{"robot_description": xacro.process_file(str(description_file)).toxml()}],
            remappings=[("/tf", "/foreign/tf"), ("/tf_static", "/foreign/tf_static")],
        ))
    actions.extend([gazebo, ReadyToTest()])
    return LaunchDescription(actions), {"prefix": prefix}


def yaw(rotation):
    return math.atan2(
        2 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1 - 2 * (rotation.y**2 + rotation.z**2),
    )


class TestGazeboControl(unittest.TestCase):
    def wait_until(self, predicate, timeout=10.0, description="simulation data"):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail(f"Timed out waiting for {description}; pose frames: {self.pose_frames}")

    def advance_sim(self, duration):
        target = self.node.get_clock().now().nanoseconds + int(duration * 1e9)
        self.wait_until(
            lambda: self.node.get_clock().now().nanoseconds >= target,
            timeout=30, description=f"{duration} seconds of simulation time",
        )

    def command_for(self, linear, angular, duration=2.0):
        def publish():
            command = TwistStamped()
            command.header.stamp = self.node.get_clock().now().to_msg()
            command.twist.linear.x = linear
            command.twist.angular.z = angular
            self.publisher.publish(command)

        publish()
        timer = self.node.create_timer(0.05, publish)
        try:
            self.advance_sim(duration)
        finally:
            self.node.destroy_timer(timer)

    def observe_pose(self, message):
        for transform in message.transforms:
            self.pose_frames.add(transform.child_frame_id)
            if transform.child_frame_id == "kabot":
                self.poses.append(transform.transform)

    def wheel_velocities(self):
        values = dict(zip(self.states[-1].name, self.states[-1].velocity))
        return [values[self.prefix + side + "_wheel_joint"] for side in ("left", "right")]

    def assert_motion(self, direction):
        initial = self.poses[-1]
        self.command_for(direction * 0.03, 0.0)
        final = self.poses[-1]
        delta_x = final.translation.x - initial.translation.x
        delta_y = final.translation.y - initial.translation.y
        forward = delta_x * math.cos(yaw(initial.rotation)) + delta_y * math.sin(yaw(initial.rotation))
        print(f"Ground truth: direction={direction}, displacement={forward:.5f} m", flush=True)
        self.assertGreater(direction * forward, 0.02)
        for velocity in self.wheel_velocities():
            self.assertAlmostEqual(velocity, direction * 0.03 / 0.016, delta=0.25)

    def test_physical_motion_and_timeout(self, prefix):
        self.prefix = prefix
        rclpy.init()
        self.node = rclpy.create_node(
            "kabot_gazebo_test", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.states = deque(maxlen=1)
        self.odometry = deque(maxlen=1)
        self.poses = deque(maxlen=100)
        self.pose_frames = set()
        self.node.create_subscription(JointState, "/joint_states", self.states.append, 10)
        self.node.create_subscription(Odometry, "/kabot_base_controller/odom", self.odometry.append, 10)
        self.node.create_subscription(TFMessage, "/simulation/ground_truth", self.observe_pose, 10)
        self.publisher = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        buffer = Buffer()
        listener = TransformListener(buffer, self.node)
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=35))
            controllers = {}
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                future = client.call_async(ListControllers.Request())
                self.wait_until(future.done)
                controllers = {item.name: item.state for item in future.result().controller}
                if controllers == {"joint_state_broadcaster": "active", "kabot_base_controller": "active"}:
                    break
            self.assertEqual(controllers, {
                "joint_state_broadcaster": "active", "kabot_base_controller": "active"
            })
            self.wait_until(
                lambda: self.node.get_clock().now().nanoseconds > 0
                and self.states and self.odometry and self.publisher.get_subscription_count() == 1,
                description="clock, wheel feedback, odometry and command subscription",
            )
            self.wait_until(
                lambda: all(buffer.can_transform(prefix + "odom", prefix + child, Time()) for child in (
                    "base_link", "chassis", "top", "front_slider", "back_slider",
                    "left_wheel", "right_wheel",
                )),
                description="complete odom-to-robot TF tree",
            )
            self.wait_until(lambda: self.poses, description="Gazebo model pose")
            self.assertEqual(self.node.count_publishers("/joint_states"), 1)
            self.assertEqual(self.node.count_publishers("/simulation/robot_description"), 1)
            self.assertEqual(self.node.get_node_names().count("controller_manager"), 1)
            self.assertNotIn("joint_state_publisher", self.node.get_node_names())
            self.assertNotIn("joint_state_publisher_gui", self.node.get_node_names())
            self.assertEqual(set(self.states[-1].name), {
                prefix + "left_wheel_joint", prefix + "right_wheel_joint"
            })
            self.assertTrue(all(math.isfinite(value) for value in self.states[-1].position))
            self.assertEqual(self.odometry[-1].header.frame_id, prefix + "odom")
            self.assertEqual(self.odometry[-1].child_frame_id, prefix + "base_link")
            for name in ("controller_manager", "joint_state_broadcaster",
                         "kabot_base_controller", "simulation_robot_state_publisher"):
                parameters = AsyncParameterClient(self.node, name)
                self.assertTrue(parameters.wait_for_services(timeout_sec=5))
                future = parameters.get_parameters(["use_sim_time"])
                self.wait_until(future.done)
                self.assertTrue(future.result().values[0].bool_value, name)

            self.command_for(0.0, 0.0)
            heights = [pose.translation.z for pose in list(self.poses)[-50:]]
            self.assertGreater(len(heights), 25)
            self.assertLess(max(heights) - min(heights), 0.002)
            self.assertGreater(min(heights), 0.005)
            self.assertLess(max(heights), 0.05)
            rotation = self.poses[-1].rotation
            roll = math.atan2(2 * (rotation.w * rotation.x + rotation.y * rotation.z),
                              1 - 2 * (rotation.x**2 + rotation.y**2))
            pitch = math.asin(max(-1, min(1, 2 * (rotation.w * rotation.y - rotation.z * rotation.x))))
            print(f"Settled: z={heights[-1]:.5f} m, roll={roll:.4f}, pitch={pitch:.4f}", flush=True)
            self.assertLess(abs(roll), 0.2)
            self.assertLess(abs(pitch), 0.2)
            self.assert_motion(1)
            self.assert_motion(-1)
            initial_yaw = yaw(self.poses[-1].rotation)
            self.command_for(0.0, 0.4)
            delta_yaw = math.remainder(yaw(self.poses[-1].rotation) - initial_yaw, 2 * math.pi)
            print(f"Ground truth: rotation={delta_yaw:.4f} rad", flush=True)
            self.assertGreater(delta_yaw, 0.2)
            left_velocity, right_velocity = self.wheel_velocities()
            self.assertLess(left_velocity, -0.5)
            self.assertGreater(right_velocity, 0.5)

            self.advance_sim(2.0)
            self.assertTrue(all(abs(value) < 0.1 for value in self.wheel_velocities()))
            stopped = self.poses[-1].translation
            self.advance_sim(0.5)
            final = self.poses[-1].translation
            self.assertLess(math.hypot(final.x - stopped.x, final.y - stopped.y), 0.003)
            self.assertLess(abs(self.odometry[-1].twist.twist.linear.x), 0.002)
            self.assertLess(abs(self.odometry[-1].twist.twist.angular.z), 0.03)
        finally:
            listener.unregister()
            self.node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)