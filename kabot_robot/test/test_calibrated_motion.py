from collections import deque
import importlib.util
import math
from pathlib import Path
import tempfile
import signal
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import MultiDOFStateStamped
from geometry_msgs.msg import Twist, TwistStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import xacro
import yaml


NAMESPACE = "/kabot_calibrated_test"


def generate_test_description():
    package = Path(get_package_share_directory("kabot_robot"))
    robot = ET.fromstring(xacro.process_file(
        str(package / "urdf/kabot_native_sim.urdf.xacro"), mappings={"schema_path": "/unused-by-mock"}).toxml())
    hardware = robot.find("ros2_control/hardware")
    hardware.clear()
    ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"
    ET.SubElement(hardware, "param", name="disable_commands").text = "true"
    for joint in robot.findall("ros2_control/joint"):
        for interface in joint.findall("state_interface"):
            ET.SubElement(interface, "param", name="initial_value").text = "7.0"
    config = yaml.safe_load((package / "config/kabot_calibrated_controllers.yaml").read_text())
    directory = tempfile.TemporaryDirectory(prefix="kabot-calibrated-")
    path = Path(directory.name) / "controllers.yaml"
    path.write_text(yaml.safe_dump({NAMESPACE + "/" + name: value for name, value in config.items()}))
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=NAMESPACE,
             parameters=[{"robot_description": ET.tostring(robot, encoding="unicode")}],
             remappings=[("/tf", NAMESPACE + "/tf"), ("/tf_static", NAMESPACE + "/tf_static")]),
        Node(package="controller_manager", executable="ros2_control_node", name="controller_manager",
             namespace=NAMESPACE, parameters=[str(path)], remappings=[("/tf", NAMESPACE + "/tf")]),
        Node(package="controller_manager", executable="spawner", namespace=NAMESPACE,
             arguments=["kabot_state_broadcaster", "kabot_motion_model", "kabot_base_controller",
                        "--activate-as-group", "--controller-manager", NAMESPACE + "/controller_manager",
                        "--param-file", str(path), "--controller-ros-args",
                        "-r ~/cmd_vel:=" + NAMESPACE + "/cmd_vel -r /tf:=" + NAMESPACE + "/tf"]),
        Node(package="twist_stamper", executable="twist_stamper", namespace=NAMESPACE,
             parameters=[{"frame_id": "base_link"}], remappings=[
                 ("cmd_vel_in", NAMESPACE + "/cmd_vel_unstamped"), ("cmd_vel_out", NAMESPACE + "/cmd_vel")]),
        ReadyToTest(),
    ]), {"directory": directory}


class TestCalibratedMotion(unittest.TestCase):
    def test_physical_units_and_model_odometry(self):
        rclpy.init()
        paths = ("cmd_vel", "cmd_vel_unstamped", "kabot_state_broadcaster/names", "kabot_state_broadcaster/values",
             "kabot_motion_model/controller_state", "kabot_base_controller/get_parameters",
             "kabot_motion_model/get_parameters", "controller_manager/list_controllers")
        remappings = [argument for path in paths for argument in ("-r", f"/{path}:={NAMESPACE}/{path}")]
        node = rclpy.create_node("calibrated_check", namespace=NAMESPACE, cli_args=["--ros-args", *remappings])
        states, odometry, joints, transforms = (deque(maxlen=100) for unused in range(4))
        node.create_subscription(MultiDOFStateStamped, NAMESPACE + "/kabot_motion_model/controller_state", states.append, 10)
        node.create_subscription(Odometry, NAMESPACE + "/kabot_base_controller/odom", odometry.append, 10)
        node.create_subscription(JointState, NAMESPACE + "/joint_states", joints.append, 10)
        node.create_subscription(TFMessage, NAMESPACE + "/tf", transforms.append, 10)
        publisher = node.create_publisher(Twist, NAMESPACE + "/cmd_vel_unstamped", 1)

        def receive(duration, linear=None, angular=0):
            deadline = time.monotonic() + duration
            next_publish = 0
            while time.monotonic() < deadline:
                if linear is not None and time.monotonic() >= next_publish:
                    message = Twist()
                    message.linear.x = float(linear)
                    message.angular.z = float(angular)
                    publisher.publish(message)
                    next_publish = time.monotonic() + 0.04
                rclpy.spin_once(node, timeout_sec=0.02)

        def check(linear, angular, efforts):
            values = {value.name: value for value in states[-1].dof_states}
            self.assertEqual(set(joints[-1].name), {"left_wheel_joint", "right_wheel_joint"})
            for wheel, direction, expected in zip(("left_wheel_joint", "right_wheel_joint"), (-1, 1), efforts):
                self.assertAlmostEqual(values[wheel].output, expected, delta=1e-6)
                velocity = (linear + direction * angular * .102 / 2) / .016
                self.assertAlmostEqual(values[wheel].feedback, velocity, delta=1e-6)
                index = joints[-1].name.index(wheel)
                self.assertAlmostEqual(joints[-1].velocity[index], velocity, delta=1e-6)
                self.assertAlmostEqual(joints[-1].effort[index], expected, delta=1e-6)
            self.assertAlmostEqual(odometry[-1].twist.twist.linear.x, linear, delta=1e-6)
            self.assertAlmostEqual(odometry[-1].twist.twist.angular.z, angular, delta=1e-6)
            world_transforms = {(item.header.stamp.sec, item.header.stamp.nanosec): item.transform
                                for batch in transforms for item in batch.transforms
                                if item.header.frame_id == "odom" and item.child_frame_id == "base_link"}
            matched = [message for message in odometry
                       if (message.header.stamp.sec, message.header.stamp.nanosec) in world_transforms]
            self.assertTrue(matched, "No TF and odometry with matching timestamps")
            for message in matched:
                self.assertEqual(message.header.frame_id, "odom")
                self.assertEqual(message.child_frame_id, "base_link")
                transform = world_transforms[(message.header.stamp.sec, message.header.stamp.nanosec)]
                for axis in ("x", "y", "z"):
                    self.assertAlmostEqual(getattr(transform.translation, axis), getattr(message.pose.pose.position, axis))
                self.assertEqual(transform.rotation, message.pose.pose.orientation)

        try:
            deadline = time.monotonic() + 20
            while not states or not odometry or not joints or not transforms or publisher.get_subscription_count() != 1:
                self.assertLess(time.monotonic(), deadline, "Calibrated chain did not start")
                receive(.1)
            receive(.6, 0)
            check(0, 0, (0, 0))
            for linear, angular, efforts in ((.575 / 17.05, 0, (.75478125, .74521875)),
                                             (1.265 / 18.59, 0, (1, .9611125)),
                                             (0, 10 * math.pi / 22.9, (-1, 1)),
                                             (0, -10 * math.pi / 22.93, (1, -1)),
                                             (-.575 / 17.05, 0, (-.75478125, -.74521875))):
                receive(.8, linear, angular)
                check(linear, angular, efforts)
            receive(.8, 0)
            initial = odometry[-1]
            receive(1.2, 1.265 / 18.59)
            receive(.8)
            check(0, 0, (0, 0))
            travelled = math.hypot(odometry[-1].pose.pose.position.x - initial.pose.pose.position.x,
                                   odometry[-1].pose.pose.position.y - initial.pose.pose.position.y)
            self.assertGreater(travelled, .07)
            self.assertLess(travelled, .12)
            held_pose = odometry[-1].pose.pose
            held_joints = list(joints[-1].position)
            receive(.4)
            self.assertEqual(odometry[-1].pose.pose, held_pose)
            self.assertEqual(list(joints[-1].position), held_joints)
            receive(.8, .5, 3)
            actual = odometry[-1].twist.twist
            self.assertGreater(actual.linear.x, 0)
            self.assertLess(actual.linear.x, .5)
            self.assertAlmostEqual(actual.angular.z / actual.linear.x, 6, delta=1e-5)
            self.assertTrue(all(abs(state.output) <= 1 for state in states[-1].dof_states))
            receive(.8)
            check(0, 0, (0, 0))
            stamped = node.create_publisher(TwistStamped, NAMESPACE + "/cmd_vel", 1)
            stale = TwistStamped()
            stale.header.stamp = (node.get_clock().now() - Duration(seconds=2)).to_msg()
            stale.twist.linear.x = .06
            stamped.publish(stale)
            receive(.6)
            check(0, 0, (0, 0))
            edges = {(item.header.frame_id, item.child_frame_id) for batch in transforms for item in batch.transforms}
            self.assertIn(("odom", "base_link"), edges)
            self.assertIn(("base_link", "left_wheel"), edges)
            self.assertIn(("base_link", "right_wheel"), edges)
            node.destroy_publisher(stamped)
            node.destroy_publisher(publisher)
            spec = importlib.util.spec_from_file_location(
                "drive_loop", Path(__file__).resolve().parents[2] / "scripts/drive-loop.py")
            route = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(route)
            control = route.calibration.Calibration(node, "127.0.0.1", calibrated=True)
            node.create_timer(.1, lambda: setattr(control, "heartbeat_at", time.monotonic()))
            control.pump(.6)
            parameters = control.parameters()
            self.assertFalse(parameters["drive"]["open_loop"])
            control.check_health()
            control.publisher = node.create_publisher(Twist, "/cmd_vel_unstamped", 1)
            control.pump(.5)
            start = odometry[-1].pose.pose
            start_yaw = 2 * math.atan2(start.orientation.z, start.orientation.w)
            report = {}
            with patch.object(control, "discovery", side_effect=lambda **flags: SimpleNamespace(
                    is_claimed=flags.get("claim", False))), patch.object(signal, "signal"):
                route.calibration.run_segments(control, route.make_plan(), report)
            self.assertTrue(report["completed"] and report["released"])
            self.assertEqual(len(report["segments"]), 6)
            receive(.3)
            check(0, 0, (0, 0))
            end = odometry[-1].pose.pose
            self.assertLess(math.hypot(end.position.x - start.position.x, end.position.y - start.position.y), .05)
            end_yaw = 2 * math.atan2(end.orientation.z, end.orientation.w)
            self.assertLess(abs(math.remainder(end_yaw - start_yaw, math.tau)), .3)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, directory):
        directory.cleanup()
        launch_testing.asserts.assertExitCodes(proc_info)