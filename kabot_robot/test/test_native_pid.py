from collections import deque
import importlib.util
import math
import os
from pathlib import Path
import signal
import tempfile
import time
from types import SimpleNamespace
import unittest
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
import xacro
import yaml


NAMESPACE = "/kabot_pid_test"
FEEDBACK = {"left_wheel_joint": 1.0, "right_wheel_joint": -2.0}


def generate_test_description():
    package = Path(get_package_share_directory("kabot_robot"))
    robot = ET.fromstring(xacro.process_file(
        str(package / "urdf/kabot_native_sim.urdf.xacro"),
        mappings={"schema_path": "/unused-by-mock-hardware"},
    ).toxml())
    control = robot.find("ros2_control")
    hardware = control.find("hardware")
    hardware.clear()
    ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"
    ET.SubElement(hardware, "param", name="disable_commands").text = "true"
    for joint in control.findall("joint"):
        assert [item.attrib["name"] for item in joint.findall("command_interface")] == ["effort"]
        for interface in joint.findall("state_interface"):
            initial = ET.SubElement(interface, "param", name="initial_value")
            initial.text = str(FEEDBACK[joint.attrib["name"]] if interface.attrib["name"] == "velocity" else 0.0)
    config = yaml.safe_load((package / "config/kabot_native_sim_controllers.yaml").read_text())
    closed_loop = os.environ.get("KABOT_TEST_CONTROL_MODE") == "pid"
    drive = config["kabot_base_controller"]["ros__parameters"]
    pid = config["kabot_wheel_pid"]["ros__parameters"]
    assert drive["open_loop"] is True
    assert pid["set_current_state_as_first_setpoint"] is False
    for gains in pid["gains"].values():
        assert gains["p"] == gains["i"] == gains["d"] == 0.0
        assert gains["feedforward_gain"] == 1.0
    if closed_loop:
        drive["open_loop"] = False
        pid["set_current_state_as_first_setpoint"] = True
        for gains in pid["gains"].values():
            gains["p"] = 0.5
            gains["feedforward_gain"] = 0.0
    directory = tempfile.TemporaryDirectory(prefix="kabot-pid-")
    config_path = Path(directory.name) / "controllers.yaml"
    config_path.write_text(yaml.safe_dump({NAMESPACE + "/" + name: value for name, value in config.items()}))
    return LaunchDescription([
        Node(
            package="robot_state_publisher", executable="robot_state_publisher", namespace=NAMESPACE,
            parameters=[{"robot_description": ET.tostring(robot, encoding="unicode")}],
        ),
        Node(
            package="controller_manager", executable="ros2_control_node", name="controller_manager",
            namespace=NAMESPACE, parameters=[str(config_path)],
        ),
        Node(
            package="controller_manager", executable="spawner", namespace=NAMESPACE,
            arguments=[
                "kabot_state_broadcaster", "kabot_wheel_pid", "kabot_base_controller", "--activate-as-group",
                "--controller-manager", NAMESPACE + "/controller_manager",
                "--param-file", str(config_path),
                "--controller-ros-args", "-r ~/cmd_vel:=" + NAMESPACE + "/cmd_vel",
            ],
        ),
        Node(
            package="twist_stamper", executable="twist_stamper", namespace=NAMESPACE,
            parameters=[{"frame_id": "base_link"}],
            remappings=[
                ("cmd_vel_in", NAMESPACE + "/cmd_vel_unstamped"),
                ("cmd_vel_out", NAMESPACE + "/cmd_vel"),
            ],
        ),
        ReadyToTest(),
    ]), {"directory": directory, "closed_loop": closed_loop}


class TestNativePid(unittest.TestCase):
    def test_calibration_script(self, closed_loop):
        if closed_loop:
            return
        spec = importlib.util.spec_from_file_location(
            "calibrate_motion", Path(__file__).resolve().parents[2] / "scripts/calibrate-motion.py")
        calibration = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(calibration)
        paths = ("cmd_vel", "cmd_vel_unstamped", "kabot_state_broadcaster/names", "kabot_state_broadcaster/values",
                 "kabot_wheel_pid/controller_state", "kabot_base_controller/get_parameters",
                 "kabot_wheel_pid/get_parameters", "controller_manager/list_controllers")
        remappings = [argument for path in paths for argument in ("-r", f"/{path}:={NAMESPACE}/{path}")]
        rclpy.init()
        node = rclpy.create_node("calibration_check", namespace=NAMESPACE, cli_args=["--ros-args", *remappings])
        control = calibration.Calibration(node, "127.0.0.1")
        node.create_timer(0.1, lambda: setattr(control, "heartbeat_at", time.monotonic()))
        try:
            deadline = time.monotonic() + 20
            while not control.outputs:
                self.assertLess(time.monotonic(), deadline, "Controller state did not start")
                control.pump(0.1)
            parameters = control.parameters()
            self.assertIsNone(parameters["linear.x.max_velocity"])
            control.pump(0.5)
            control.check_health()
            control.publisher = node.create_publisher(Twist, "/cmd_vel_unstamped", 1)
            control.pump(0.5)

            def discovery(**flags):
                return SimpleNamespace(is_claimed=flags.get("claim", False))

            with patch.object(control, "discovery", side_effect=discovery) as discovery_mock, patch.object(signal, "signal"):
                for command, expected in (((0.016, 0.0), (1.0, 1.0)), ((0.0, 0.32), (-1.0, 1.0)),
                                          ((0.0, -0.32), (1.0, -1.0)),
                                          ((0.016, -0.01), (1.0, 0.968125)),
                                          ((0.016, 0.01), (0.968125, 1.0))):
                    control.samples.clear()
                    report = {}
                    calibration.run_trial(control, 0.6, command, report)
                    self.assertTrue(report["completed"] and report["released"])
                    self.assertGreaterEqual(report["command_elapsed_s"], 0.6)
                    self.assertLess(report["command_elapsed_s"], 0.8)
                    matching = [sample for sample in control.samples if all(
                        math.isclose(sample["effort"][wheel], effort, abs_tol=1e-4)
                        for wheel, effort in zip(calibration.WHEELS, expected))]
                    self.assertGreaterEqual(len(matching), 3)
                    self.assertTrue(all(abs(value) < 1e-6 for value in control.outputs.values()))
                self.assertEqual(discovery_mock.call_count, 15)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_control_chain_and_odometry(self, closed_loop):
        rclpy.init()
        node = rclpy.create_node("pid_check", namespace=NAMESPACE)
        states = deque(maxlen=20)
        odometry = deque(maxlen=20)
        stamped_commands = []
        node.create_subscription(MultiDOFStateStamped, NAMESPACE + "/kabot_wheel_pid/controller_state", states.append, 10)
        node.create_subscription(Odometry, NAMESPACE + "/kabot_base_controller/odom", odometry.append, 10)
        node.create_subscription(
            TwistStamped, NAMESPACE + "/cmd_vel", stamped_commands.append, 10,
        )
        publisher = node.create_publisher(Twist, NAMESPACE + "/cmd_vel_unstamped", 10)

        def receive(duration, linear=None, angular=0.0):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if linear is not None:
                    message = Twist()
                    message.linear.x = linear
                    message.angular.z = angular
                    publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)

        def check(linear, angular):
            self.assertTrue(stamped_commands)
            self.assertEqual(stamped_commands[-1].header.frame_id, "base_link")
            self.assertGreater(stamped_commands[-1].header.stamp.sec, 0)
            self.assertTrue(states)
            values = {state.name: state for state in states[-1].dof_states}
            self.assertEqual(set(values), set(FEEDBACK))
            for name, direction in (("left_wheel_joint", -1), ("right_wheel_joint", 1)):
                reference = (linear + direction * angular * 0.102 / 2) / 0.016
                error = reference - FEEDBACK[name]
                self.assertAlmostEqual(values[name].reference, reference, delta=0.0001)
                self.assertAlmostEqual(values[name].feedback, FEEDBACK[name], delta=0.0001)
                self.assertAlmostEqual(values[name].error, error, delta=0.0001)
                output = max(-1.0, min(1.0, 0.5 * error if closed_loop else reference))
                self.assertAlmostEqual(values[name].output, output, delta=0.0001)
            self.assertTrue(odometry)
            self.assertAlmostEqual(odometry[-1].twist.twist.linear.x, -0.008 if closed_loop else linear, delta=0.0001)
            self.assertAlmostEqual(odometry[-1].twist.twist.angular.z, -0.048 / 0.102 if closed_loop else angular, delta=0.0001)

        try:
            deadline = time.monotonic() + 20
            while not states or not odometry or publisher.get_subscription_count() != 1:
                self.assertLess(time.monotonic(), deadline, "PID chain did not start")
                receive(0.1)
            receive(0.5, 0.0)
            check(0.0, 0.0)
            first_stamp = stamped_commands[-1].header.stamp
            initial = odometry[-1]
            for linear, angular in ((0.005, 0.0), (-0.005, 0.0), (0.0, 0.05)):
                receive(0.7, linear, angular)
                self.assertAlmostEqual(stamped_commands[-1].twist.linear.x, linear)
                self.assertAlmostEqual(stamped_commands[-1].twist.angular.z, angular)
                check(linear, angular)
            last_stamp = stamped_commands[-1].header.stamp
            self.assertGreater(
                last_stamp.sec * 1000000000 + last_stamp.nanosec,
                first_stamp.sec * 1000000000 + first_stamp.nanosec,
            )
            receive(0.7)
            check(0.0, 0.0)
            final = odometry[-1]
            elapsed = final.header.stamp.sec - initial.header.stamp.sec + (
                final.header.stamp.nanosec - initial.header.stamp.nanosec
            ) * 1e-9
            initial_yaw = 2 * math.atan2(initial.pose.pose.orientation.z, initial.pose.pose.orientation.w)
            final_yaw = 2 * math.atan2(final.pose.pose.orientation.z, final.pose.pose.orientation.w)
            yaw_delta = math.remainder(final_yaw - initial_yaw, 2 * math.pi)
            if closed_loop:
                self.assertAlmostEqual(yaw_delta, elapsed * -0.048 / 0.102, delta=0.01)
                self.assertGreater(math.hypot(
                    final.pose.pose.position.x - initial.pose.pose.position.x,
                    final.pose.pose.position.y - initial.pose.pose.position.y,
                ), 0.01)
            else:
                self.assertGreater(yaw_delta, 0.015)
                commands_before_idle = len(stamped_commands)
                receive(0.4)
                self.assertEqual(len(stamped_commands), commands_before_idle)
                check(0.0, 0.0)
                self.assertEqual(odometry[-1].pose.pose, final.pose.pose)
                for linear, angular in ((0.016, 0.0), (0.0, 0.32), (0.0, -0.32),
                                        (1.0, 1.0), (-1.0, -1.0)):
                    receive(0.7, linear, angular)
                    check(linear, angular)
                receive(0.7)
                check(0.0, 0.0)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, directory):
        directory.cleanup()
        launch_testing.asserts.assertExitCodes(proc_info)