import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration


def main():
    workspace = Path(__file__).resolve().parents[2]
    firmware = workspace / "build/native_sim_zenbedded/zephyr/zephyr.exe"
    sys.path.insert(0, str(workspace / "scripts/kabot_io"))
    from proto_codec import decode_bonjour_response, encode_bonjour

    for port in (30010, 30012):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", port))
    with socket.create_connection(("127.0.0.1", 7447), timeout=2):
        pass

    rclpy.init()
    node = rclpy.create_node("kabot_native_motor_check")
    publisher = node.create_publisher(TwistStamped, "/cmd_vel", 1)
    odometry = []
    node.create_subscription(Odometry, "/kabot_base_controller/odom", odometry.append, 10)
    processes = []

    with tempfile.TemporaryDirectory(prefix="kabot-motors-") as directory:
        firmware_log = Path(directory) / "firmware.log"
        ros_log = Path(directory) / "ros.log"
        firmware_output = firmware_log.open("w")
        ros_output = ros_log.open("w")

        def pump(duration, linear=None, angular=0.0, stale=False):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                assert all(process.poll() is None for process in processes), "A test process exited"
                if linear is not None:
                    message = TwistStamped()
                    stamp = node.get_clock().now()
                    if stale:
                        stamp = stamp - Duration(seconds=2)
                    message.header.stamp = stamp.to_msg()
                    message.twist.linear.x = linear
                    message.twist.angular.z = angular
                    publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)

        def claim(enabled):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as discovery:
                discovery.settimeout(2)
                discovery.sendto(
                    encode_bonjour(30011, claim=enabled, release=not enabled),
                    ("127.0.0.1", 30012),
                )
                response, _ = discovery.recvfrom(1024)
                assert decode_bonjour_response(response).is_claimed == enabled

        def efforts(text, side):
            return [float(value) for value in re.findall(
                rf"sim motor 'sim_{side}' effort=(-?[0-9.]+)", text
            )]

        def check_pair(text, left_value, right_value):
            for side, expected in (("left", left_value), ("right", right_value)):
                values = efforts(text, side)
                assert values and math.isclose(values[-1], expected, abs_tol=0.001), (side, values)

        def check_motion(linear, angular):
            check_pair(firmware_log.read_text(),
                       max(-1.0, min(1.0, (linear - angular * 0.102 / 2) / 0.016)),
                       max(-1.0, min(1.0, (linear + angular * 0.102 / 2) / 0.016)))
            assert odometry, "No open-loop odometry received"
            assert odometry[-1].header.frame_id == "odom"
            assert odometry[-1].child_frame_id == "base_link"
            assert math.isclose(odometry[-1].twist.twist.linear.x, linear, abs_tol=0.0001)
            assert math.isclose(odometry[-1].twist.twist.angular.z, angular, abs_tol=0.0001)

        try:
            processes.append(subprocess.Popen(
                [str(firmware)], stdin=subprocess.PIPE, stdout=firmware_output,
                stderr=subprocess.STDOUT, start_new_session=True, cwd=directory,
            ))
            deadline = time.monotonic() + 10
            while "ZenbeddedClient initialized" not in firmware_log.read_text():
                assert time.monotonic() < deadline, "Firmware did not initialize"
                pump(0.05)
            processes.append(subprocess.Popen(
                ["ros2", "launch", "kabot_robot", "native_sim.launch.py",
                 "calibrated:=false",
                 f"schema_path:={workspace / 'app/config/zenbedded_native_sim.yaml'}"],
                stdout=ros_output, stderr=subprocess.STDOUT, start_new_session=True,
            ))
            deadline = time.monotonic() + 20
            while not publisher.get_subscription_count() or not odometry:
                assert time.monotonic() < deadline, "Motor controllers did not start"
                pump(0.05)
            pump(0.5)

            pump(0.6, 0.005)
            for side in ("left", "right"):
                assert not efforts(firmware_log.read_text(), side), "Unclaimed motor received effort"
            claim(True)
            pump(0.7, 0.005)
            check_motion(0.005, 0.0)
            pump(0.7, -0.005)
            check_motion(-0.005, 0.0)
            print("PASS: claim gate, forward/reverse effort and open-loop odometry", flush=True)

            pump(0.7, 0.0, 0.05)
            check_motion(0.0, 0.05)
            pump(0.7, 1.0, 1.0)
            check_motion(1.0, 1.0)
            print("PASS: differential turning and effort saturation", flush=True)

            pump(0.8, 0.005, stale=True)
            check_motion(0.0, 0.0)
            print("PASS: expired TwistStamped commands cannot sustain motion", flush=True)

            pump(0.6, 0.005)
            pump(0.8)
            check_motion(0.0, 0.0)
            pump(0.6, 0.005)
            claim(False)
            pump(0.6, 0.005)
            check_pair(firmware_log.read_text(), 0.0, 0.0)
            print("PASS: publisher timeout and release stop both motors", flush=True)

            claim(True)
            pump(0.6, 0.005)
            check_motion(0.005, 0.0)
            ros_process = processes.pop()
            os.killpg(ros_process.pid, signal.SIGKILL)
            ros_process.wait(timeout=5)
            pump(0.8)
            check_pair(firmware_log.read_text(), 0.0, 0.0)
            claim(False)
            print("PASS: firmware watchdog stops motors after ROS process loss", flush=True)
        except Exception:
            print(ros_log.read_text()[-8000:])
            print(firmware_log.read_text()[-8000:])
            raise
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
                if process.stdin:
                    process.stdin.close()
            firmware_output.close()
            ros_output.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()