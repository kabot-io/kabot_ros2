import argparse
import fcntl
import math
import os
from pathlib import Path
import re
import signal
import socket
import struct
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration


def main():
    parser = argparse.ArgumentParser(description="Short motor tests on a secured, lifted robot.")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--serial-port", default="/dev/ttyACM1")
    parser.add_argument("--wheels-lifted", action="store_true", required=True)
    parser.add_argument("--controller-pid", type=int)
    arguments = parser.parse_args()
    workspace = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(workspace / "scripts/kabot_io"))
    from proto_codec import decode_bonjour_response, encode_bonjour

    if arguments.controller_pid:
        command = Path(f"/proc/{arguments.controller_pid}/cmdline").read_bytes()
        assert b"ros2_control_node" in command, "PID is not the ROS controller"

    def discovery(claim=False, release=False):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
            connection.settimeout(2)
            connection.sendto(encode_bonjour(30011, claim=claim, release=release),
                              (arguments.robot_ip, 30012))
            payload, sender = connection.recvfrom(1024)
        assert sender[0] == arguments.robot_ip, sender
        response = decode_bonjour_response(payload)
        if claim or release:
            assert response.is_claimed == claim, response
        return response

    response = discovery()
    assert not response.is_claimed, "Robot is already claimed; refusing takeover"
    print(f"Robot: {response.serial}; firmware: {response.firmware_version}; unclaimed", flush=True)

    rclpy.init()
    node = rclpy.create_node("kabot_hardware_motor_check")
    publisher = node.create_publisher(TwistStamped, "/cmd_vel", 1)
    console = os.open(arguments.serial_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original_settings = termios.tcgetattr(console)
    tty.setraw(console)
    settings = termios.tcgetattr(console)
    settings[4] = settings[5] = termios.B115200
    settings[2] |= termios.CLOCAL | termios.CREAD
    settings[2] &= ~termios.CRTSCTS
    termios.tcsetattr(console, termios.TCSANOW, settings)
    fcntl.ioctl(console, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
    termios.tcflush(console, termios.TCIFLUSH)
    pending = ""
    samples = []
    events = []
    claim_attempted = False
    controller_paused = False
    encoder_pattern = re.compile(
        r"Sensor tuple: left=\(ts=\d+,value=([-0-9.]+)\) "
        r"right=\(ts=\d+,value=([-0-9.]+)\)"
    )

    def read_console():
        nonlocal pending
        try:
            data = os.read(console, 65536)
        except BlockingIOError:
            return
        if not data:
            raise RuntimeError("UART disconnected")
        pending += data.decode("utf-8", errors="replace")
        lines = pending.split("\n")
        pending = lines.pop()
        for line in lines:
            line = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", line)
            match = encoder_pattern.search(line)
            if match:
                pair = tuple(float(value) for value in match.groups())
                assert all(math.isfinite(value) for value in pair), pair
                samples.append(pair)
            elif "Control watchdog:" in line or "<err>" in line or "Failed to set motor" in line:
                events.append(line.strip())
                print(line.strip(), flush=True)

    def publish(linear, angular=0.0, stale=False):
        message = TwistStamped()
        stamp = node.get_clock().now()
        if stale:
            stamp -= Duration(seconds=2)
        message.header.stamp = stamp.to_msg()
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        publisher.publish(message)

    def pump(duration, linear=None, angular=0.0, stale=False):
        start = len(samples)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if linear is not None:
                publish(linear, angular, stale)
            rclpy.spin_once(node, timeout_sec=0.04)
            read_console()
        return samples[start:]

    def stationary(batch):
        assert len(batch) >= 5, f"Insufficient encoder samples: {batch}"
        tail = batch[-4:]
        for side in range(2):
            values = [pair[side] for pair in tail]
            assert max(values) - min(values) < 0.001, f"Encoder still changing: {tail}"

    def motion(label, linear, angular=0.0):
        baseline = samples[-1]
        batch = pump(1.0, linear, angular)
        print(f"{label}: encoder {baseline} -> {batch[-1] if batch else None}", flush=True)
        assert len(batch) >= 5, "Missing live encoder samples"
        for side in range(2):
            assert any(abs(pair[side] - baseline[side]) > 0.001 for pair in batch), (
                f"{label}: encoder {side} did not change", batch
            )
        assert not any("Failed to set motor" in event or "<err>" in event for event in events)
        return batch

    try:
        deadline = time.monotonic() + 5
        while not publisher.get_subscription_count():
            assert time.monotonic() < deadline, "No /cmd_vel subscriber"
            pump(0.1)
        assert publisher.get_subscription_count() == 1, "Multiple /cmd_vel subscribers"
        os.write(console, b"\x15log enable inf sensor_subscriber\r")
        pump(1.0, 0.0)
        baseline = pump(1.0, 0.0)
        stationary(baseline)
        unclaimed = pump(0.8, 0.005)
        stationary(unclaimed)
        assert all(abs(pair[side] - baseline[-1][side]) < 0.001
                   for pair in unclaimed for side in range(2)), "Motion while unclaimed"
        stationary(pump(0.8, 0.0))
        print("PASS: unclaimed gate; live stationary encoder samples", flush=True)

        claim_attempted = True
        discovery(claim=True)
        forward = motion("Forward", 0.005)
        stationary(pump(0.8, 0.0))
        reverse = motion("Reverse", -0.005)
        stationary(pump(0.8, 0.0))
        for side in range(2):
            forward_delta = forward[-1][side] - forward[0][side]
            reverse_delta = reverse[-1][side] - reverse[0][side]
            assert forward_delta * reverse_delta < 0, "Encoder direction did not reverse"
        print("PASS: forward/reverse encoder direction and zero command stop", flush=True)

        motion("Turn", 0.0, 0.08)
        stationary(pump(0.8, 0.0))
        motion("Before stale command", 0.005)
        stationary(pump(1.0, 0.005, stale=True))
        print("PASS: turning; expired TwistStamped stops encoder changes", flush=True)

        motion("Before command timeout", 0.005)
        stationary(pump(1.0))
        print("PASS: command publisher timeout stops encoder changes", flush=True)

        motion("Before release", 0.005)
        discovery(release=True)
        stationary(pump(1.0, 0.005))
        stationary(pump(0.8, 0.0))
        print("PASS: release stops motion despite continuing ROS commands", flush=True)

        if arguments.controller_pid:
            discovery(claim=True)
            motion("Before controller suspension", 0.005)
            event_start = len(events)
            os.kill(arguments.controller_pid, signal.SIGSTOP)
            controller_paused = True
            stationary(pump(1.2))
            assert any("no command in 300 ms" in event for event in events[event_start:]), events
            print("PASS: firmware watchdog after controller suspension", flush=True)
    finally:
        try:
            if claim_attempted:
                discovery(release=True)
                print("Cleanup: claim released", flush=True)
        finally:
            try:
                publish(0.0)
                if controller_paused:
                    os.kill(arguments.controller_pid, signal.SIGCONT)
                pump(0.8, 0.0)
            finally:
                termios.tcsetattr(console, termios.TCSANOW, original_settings)
                os.close(console)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    main()