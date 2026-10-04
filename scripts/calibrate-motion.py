import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import socket
import sys
import time

import rclpy
from control_msgs.msg import Float64Values, Keys, MultiDOFStateStamped
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions


WHEELS = ("left_wheel_joint", "right_wheel_joint")


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("Must be finite")
    return number


def positive(value):
    number = finite(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("Must be finite and greater than zero")
    return number


def parse_arguments(arguments=None):
    parser = argparse.ArgumentParser(description="Timed open-loop calibration; no motion without --execute.")
    parser.add_argument("motion", choices=("forward", "left", "right"))
    parser.add_argument("--host", required=True, help="Robot IP or hostname; must be unclaimed")
    parser.add_argument("--speed", required=True, type=positive, help="m/s for forward; rad/s for left/right")
    parser.add_argument("--duration", required=True, type=positive, help="Command duration in seconds")
    parser.add_argument("--angular", type=finite, help="Forward-only rotation trim in rad/s (default: 0; negative = right)")
    parser.add_argument("--execute", action="store_true", help="Claim, move after 3 seconds, stop and release")
    parser.add_argument("--output", type=Path, help="New JSON report (default: log/calibration/<timestamp>.json)")
    parsed = parser.parse_args(arguments)
    if parsed.angular is not None and parsed.motion != "forward":
        parser.error("--angular is only supported for forward; use --speed for left/right")
    if parsed.angular is None:
        parsed.angular = 0.0
    return parsed


def command_for(motion, speed, angular=0.0):
    return (speed, angular) if motion == "forward" else (0.0, speed if motion == "left" else -speed)


def predict_effort(parameters, linear, angular):
    result = {}
    for wheel, direction in zip(WHEELS, (-1, 1)):
        reference = (linear + direction * angular * parameters["wheel_separation"] / 2) / parameters["wheel_radius"]
        raw = reference * parameters[f"gains.{wheel}.feedforward_gain"]
        if not math.isfinite(reference) or not math.isfinite(raw):
            raise ValueError("Speed is too large for a finite wheel reference/effort")
        result[wheel] = {"reference_rad_s": reference, "unclamped_effort": raw,
                         "effort": max(-1.0, min(1.0, raw)), "saturated": abs(raw) > 1.0}
    return result


def run_trial(control, duration, command, report):
    run_segments(control, [{"name": "trial", "duration_s": duration,
                            "linear_m_s": command[0], "angular_rad_s": command[1]}], report)


def run_segments(control, segments, report):
    claim_attempted = False
    try:
        control.pump(3.0, monitor=True)
        control.check_unclaimed()
        control.pump(0.4, (0.0, 0.0), monitor=True)
        claim_attempted = True
        control.claim(True)
        started = time.monotonic()
        report["command_started_utc"] = datetime.now(timezone.utc).isoformat()
        report["command_started_monotonic_s"] = started
        try:
            report["segments"] = []
            for index, segment in enumerate(segments):
                entry = dict(segment, completed=False, started_monotonic_s=time.monotonic())
                report["segments"].append(entry)
                print(f"Segment {index + 1}/{len(segments)}: {segment['name']}", flush=True)
                try:
                    control.pump(segment["duration_s"], (segment["linear_m_s"], segment["angular_rad_s"]), monitor=True)
                    entry["completed"] = True
                finally:
                    entry["elapsed_s"] = time.monotonic() - entry["started_monotonic_s"]
                    control.publish((0.0, 0.0))
                if index + 1 < len(segments):
                    control.pump(0.5, (0.0, 0.0), monitor=True)
            report["completed"] = True
        finally:
            report["command_elapsed_s"] = time.monotonic() - started
    finally:
        if claim_attempted:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            errors = []
            try:
                control.publish((0.0, 0.0))
            except Exception as error:
                errors.append(f"Zero publish: {error}")
            report["released"] = False
            for attempt in range(3):
                try:
                    control.claim(False)
                    report["released"] = True
                    break
                except Exception as error:
                    if attempt == 2:
                        errors.append(f"Release unconfirmed: {error}")
            try:
                control.pump(0.5, (0.0, 0.0))
            except Exception as error:
                errors.append(f"Stop publishing: {error}")
            report["cleanup_errors"] = errors
            if errors:
                raise RuntimeError("; ".join(errors))


class Calibration:
    def __init__(self, node, host, calibrated=False):
        self.node = node
        self.calibrated = calibrated
        self.address = socket.gethostbyname(host)
        self.publisher = None
        self.names = []
        self.heartbeat = None
        self.heartbeat_at = 0.0
        self.state_at = 0.0
        self.outputs = {}
        self.samples = []
        node.create_subscription(Keys, "/kabot_state_broadcaster/names", self.receive_names,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Float64Values, "/kabot_state_broadcaster/values", self.receive_values, 10)
        controller = "kabot_motion_model" if calibrated else "kabot_wheel_pid"
        node.create_subscription(MultiDOFStateStamped, f"/{controller}/controller_state", self.receive_state, 10)

    def receive_names(self, message):
        self.names = message.keys

    def receive_values(self, message):
        if len(self.names) != len(message.values):
            return
        heartbeat = dict(zip(self.names, message.values)).get("kabot/heartbeat")
        if heartbeat is not None and math.isfinite(heartbeat) and heartbeat > 0:
            if self.heartbeat is not None and heartbeat > self.heartbeat:
                self.heartbeat_at = time.monotonic()
            self.heartbeat = heartbeat

    def receive_state(self, message):
        self.outputs = {state.name: state.output for state in message.dof_states}
        self.state_at = time.monotonic()
        self.samples.append({"monotonic_s": self.state_at,
                     "effort": {name: value if math.isfinite(value) else None
                        for name, value in self.outputs.items()}})

    def discovery(self, **flags):
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts/kabot_io"))
        from proto_codec import decode_bonjour_response, encode_bonjour

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
            connection.settimeout(1)
            connection.sendto(encode_bonjour(30011, **flags), (self.address, 30012))
            payload, sender = connection.recvfrom(1024)
        if sender != (self.address, 30012):
            raise RuntimeError(f"Unexpected robot responder: {sender}")
        return decode_bonjour_response(payload)

    def check_unclaimed(self):
        response = self.discovery()
        if response.is_claimed:
            raise RuntimeError("Robot already claimed; stop teleop/HMI and release it first")
        return response

    def claim(self, enabled):
        if self.discovery(claim=enabled, release=not enabled).is_claimed != enabled:
            raise RuntimeError("Robot rejected claim/release")

    def call(self, service_type, name, request):
        client = self.node.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError(f"Missing service: {name}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
            if not future.done() or future.result() is None:
                raise RuntimeError(f"No response: {name}")
            return future.result()
        finally:
            self.node.destroy_client(client)

    def parameters(self):
        controllers = self.call(ListControllers, "/controller_manager/list_controllers", ListControllers.Request())
        if self.calibrated:
            return self.model_parameters(controllers)
        active = {item.name for item in controllers.controller if item.state == "active"}
        if not {"kabot_base_controller", "kabot_wheel_pid", "kabot_state_broadcaster"} <= active:
            raise RuntimeError("Raw calibration requires native calibrated:=false; do not use historical speeds on the calibrated profile")
        names = {
            "kabot_base_controller": ["open_loop", "wheel_radius", "wheel_separation", "cmd_vel_timeout",
                                      "left_wheel_radius_multiplier", "right_wheel_radius_multiplier",
                                      "wheel_separation_multiplier", "linear.x.max_velocity", "linear.x.min_velocity",
                                      "angular.z.max_velocity", "angular.z.min_velocity"],
            "kabot_wheel_pid": [f"gains.{wheel}.{gain}" for wheel in WHEELS
                                for gain in ("p", "i", "d", "feedforward_gain", "u_clamp_min", "u_clamp_max")],
        }
        parameters = {}
        for controller, requested in names.items():
            response = self.call(GetParameters, f"/{controller}/get_parameters", GetParameters.Request(names=requested))
            for name, value in zip(requested, response.values):
                if value.type not in (1, 3):
                    raise RuntimeError(f"Unexpected parameter type: {name}")
                parameters[name] = value.bool_value if value.type == 1 else value.double_value
        if not parameters["open_loop"] or not 0.15 <= parameters["cmd_vel_timeout"] <= 0.3:
            raise RuntimeError("Requires open-loop control with timeout between 0.15 and 0.3 s")
        for name in names["kabot_base_controller"][-4:]:
            if not math.isnan(parameters[name]):
                raise RuntimeError("Old velocity limits still active: restart native launch before calibration")
            parameters[name] = None
        for name in ("wheel_radius", "wheel_separation"):
            if not math.isfinite(parameters[name]) or parameters[name] <= 0:
                raise RuntimeError(f"Invalid {name}")
        for name in ("left_wheel_radius_multiplier", "right_wheel_radius_multiplier", "wheel_separation_multiplier"):
            if parameters[name] != 1.0:
                raise RuntimeError("Calibration currently requires geometry multipliers = 1")
        for wheel in WHEELS:
            gains = {gain: parameters[f"gains.{wheel}.{gain}"] for gain in
                     ("p", "i", "d", "feedforward_gain", "u_clamp_min", "u_clamp_max")}
            if (not all(math.isfinite(value) for value in gains.values()) or
                    any(gains[gain] != 0.0 for gain in ("p", "i", "d")) or
                    gains["feedforward_gain"] <= 0 or gains["u_clamp_min"] != -1.0 or gains["u_clamp_max"] != 1.0):
                raise RuntimeError("Requires feedforward-only mapping and effort clamps [-1, 1]")
        return parameters

    def model_parameters(self, controllers):
        active = {item.name: item.type for item in controllers.controller if item.state == "active"}
        if (active.get("kabot_motion_model") != "kabot_robot/CalibratedMotionController" or
                active.get("kabot_base_controller") != "diff_drive_controller/DiffDriveController" or
                "kabot_state_broadcaster" not in active or "kabot_wheel_pid" in active):
            raise RuntimeError("This route requires native calibrated:=true with the calibrated controller active")
        requested = ["open_loop", "position_feedback", "cmd_vel_timeout", "wheel_radius", "wheel_separation",
                     "left_wheel_names", "right_wheel_names", "left_wheel_radius_multiplier",
                     "right_wheel_radius_multiplier", "wheel_separation_multiplier"]
        response = self.call(GetParameters, "/kabot_base_controller/get_parameters", GetParameters.Request(names=requested))
        parameters = {}
        for name, value in zip(requested, response.values):
            if value.type == 1:
                parameters[name] = value.bool_value
            elif value.type == 3:
                parameters[name] = value.double_value
            elif value.type == 9:
                parameters[name] = list(value.string_array_value)
            else:
                raise RuntimeError(f"Unexpected parameter type: {name}")
        if (parameters["open_loop"] or parameters["position_feedback"] or
                not 0.15 <= parameters["cmd_vel_timeout"] <= 0.3 or
                parameters["left_wheel_names"] != ["kabot_motion_model/left_wheel_joint"] or
                parameters["right_wheel_names"] != ["kabot_motion_model/right_wheel_joint"]):
            raise RuntimeError("Wrong model odometry wiring or command timeout")
        for name in ("left_wheel_radius_multiplier", "right_wheel_radius_multiplier", "wheel_separation_multiplier"):
            if parameters[name] != 1.0:
                raise RuntimeError("Expected unmodified physical wheel geometry")
        model_names = ["wheel_radius", "wheel_separation", "calibration.low_speed", "calibration.high_speed",
                       "calibration.low_effort", "calibration.high_effort", "calibration.ccw_rate", "calibration.cw_rate"]
        response = self.call(GetParameters, "/kabot_motion_model/get_parameters", GetParameters.Request(names=model_names))
        model = {}
        for name, value in zip(model_names, response.values):
            if value.type == 3 and math.isfinite(value.double_value):
                model[name] = value.double_value
            elif value.type == 8 and all(math.isfinite(item) for item in value.double_array_value):
                model[name] = list(value.double_array_value)
            else:
                raise RuntimeError(f"Invalid model parameter: {name}")
        for name in ("wheel_radius", "wheel_separation"):
            if parameters[name] != model[name]:
                raise RuntimeError(f"Model/diff-drive geometry mismatch: {name}")
        return {"drive": parameters, "model": model}

    def check_health(self):
        now = time.monotonic()
        if now - self.heartbeat_at > 0.6 or now - self.state_at > 0.6:
            raise RuntimeError("Lost advancing firmware heartbeat or controller state")
        if set(self.outputs) != set(WHEELS) or not all(math.isfinite(value) for value in self.outputs.values()):
            raise RuntimeError("Invalid controller effort state")
        unstamped_topic = self.node.resolve_topic_name("/cmd_vel_unstamped")
        stamped_topic = self.node.resolve_topic_name("/cmd_vel")
        if self.node.count_publishers(unstamped_topic) != (1 if self.publisher else 0):
            raise RuntimeError("Another command publisher is active; close teleop first")
        stamped = self.node.get_publishers_info_by_topic(stamped_topic)
        if (len(stamped) != 1 or stamped[0].node_name != "twist_stamper" or
                stamped[0].topic_type != "geometry_msgs/msg/TwistStamped" or
            self.node.count_subscribers(unstamped_topic) != 1 or
            self.node.count_subscribers(stamped_topic) < 1):
            raise RuntimeError("Expected one stamper and a connected controller, without direct command publishers")

    def publish(self, command):
        message = Twist()
        message.linear.x, message.angular.z = command
        self.publisher.publish(message)

    def pump(self, duration, command=None, monitor=False):
        deadline = time.monotonic() + duration
        next_publish = time.monotonic()
        while time.monotonic() < deadline:
            if monitor:
                self.check_health()
            now = time.monotonic()
            if command is not None and now >= next_publish:
                self.publish(command)
                next_publish = now + 0.05
            rclpy.spin_once(self.node, timeout_sec=max(0.0, min(0.02, deadline - time.monotonic())))


def main():
    arguments = parse_arguments()
    command = command_for(arguments.motion, arguments.speed, arguments.angular)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = arguments.output or Path("log/calibration") / f"{stamp}-{arguments.motion}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {"motion": arguments.motion, "host": arguments.host, "speed": arguments.speed,
              "duration_s": arguments.duration, "linear_m_s": command[0], "angular_rad_s": command[1],
              "execute": arguments.execute, "completed": False,
              "measured_distance_m": None, "measured_turns": None}

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    with output.open("x") as stream:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGINT, interrupt)
        signal.signal(signal.SIGTERM, interrupt)
        node = rclpy.create_node("kabot_calibration")
        control = None
        try:
            control = Calibration(node, arguments.host)
            robot = control.check_unclaimed()
            report["robot_serial"] = robot.serial
            report["firmware_version"] = robot.firmware_version
            report["parameters"] = control.parameters()
            control.pump(1.0)
            control.check_health()
            if any(abs(value) > 1e-6 for value in control.outputs.values()):
                raise RuntimeError("Controller output is not zero; stop other command sources first")
            report["predicted"] = predict_effort(report["parameters"], *command)
            print(json.dumps(report["predicted"], indent=2), flush=True)
            if any(value["saturated"] for value in report["predicted"].values()):
                print("SATURATION: increasing speed further cannot increase effort beyond 1.0.", flush=True)
            if arguments.execute:
                control.publisher = node.create_publisher(Twist, "/cmd_vel_unstamped", 1)
                control.pump(0.5)
                print(f"Clear the path: {arguments.motion} for {arguments.duration}s starts in 3s. Ctrl+C stops.", flush=True)
                run_trial(control, arguments.duration, command, report)
            else:
                print("Preview only: no claim or commands sent. Add --execute to run.", flush=True)
        except (Exception, KeyboardInterrupt) as error:
            report["error"] = str(error) or "Interrupted"
            print(f"STOP: {report['error']}", file=sys.stderr, flush=True)
        finally:
            if control:
                report["effort_samples"] = control.samples
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
            node.destroy_node()
            rclpy.shutdown()
            print(f"Report: {output}", flush=True)
    return 1 if "error" in report else 0


if __name__ == "__main__":
    sys.exit(main())