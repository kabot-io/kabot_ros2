import argparse
from datetime import datetime, timezone
import importlib.util
import json
import math
from pathlib import Path
import signal
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions


SPEC = importlib.util.spec_from_file_location("calibrate_motion", Path(__file__).with_name("calibrate-motion.py"))
calibration = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration)


def make_plan():
    speed = 0.575 / 17.05
    turn_rate = 10 * math.pi / 22.93
    side = 0.2
    straight = {"name": "forward 20 cm", "linear_m_s": speed, "angular_rad_s": 0.0, "duration_s": side / speed}
    turn = {"name": "right 90 degrees", "linear_m_s": 0.0, "angular_rad_s": -turn_rate,
            "duration_s": math.pi / (2 * turn_rate)}
    arc = {"name": "right quarter-circle, diameter 40 cm", "linear_m_s": speed,
           "angular_rad_s": -speed / side, "duration_s": math.pi * side / (2 * speed)}
    return [dict(straight), dict(turn), dict(straight), dict(turn), arc, dict(turn)]


def nominal_poses(segments):
    horizontal, vertical, heading = 0.0, 0.0, 0.0
    poses = []
    for segment in segments:
        linear, angular, duration = segment["linear_m_s"], segment["angular_rad_s"], segment["duration_s"]
        end_heading = heading + angular * duration
        if angular == 0:
            horizontal += linear * duration * math.cos(heading)
            vertical += linear * duration * math.sin(heading)
        else:
            horizontal += linear / angular * (math.sin(end_heading) - math.sin(heading))
            vertical -= linear / angular * (math.cos(end_heading) - math.cos(heading))
        heading = end_heading
        poses.append({"x_m": horizontal, "y_m": vertical, "yaw_rad": math.remainder(heading, math.tau)})
    return poses


def main():
    parser = argparse.ArgumentParser(description="20 cm forward, right 90, 20 cm forward, right 90, right arc back, right 90.")
    parser.add_argument("--host", required=True, help="Unclaimed robot IP/hostname")
    parser.add_argument("--execute", action="store_true", help="Run on the calibrated profile after a 3 s countdown")
    parser.add_argument("--output", type=Path, help="New JSON report file")
    arguments = parser.parse_args()
    plan = make_plan()
    report = {"host": arguments.host, "execute": arguments.execute, "completed": False,
              "plan": plan, "nominal_poses": nominal_poses(plan), "model_only": True}
    print(json.dumps(report, indent=2), flush=True)
    if not arguments.execute:
        print("Plan only; no network, claim or commands. Add --execute to run on native calibrated:=true.")
        return 0
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = arguments.output or Path("log/calibration") / f"{timestamp}-drive-loop.json"
    output.parent.mkdir(parents=True, exist_ok=True)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    with output.open("x") as stream:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGINT, interrupt)
        signal.signal(signal.SIGTERM, interrupt)
        node = rclpy.create_node("kabot_drive_loop")
        control = None
        try:
            control = calibration.Calibration(node, arguments.host, calibrated=True)
            robot = control.check_unclaimed()
            report["robot_serial"] = robot.serial
            report["firmware_version"] = robot.firmware_version
            report["parameters"] = control.parameters()
            control.pump(1.0)
            control.check_health()
            if any(abs(value) > 1e-6 for value in control.outputs.values()):
                raise RuntimeError("Nonzero controller output before start")
            control.publisher = node.create_publisher(Twist, "/cmd_vel_unstamped", 1)
            control.pump(0.5)
            print("Clear the path. Starts in 3 seconds; Ctrl+C stops. Model return is not physical localization.", flush=True)
            calibration.run_segments(control, plan, report)
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