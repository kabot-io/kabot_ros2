import importlib.util
import io
from itertools import product
from pathlib import Path
import signal
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    "calibrate_motion", Path(__file__).resolve().parents[2] / "scripts/calibrate-motion.py")
calibration = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration)


class FakeControl:
    def __init__(self, failure=None):
        self.events = []
        self.failure = failure

    def pump(self, duration, command=None, monitor=False):
        self.events.append(("pump", duration, command, monitor))
        if command is not None and command != (0.0, 0.0) and self.failure == "motion":
            raise RuntimeError("Lost heartbeat")
        if command is not None and command != (0.0, 0.0) and self.failure == "interrupt":
            raise KeyboardInterrupt

    def check_unclaimed(self):
        self.events.append(("unclaimed",))
        if self.failure == "claimed":
            raise RuntimeError("Already claimed")

    def claim(self, enabled):
        self.events.append(("claim", enabled))
        if enabled and self.failure == "claim":
            raise TimeoutError("Lost claim reply")
        if not enabled and self.failure == "release":
            raise TimeoutError("Lost release reply")

    def publish(self, command):
        self.events.append(("publish", command))


class TestCalibration(unittest.TestCase):
    def test_arguments_and_directions(self):
        for motion, expected in (("forward", (0.2, 0.0)), ("left", (0.0, 0.2)), ("right", (0.0, -0.2))):
            arguments = calibration.parse_arguments([motion, "--host", "localhost", "--speed", "0.2", "--duration", "5"])
            self.assertFalse(arguments.execute)
            self.assertEqual(arguments.angular, 0.0)
            self.assertEqual(calibration.command_for(motion, arguments.speed), expected)
        for value in ("0", "-1", "nan", "inf", "-inf"):
            with self.assertRaises(calibration.argparse.ArgumentTypeError):
                calibration.positive(value)

    def test_forward_angular_trim(self):
        common = ["--host", "localhost", "--speed", "0.016", "--duration", "20"]
        for angular in ("-0.01", "0", "0.01"):
            arguments = calibration.parse_arguments(["forward", *common, "--angular", angular])
            self.assertEqual(calibration.command_for(arguments.motion, arguments.speed, arguments.angular),
                             (0.016, float(angular)))
        for motion, angular in (("left", "-0.01"), ("right", "0"), ("forward", "nan"),
                                ("forward", "inf"), ("forward", "-inf")):
            with self.subTest(motion=motion, angular=angular), patch("sys.stderr", new_callable=io.StringIO):
                with self.assertRaises(SystemExit) as error:
                    calibration.parse_arguments([motion, *common, f"--angular={angular}"])
                self.assertEqual(error.exception.code, 2)

    def test_full_effort_and_saturation(self):
        parameters = {"wheel_radius": 0.016, "wheel_separation": 0.102,
                      "gains.left_wheel_joint.feedforward_gain": 1.0,
                      "gains.right_wheel_joint.feedforward_gain": 1.0}
        for linear, angular, expected in ((0.0, 0.0, (0.0, 0.0)), (0.016, 0.0, (1.0, 1.0)),
                                          (1.0, 0.0, (1.0, 1.0)), (0.0, 1.0, (-1.0, 1.0)),
                                          (0.0, -1.0, (1.0, -1.0)),
                                          (0.016, -0.01, (1.0, 0.968125)),
                                          (0.016, 0.01, (0.968125, 1.0))):
            prediction = calibration.predict_effort(parameters, linear, angular)
            for wheel, effort in zip(calibration.WHEELS, expected):
                self.assertAlmostEqual(prediction[wheel]["effort"], effort)
        self.assertTrue(calibration.predict_effort(parameters, 1.0, 0.0)["left_wheel_joint"]["saturated"])
        with self.assertRaises(ValueError):
            calibration.predict_effort(parameters, 1e308, 0.0)

    def test_trial_cleanup_on_success_and_failures(self):
        for failure, command in product((None, "motion", "interrupt", "claim", "release", "claimed"),
                                        ((0.016, 0.0), (0.016, -0.01))):
            with self.subTest(failure=failure, command=command), patch.object(signal, "signal"):
                control = FakeControl(failure)
                report = {"completed": False}
                if failure:
                    with self.assertRaises((RuntimeError, TimeoutError, KeyboardInterrupt)):
                        calibration.run_trial(control, 5.0, command, report)
                else:
                    calibration.run_trial(control, 5.0, command, report)
                    self.assertTrue(report["completed"])
                    self.assertIn(("pump", 5.0, command, True), control.events)
                if failure == "claimed":
                    self.assertNotIn(("claim", True), control.events)
                    self.assertNotIn(("claim", False), control.events)
                else:
                    self.assertIn(("publish", (0.0, 0.0)), control.events)
                    self.assertIn(("claim", False), control.events)
                    self.assertEqual(control.events[-1], ("pump", 0.5, (0.0, 0.0), False))
                    self.assertEqual(report["released"], failure != "release")
                    if failure == "release":
                        self.assertEqual(control.events.count(("claim", False)), 3)


if __name__ == "__main__":
    unittest.main()