import importlib.util
import math
from pathlib import Path
import signal
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location("drive_loop", Path(__file__).resolve().parents[2] / "scripts/drive-loop.py")
route = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(route)


class FakeControl:
    def __init__(self, failure=None):
        self.events = []
        self.failure = failure
        self.moves = 0

    def pump(self, duration, command=None, monitor=False):
        self.events.append(("pump", duration, command))
        if command and command != (0, 0):
            self.moves += 1
            if self.moves == self.failure:
                raise RuntimeError("Lost connection")

    def check_unclaimed(self):
        pass

    def claim(self, enabled):
        self.events.append(("claim", enabled))

    def publish(self, command):
        self.events.append(("publish", command))


class TestDriveLoop(unittest.TestCase):
    def test_geometry_and_units(self):
        plan = route.make_plan()
        self.assertEqual(len(plan), 6)
        expected = ((.2, 0, 0), (.2, 0, -math.pi / 2), (.2, -.2, -math.pi / 2),
                    (.2, -.2, -math.pi), (0, 0, math.pi / 2), (0, 0, 0))
        for pose, goal in zip(route.nominal_poses(plan), expected):
            for actual, value in zip(pose.values(), goal):
                self.assertAlmostEqual(actual, value)
        self.assertAlmostEqual(plan[4]["linear_m_s"] / abs(plan[4]["angular_rad_s"]), .2)
        self.assertAlmostEqual(plan[1]["duration_s"], 22.93 / 20)

    def test_sequence_and_abort(self):
        for failure in (None, 1, 3, 5, 6):
            with self.subTest(failure=failure), patch.object(signal, "signal"):
                control = FakeControl(failure)
                report = {"completed": False}
                if failure:
                    with self.assertRaises(RuntimeError):
                        route.calibration.run_segments(control, route.make_plan(), report)
                    self.assertFalse(report["completed"])
                    self.assertEqual(control.moves, failure)
                else:
                    route.calibration.run_segments(control, route.make_plan(), report)
                    self.assertTrue(report["completed"])
                    self.assertEqual(control.moves, 6)
                self.assertTrue(report["released"])
                self.assertEqual(control.events[-1], ("pump", .5, (0, 0)))
                self.assertIn(("claim", False), control.events)
                self.assertIn(("publish", (0, 0)), control.events)


if __name__ == "__main__":
    unittest.main()