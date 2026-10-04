import math
import time

import rclpy
from control_msgs.msg import Float64Values, Keys
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    rclpy.init()
    node = rclpy.create_node("kabot_native_sim_check")
    names = []
    samples = []
    expected_names = {
        "kabot/heartbeat",
        "left_wheel_joint/position", "left_wheel_joint/velocity",
        "right_wheel_joint/position", "right_wheel_joint/velocity",
    }

    def receive_names(message):
        names[:] = message.keys

    def receive_values(message):
        if len(names) == len(message.values) == len(expected_names):
            samples.append(dict(zip(names, message.values)))

    node.create_subscription(
        Keys, "/kabot_state_broadcaster/names", receive_names,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        Float64Values, "/kabot_state_broadcaster/values", receive_values, 10,
    )
    deadline = time.monotonic() + 10
    try:
        while time.monotonic() < deadline and (not names or len(samples) < 12):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(names) == len(expected_names) and set(names) == expected_names, names
        assert len(samples) >= 12, f"Received only {len(samples)} samples"
        assert all(math.isfinite(value) for sample in samples for value in sample.values()), samples
        heartbeat = [sample["kabot/heartbeat"] for sample in samples]
        assert all(value > 0 for value in heartbeat), heartbeat
        assert all(after >= before for before, after in zip(heartbeat, heartbeat[1:])), heartbeat
        assert heartbeat[-1] - heartbeat[0] >= 0.5, f"Heartbeat is not advancing: {heartbeat}"
        assert all(
            0 <= sample[name] <= math.tau + 1e-6
            for sample in samples
            for name in ("left_wheel_joint/position", "right_wheel_joint/position")
        ), samples
        print(f"PASS: {len(samples)} samples; heartbeat {heartbeat[0]:.2f} -> {heartbeat[-1]:.2f} s; "
              f"position L={samples[-1]['left_wheel_joint/position']:.6f} "
              f"R={samples[-1]['right_wheel_joint/position']:.6f} rad; "
              f"velocity L={samples[-1]['left_wheel_joint/velocity']:.6f} "
              f"R={samples[-1]['right_wheel_joint/velocity']:.6f} rad/s")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()