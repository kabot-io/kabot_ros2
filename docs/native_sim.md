# Zephyr Native Simulator

The default native launch now uses [calibrated physical motion and model odometry](../../docs/calibrated-motion.md).
The simulation and raw numerical examples below explicitly select
`calibrated:=false`, which preserves FF-only PID mapping. Use the linked guide
for `rviz:=true` and `pixi run drive-loop`; these need the calibrated profile.

For the physical ESP32-S3 with router and controller on the host, follow the
[hardware stack runbook](../../docs/ros2-zenbedded-hardware-stack.md). It includes
the remote RMW/plugin endpoints, physical robot claim/release and bounded
`cmd_vel` commands; `native-claim` defaults to localhost, with `--host` for a robot.

The data path follows the Zenbedded sine-wave demo: firmware Tier 2 payloads,
Zenoh router, `zenbedded_hardware_interface/ZenbeddedHardware`, ros2_control,
then a standard ROS broadcaster, `pid_controller/PidController` and
`diff_drive_controller/DiffDriveController`.
ROS Lyrical's `state_interfaces_broadcaster/StateInterfacesBroadcaster`
publishes arbitrary state interfaces using separate names and values topics.

## Start

In `kabot_ros2`, build the hardware plugin from the adjacent Zenbedded checkout:

```bash
pixi run build-native
```

Pixi activation sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` for launches and CLI
commands, including the existing mock and Gazebo tasks. Outside Pixi, activate
your ROS environment and export this variable explicitly. After switching RMW,
stop an old ROS CLI daemon with `ros2 daemon stop` if graph queries show stale nodes.

Start the router yourself in a separate terminal, before firmware or ROS:

```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
```

The launch does not start or stop the router. Both the firmware and hardware
plugin connect as clients to `tcp/127.0.0.1:7447` in the same network namespace.
The vendor configuration may warn about deprecated routing options; these do
not prevent the router from starting.

From the parent `kabot-zephyr` directory, build and run firmware:

```bash
source .venv/bin/activate
west build app -b native_sim -d build/native_sim_zenbedded
build/native_sim_zenbedded/zephyr/zephyr.exe
```

In another terminal, from `kabot_ros2`:

```bash
pixi run native -- calibrated:=false
```

Do not run `native`, `mock`, `sim` or `view` together in the same ROS domain.

## Read

```bash
pixi run ros2 control list_controllers
pixi run ros2 topic list -t
pixi run ros2 topic echo /kabot_state_broadcaster/names control_msgs/msg/Keys --once --qos-durability transient_local
pixi run ros2 topic echo /kabot_state_broadcaster/values control_msgs/msg/Float64Values
pixi run test-native
```

`names.keys` contains `kabot/heartbeat`, `left_wheel_joint/position`,
`left_wheel_joint/velocity`, `right_wheel_joint/position` and
`right_wheel_joint/velocity`; match each value to the name at the same index.
Heartbeat is firmware uptime in seconds. Wheel positions are PCNT encoder angles
converted from Q31 degrees to radians, wrapped to one revolution (`[0, 2*pi)`).
These are not accumulated multi-turn positions. Firmware reads the latest paired
sample from `sensor_channel` without blocking and retains the previous positions
when no valid pair can be read. Positions start at zero; native_sim leaves them
at zero unless encoder samples are supplied.

Wheel velocities are measured in rad/s using the shortest angular difference
between new encoder samples and their sensor timestamps, not the ROS loop time
or the velocity commands. The first sample initializes velocity to zero; repeated
timestamps retain the previous estimate, and a backward timestamp resets it to
zero. A new stationary sample yields zero velocity. Missing samples retain the
last state, so these fields do not themselves indicate sensor freshness.

Because PCNT supplies wrapped angles, this estimate assumes less than half a
revolution between consumed samples (about 5 revolutions/s at a 100 ms interval).
Faster motion or missed samples can alias the measured speed and direction.

Values are published at 10 Hz. The check requires all five interfaces, 12 finite
samples, positions in the wrapped range and a positive, advancing heartbeat.

Run the velocity estimator unit test from the firmware repository:

```bash
c++ -std=c++17 -Wall -Wextra -Werror -I app/include tests/wheel_encoder_state_test.cpp -o /tmp/kabot-wheel-encoder-state-test
/tmp/kabot-wheel-encoder-state-test
```

## Keyboard Control

The keyboard task now defaults to physical units for `calibrated:=true`.
Restart the single native stack in that mode before using this task; the raw
simulation examples in this page use a different command scale.

The native launch starts `twist_stamper`. In a second interactive terminal from
`kabot_ros2`, use the same `ZENOH_CONFIG_OVERRIDE` as the controller terminal:

```bash
pixi run native-claim
pixi run teleop
```

For the physical robot, replace the first command with
`pixi run native-claim --host 192.168.0.101`. Claim only when the robot can move
safely. Use `i`/`,` for forward/backward, `j`/`l` for turning, `k` to stop and
Ctrl+C to exit. Afterwards run `pixi run native-claim --release`, also supplying
`--host 192.168.0.101` for the physical robot.

The task sends unstamped `Twist` to `/cmd_vel_unstamped`; stamper converts each
message to a fresh `TwistStamped` on `/cmd_vel` with `frame_id=base_link`.
Initial speed/turn are 0.03372434 m/s and 0.5 rad/s; use `w`/`x` and `e`/`c` to
adjust them; effort saturates at +/-1 with no low diff-drive speed cap. This keyboard version
publishes only on key events, including OS auto-repeat. Gaps longer than 300 ms
trigger the controller timeout; stamper never periodically refreshes old input.
Use one command source at a time, not teleop and a direct publisher together.

## Historical Raw Differential Drive

Current mode is open-loop because one physical encoder is suspect. The
[PID knowledge dump](../../docs/host-pid-bringup.md) preserves the closed-loop
implementation, trial results and prerequisites for enabling it again.

`kabot_base_controller` is the standard `diff_drive_controller/DiffDriveController`.
It accepts `geometry_msgs/msg/TwistStamped` on `/cmd_vel` and writes
`kabot_wheel_pid/left_wheel_joint/velocity` and
`kabot_wheel_pid/right_wheel_joint/velocity`. These are references exported by
`kabot_wheel_pid`, a standard `pid_controller/PidController` with two independent
channels. PID reads the hardware wheel `velocity` states and owns the hardware
`left_wheel_joint/effort` and `right_wheel_joint/effort` command interfaces.

Wheel separation is 0.102 m and radius is 0.016 m, matching the existing Kabot
mock configuration. The controller computes left/right wheel angular velocity
from `linear.x` (m/s) and `angular.z` (rad/s). Each wheel now has P=I=D=0,
feedforward=1 and `set_current_state_as_first_setpoint: false`. The standard PID
node is retained, but performs no feedback correction: references map numerically
1:1 to normalized effort, not torque in Nm. Hardware effort limits remain +/-1.
The old 0.01 m/s and 0.1 rad/s input limits have been removed: forward 0.016 m/s
reaches effort 1 and turning about 0.3137255 rad/s reaches opposite efforts +/-1.
`linear.x=0.005` requests 0.3125 rad/s and yields 0.3125 effort irrespective of
finite measured speed. This is twice the former P=0.5 output at rest, not a
calibrated physical speed. Larger commands beyond saturation do not increase
power. Restart the native launch to load the new configuration.

For timed physical forward/left/right trials use `pixi run calibrate`; the
[calibration guide](../../docs/open-loop-calibration.md) covers preview,
execution, stop/release, JSON reports and manual distance/rotation measurements.

Explicitly claim the local simulator before driving (or use the existing HMI):

```bash
pixi run native-claim
```

Drive forward with fresh timestamps:

```bash
pixi run ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped '{header: auto, twist: {linear: {x: 0.005}, angular: {z: 0.0}}}'
```

Alternatively, turn in place:

```bash
pixi run ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped '{header: auto, twist: {linear: {x: 0.0}, angular: {z: 0.05}}}'
```

Use one publisher at a time. Ctrl+C stops refreshing the command; diff-drive
zeros both velocity references after 300 ms, plus 10 Hz scheduling/transport
latency. With feedback gains zero, this also zeros effort even if reported
encoder velocity remains nonzero; there is no PID braking correction.
Publish zero linear and angular components to command a stop.
To disable both motors independently of the publishers:

```bash
pixi run native-claim --release
```

The helper uses the parent repository's existing Bonjour protobuf codec and
defaults to localhost; use `--host` for a remote robot. Do not send motor commands through the UDP HMI and ROS
simultaneously: these are not arbitrated input sources. Claim is the existing
application enable gate, not authentication.

Diff-drive uses `open_loop: true`; `position_feedback: false` is retained but
does not select an odometry source in this mode. `/kabot_base_controller/odom`
and `odom -> base_link` TF are integrated from commands, not encoder data. They
can show motion while the robot is unclaimed, disconnected or stalled. Native
firmware need not synthesize encoder samples for open-loop odometry to move.

Encoder states remain diagnostic telemetry. There is no new sensor freshness
gate or hardware fix. Do not re-enable feedback-based odometry or PID correction
until both encoder channels, their signs and scaling are verified; the knowledge
dump records the additional stale-data and claim/integrator limitations.

Diff-drive owns the `/cmd_vel` timeout. Firmware forwards only newly received
Zenbedded packets to the existing zbus validator, claim gate and motor watchdog.
The 300 ms firmware watchdog stops motors if the ROS hardware process or transport
stops delivering packets. Do not replace diff-drive with a controller that holds
nonzero outputs indefinitely: repeated cached transport packets count as fresh.

After building firmware, stop manual firmware/ROS instances and run the regression
with your router still running:

```bash
pixi run test-native-motors
```

It starts only local native firmware and ROS processes, checks the simulated motor
driver's logs for forward/reverse mapping, turning, effort saturation, claim/release,
stale timestamps, publisher timeout and ROS process loss using the current
feedforward mapping. It checks command-derived odometry and stops its processes
afterwards. It does not manage the router.

Two isolated launch tests use `mock_components/GenericSystem`, nonzero wheel
velocities and fixed positions. `kabot_native_open_loop` verifies reference-only
effort, full-scale saturation, timed calibration, command-derived odometry and zero output/odometry after
timeout despite nonzero feedback. `kabot_native_pid` restores the historical
closed-loop parameters only inside its test and checks feedback regulation:

```bash
pixi run build-native
pixi run colcon test --packages-select kabot_robot --ctest-args -R '^kabot_native_(pid|open_loop)$' --output-on-failure
pixi run colcon test-result --test-result-base build/kabot_robot/test_results --verbose
```

Use the RMW endpoint export for your existing router, as for other ROS commands.
The test does not connect to the robot or publish on its `/cmd_vel`.

## Schema And Scope

The default schema is the firmware's `../app/config/zenbedded_native_sim.yaml`.
The launch reads that same file, without a separate ROS copy. Rebuild firmware
and restart ROS after changing the schema; update the URDF interfaces and
broadcaster selection to match. Tier 2 uses packed native numeric values;
ROS exposes the decoded float32 states as float64. The state payload is 20 bytes:
heartbeat, left wheel position, left wheel velocity, right wheel position, then
right wheel velocity (all float32).

For a standalone checkout, set `ZENBEDDED_ROOT` before `build-native` and
`KABOT_NATIVE_SIM_SCHEMA` before `native`. The launch also accepts explicit
`schema_path:=/absolute/path/schema.yaml` and `zenoh_endpoint:=tcp/HOST:7447`.
For a remote router, change the firmware endpoint and configure the ROS RMW
session endpoint separately as well; the launch argument only affects the
hardware plugin's raw Tier 2 connection.

The command payload is 8 bytes: float32 `effort` for the left joint followed
by float32 `effort` for the right joint. Hardware command interfaces are no
longer named `velocity`; velocity is feedback and the internal PID reference.
Rebuild/flash firmware and restart ROS together for this contract change.
This interface also runs on ESP32 with measured wheel positions and estimated
velocities. Host-side speed regulation is preserved but disabled while the
encoder problem is investigated. Switching this deployed effort firmware between
control modes requires a ROS restart, not another flash. It is not the full
sensor protobuf stream.
The state plugin initially
reports zero and retains its last state after disconnect, so topic publication
alone is not a freshness guarantee. Start the router before firmware; automatic
recovery after router loss has not been validated.