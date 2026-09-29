# Kabot Robot Description

This package provides the Kabot model, RViz visualization, differential-drive
control with mock hardware and Gazebo physics simulation. It does not communicate
with physical motors.

## Description

- `description/urdf/kabot.urdf.xacro`: entry point with an optional `prefix`.
- `description/urdf/kabot_description.urdf.xacro`: shared geometry, with optional physics.
- `description/urdf/kabot.materials.xacro`: visual materials.
- `description/launch/view_robot.launch.py`: visualization launch file.
- `description/rviz/kabot_view.rviz`: local RViz configuration.
- `description/rviz/kabot_mock.rviz`: wider odometry-frame view for driving.
- `description/urdf/kabot_mock.urdf.xacro`: geometry plus mock control interfaces.
- `description/ros2_control/kabot_mock.ros2_control.xacro`: `GenericSystem` mock hardware.
- `bringup/launch/mock.launch.py`: controller manager, controllers and optional RViz.
- `bringup/config/kabot_mock_controllers.yaml`: controller parameters for Kabot.
- `description/urdf/kabot_gazebo.urdf.xacro`: geometry, physics and Gazebo control.
- `description/gazebo/`: physical solid macros, contact settings and model pose publisher.
- `description/ros2_control/kabot_gazebo.ros2_control.xacro`: Gazebo hardware interfaces.
- `bringup/launch/gazebo.launch.py`: simulator, bridges, robot spawn and controller activation.
- `bringup/config/kabot_gazebo_controllers.yaml`: simulation controller parameters.
- `bringup/config/kabot_gazebo_gui.config`: local Gazebo view at the robot's scale.
- `bringup/worlds/kabot.world.sdf`: floor, lighting and physics, without robot geometry.

The geometry was migrated from `../kabot.urdf`. Keep that file unchanged as a
migration reference; maintain the active description in Xacro. The model retains
all seven links, six joints, dimensions, colors and joint origins. Wheel radius
is 0.016 m, width is 0.002 m and wheel-center separation is 0.102 m. Left and right
wheel joint rolls remain -1.5 and +1.5 radians; the camber is intentional.

The only kinematic change is the right wheel's local axis: `0 0 -1` instead of
`0 0 1`. The left axis remains `0 0 1`. Positive wheel velocities now correspond
to forward rolling on both sides. Motor and encoder sign conversion belongs in
the future hardware interface. Collision shapes, masses and inertias are enabled
only by the Gazebo wrapper; the geometry-only and mock descriptions are unchanged.

`hardware/` and the old `diffbot*` files retain the copied demo sources for later
work. They are not built, installed or launched. Their package names and controller
dimensions are still those of DiffBot. Only the Kabot descriptions, launches and
configurations are installed. The `ros2_control_demos_link` checkout is not required
to build or run Kabot.

## Build and Run

On Linux, from the workspace directory containing `pixi.toml`:

```bash
pixi shell
colcon build --paths kabot_robot --packages-select kabot_robot --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
ros2 launch kabot_robot view_robot.launch.py
```

Activate Pixi before sourcing the overlay. Pixi activation loads an existing
`install/setup.sh` if available; a missing install directory is allowed before
the first build. The `mock` and `sim` tasks reload the overlay after building.
Use `--paths kabot_robot` to avoid discovering the linked demo repository.

On Windows, run these commands in PowerShell from the same workspace directory:

```powershell
pixi run build
pixi run view
```

The Pixi tasks load the ROS environment automatically.

The default launch opens RViz and the joint slider GUI. Alternative invocations:

```bash
ros2 launch kabot_robot view_robot.launch.py gui:=false
ros2 launch kabot_robot view_robot.launch.py prefix:=kabot_
```

On Windows, pass these arguments to `pixi run view`, for example
`pixi run view gui:=false` or `pixi run view prefix:=kabot_`.

Without the GUI, a regular `joint_state_publisher` supplies wheel positions. With
a prefix, all link/joint names and RViz's fixed frame use it. This is frame-name
prefixing, not a complete multi-robot namespace setup. RViz's optional TF display
can show wheel orientation; uniform cylinders do not visibly reveal their spin.

## Mock Control

From the workspace directory:

```bash
pixi run mock
```

This runs the existing `build` task first, loads the newly installed package and
starts `mock_components/GenericSystem`, `controller_manager`,
`joint_state_broadcaster`, `kabot_base_controller` (`diff_drive_controller`) and
RViz. The ordinary geometry viewer remains available as `pixi run view`.
Do not run `view`, `mock` and `sim` at the same time in the same ROS domain: they would
publish competing descriptions, joint states and TF.

```bash
pixi run mock gui:=false
pixi run mock prefix:=kabot_
```

The mock uses velocity commands (rad/s), integrates wheel positions (rad) and
returns them as feedback. Odometry is computed from that feedback, not open-loop
commands. `/joint_states` comes only from the broadcaster; there is no slider GUI.
RViz uses `odom` as its fixed frame, so the robot moves relative to the grid.
With a prefix, joint names and both `odom`/`base_link` frames are prefixed;
controller names and topic names stay unchanged.

In another terminal, while mock is running:

```bash
pixi run ros2 control list_controllers
pixi run ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped '{header: auto, twist: {linear: {x: 0.05}, angular: {z: 0.0}}}'
```

Use `linear.x: 0.0` and `angular.z: 0.5` to turn in place. Stop publishing with
Ctrl+C; commands expire after 0.5 s and the controller stops the wheels. The
controller limits speed to 0.1 m/s and yaw rate to 1 rad/s. These are conservative
mock settings, not validated physical motor limits. Always use fresh timestamps;
the input is `TwistStamped`, not `Twist`.

Observe `/kabot_base_controller/odom` (`nav_msgs/msg/Odometry`) and TF
`odom -> base_link -> wheels`. Both wheel velocities should be +3.125 rad/s for
0.05 m/s straight motion. Mock hardware does not simulate contact, slip or camber
physics. The nominal 0.102 m separation and 0.016 m radius may need calibration
on the physical robot. A warning about missing FIFO scheduling privileges is
expected on an ordinary desktop and does not prevent this mock from running.

## Gazebo Simulation

```bash
pixi run sim
pixi run sim gui:=false
pixi run sim rviz:=true prefix:=kabot_
```

These are alternative invocations, not three simultaneous sessions. The task
builds the package, reloads the overlay and starts a running Gazebo world. Gazebo
GUI is enabled by default; RViz is independently optional and disabled by default.
The local camera is framed for this small robot. Server and GUI run as separate
supervised processes. Ctrl+C stops the launched processes.

On the tested Pixi Gazebo 10.5.0 installation, the GUI renders correctly but does
not finish shutting down on SIGINT. Launch terminates that GUI with SIGTERM after
5 seconds and logs an error; the server, controllers, bridge and RViz exit cleanly.
This GUI shutdown limitation remains unresolved. `gui:=false` avoids it and has
fully passing motion and clean-shutdown tests.

Gazebo creates the robot from the shared Xacro description. Its
`gz_ros2_control/GazeboSimSystem` plugin owns the only controller manager and
exposes wheel velocity commands and position/velocity feedback. Controllers
activate after successful robot creation. No mock hardware, joint slider publisher
or second differential-drive system runs in this mode. Startup failures stop the
launch; robot creation has a 45-second wall-clock timeout.

In a second terminal, send commands using **simulation time**:

```bash
pixi run ros2 topic pub --use-sim-time --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped '{header: auto, twist: {linear: {x: 0.03}, angular: {z: 0.0}}}'
```

Use `linear.x: 0.0` and `angular.z: 0.4` to turn. Stop publishing with Ctrl+C to
exercise the 0.5-second command timeout. Both timestamps and timeout use `/clock`:
pausing Gazebo also pauses elapsed simulation time. Do not send wall-clock stamped
commands. The same conservative 0.1 m/s and 1 rad/s limits apply as in mock.

`/joint_states`, `/kabot_base_controller/odom` and TF retain their mock interfaces.
Odometry is computed from wheel feedback, so it can differ from actual movement
under slip or camber. `/simulation/ground_truth` (`tf2_msgs/msg/TFMessage`) reports
the Gazebo model pose for validation only; it is never bridged into `/tf`.
Joint and odometry frame names respect `prefix`, while the Gazebo model is always
named `kabot`. This is not a multi-robot launch. Independent concurrent simulations
require different `ROS_DOMAIN_ID` and `GZ_PARTITION` values, not just frame prefixes.

### Physical Model

| Part | Mass per part | Count |
| --- | ---: | ---: |
| Chassis | 150 g | 1 |
| Top disk | 20 g | 1 |
| Wheel | 10 g | 2 |
| Slider | 5 g | 2 |
| **Total** | **200 g** | |

Collision solids exactly match the visual solids. Inertias assume homogeneous
boxes, cylinders and spheres with their centers of mass at the geometric centers.
These are approximations, not measured mass distributions. Gazebo merges fixed
links while preserving the total mass; `base_link` receives no invented extra mass.

Initial isotropic friction coefficients are 1.0 for wheels, 0.05 for sliders and
0.5 for chassis/top; these are tuning assumptions, not material measurements.
They are parameters of the `kabot_gazebo` Xacro macro. The local world uses DART,
a 1 ms physics step, gravity 9.81 m/s^2 and a target real-time factor of 1.
The controller updates at 100 Hz; Gazebo may warn that this is slower than its
1 kHz physics step. This is intentional.

The unchanged geometry leaves approximately 2.33 mm clearance below the sliders
when the body is level. With this mass distribution the robot settles against
the rear slider with about -0.047 rad pitch; it is not forced perfectly level.
Wheel drives are ideal velocity actuators, not an electrical motor, gearbox,
torque-limit or battery model. Sensors, SLAM and navigation are outside this stage.

## Tests

On Linux, in the same Pixi shell after building and sourcing the overlay:

```bash
colcon test --paths kabot_robot --packages-select kabot_robot --event-handlers console_direct+
colcon test-result --test-result-base build/kabot_robot --verbose
```

On Windows, use PowerShell:

```powershell
pixi run test
```

The Xacro test expands the installed model, runs `check_urdf` and compares its XML
semantically with the original `../kabot.urdf`, allowing only the right-axis sign
change. It checks all three wrappers with empty and nonempty prefixes, including
control interfaces, physical solids and mass/joint/friction preservation through
`gz sdf -p` conversion. Only this migration test needs
the original file; the installed model and launch do not depend on it.

Isolated headless launch tests check `/robot_description`, both wheel states,
all six child transforms (including camber), a single joint-state publisher and
clean shutdown. Both prefix variants are tested without a display server.
Additional isolated mock tests check controller activation, a single joint-state
publisher, forward/reverse wheel speeds, position feedback, yaw motion, odometry,
the full TF tree, command timeout and stationary positions after stopping.
Gazebo tests also isolate `GZ_PARTITION`, use simulation time and check settling,
actual forward/reverse displacement and yaw from the simulator, wheel feedback,
timeout and clean shutdown. They allow physical tolerances instead of requiring
exact agreement with encoder odometry.
For source-only geometry checks before building:

```bash
pixi run python -m pytest kabot_robot/test/test_urdf_xacro.py -q
```
