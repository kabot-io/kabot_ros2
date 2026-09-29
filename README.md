# ROS2 packages for Kabot project

In this repository you'll find robot bringup files `ros_control` stuff and urdf. Use [pixi](https://pixi.prefix.dev/latest/) for development.

![alt text](docs/image.png)

## Platform status

Pixi is configured for Linux x86-64 and Windows x86-64 (`win-64`). Native Windows
support is experimental: installation, building and all six URDF geometry tests
have been verified. With the current lockfile, `mock` and `sim` crash with an
access violation (`0xC0000005`) in the Windows ROS control stack. The standalone
`ros2_control_node` also crashes without a Kabot model. Use Linux or WSL2 for
simulation until that dependency issue is resolved.

## Quickstart

On Windows, install Pixi in PowerShell:

```powershell
irm https://pixi.sh/install.ps1 | iex
```

Reopen PowerShell after installation so that `pixi` is on `PATH`. Run each command
below on a separate line; do not append `git clone` to `iex`.

On Linux, install [Pixi](https://pixi.prefix.dev/latest/):
```bash
curl -fsSL https://pixi.sh/install.sh | sh
```
Clone the repo and cd into it:
```
git clone https://github.com/kabot-io/kabot_ros2.git
cd kabot_ros2
```

Build workspace:
```
pixi run build
```

Run simulation (see the Windows limitation above):
```
pixi run sim
```

View URDF in Rviz:
```
pixi run view
```

Windows tasks use native batch scripts and the CMake/Ninja tools installed by
Pixi. The build copies files instead of creating symlinks, so Windows Developer
Mode is not required. `sim`, `mock` and `view` reload the ROS environment after
building, including on the first invocation. Bash is not needed for these tasks.

On Windows, the launch files disable Qt HiDPI scaling for RViz by default to
avoid a flickering or dark viewport at fractional display scales (confirmed at
125%). This applies to RViz in `view`, `mock` and `sim rviz:=true`; it does not
change Windows display settings or scaling in the other GUI processes. RViz's
interface may appear smaller. To opt back into Qt scaling, set
`$env:QT_ENABLE_HIGHDPI_SCALING = '1'` in PowerShell before launching. See the
[RViz HiDPI issue](https://github.com/ros2/rviz/issues/1052).

## Simulation on Windows using WSL2

If needed, install Ubuntu with `wsl --install -d Ubuntu` in an administrator
PowerShell, restart if prompted, and finish Ubuntu's first-run setup. Gazebo's
GUI requires WSLg; see the
[Microsoft prerequisites](https://learn.microsoft.com/windows/wsl/tutorials/gui-apps).

In the **Ubuntu terminal**, install Linux Pixi and use a separate Linux checkout:

```bash
curl -fsSL https://pixi.sh/install.sh | sh
export PATH="$HOME/.pixi/bin:$PATH"
git clone https://github.com/kabot-io/kabot_ros2.git ~/kabot_ros2
cd ~/kabot_ros2
pixi run sim
```

Keep the Linux checkout in the Linux home directory; do not reuse the Windows
`.pixi`, `build` or `install` directories. For a headless simulation, use
`pixi run sim gui:=false`.

