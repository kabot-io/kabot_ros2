# ROS2 packages for Kabot project

In this repository you'll find robot bringup files `ros_control` stuff and urdf. Use [pixi](https://pixi.prefix.dev/latest/) for development.

![alt text](docs/image.png)

## Quickstart:

Install [pixi](https://pixi.prefix.dev/latest/):
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

Run simulation
```
pixi run sim
```

View URDF in Rviz:
```
pixi run view
```

