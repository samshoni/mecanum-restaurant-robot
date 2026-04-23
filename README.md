# Mecanum Restaurant Delivery Robot

Autonomous 4-wheel mecanum robot that delivers orders across a simulated restaurant.
Built with **ROS 2 Humble + Ignition Gazebo Fortress**.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange) ![Nav2](https://img.shields.io/badge/Nav2-tuned-green)

## Features

- Omnidirectional mecanum drive through tight doorways
- SLAM Toolbox mapping + AMCL localization (`OmniMotionModel`)
- Nav2 path planning with dynamic obstacle avoidance
- Sensor fusion: RF2O laser odometry + IMU via robot_localization EKF
- Tkinter operator console: order queue, live mini-map, ETA, E-STOP
- 8 registered tables across Main Dining, Private Dining, Lounge, Kitchen

## Architecture

```
/scan ──► RF2O ──► /odom_laser ─┐
                                 ├──► EKF ──► /odom ──► map → odom → base_footprint
/imu  ─────────────────────────┘
                                        │
         Nav2 (AMCL + DWB) ◄────────────┘
                │
   NavigateToPose action ◄── Operator GUI (tkinter)
```

## Quick Start

```bash
# Build
cd ~/mecanum_ws && colcon build --symlink-install
source install/setup.bash

# Terminal 1 — simulation + odometry
ros2 launch mecanum_bot sim.launch.py

# Terminal 2 — navigation (with saved map)
ros2 launch mecanum_bot nav2.launch.py

# Terminal 3 — RViz
ros2 launch mecanum_bot rviz.launch.py

# Terminal 4 — operator GUI
ros2 launch mecanum_bot gui.launch.py
```

Set initial pose in RViz (**2D Pose Estimate**), then send orders from the GUI.

## Mapping a new world

```bash
ros2 launch mecanum_bot sim.launch.py
ros2 launch mecanum_bot slam.launch.py
# teleop-drive around, then:
ros2 run nav2_map_server map_saver_cli -f ~/mecanum_ws/maps/restaurant_map
```

## Project Layout

```
mecanum_bot/
├── launch/          sim, slam, nav2, rviz, gui
├── config/          nav2_params.yaml, ekf.yaml, tables.yaml
├── urdf/            robot description
├── worlds/          restaurant_world.sdf
├── models/          furniture & props
└── scripts/         restaurant_gui.py + TF/odom helpers
```

## Key Design Decisions

- **`nav2_amcl::OmniMotionModel`** — differential model causes particle divergence on lateral motion
- **Split inflation radii** — global large (plan away from walls), local small (fit through doorways)
- **RF2O over Gazebo odom** — Gazebo mecanum odom only publishes after first `cmd_vel`, unusable at EKF startup
- **Staggered `ros_gz_bridge`** — clock immediate, sensors +6s delay for lazy plugin activation

## License

MIT
