# Shell Scripts

Shell environment and helper functions for operating the Husky UGV. Sourced automatically from `~/.bashrc` via `husky_setup.sh`.

## Quick Start

```sh
# 1. Setup ROS environment (runs automatically on login)
husky_ros_setup

# 2. Launch sensors (LIDAR, GPS, IMU, DHT22) with PTP time sync
husky_launch_sensors

# 3. (optional) Record all active topics
husky_record_rosbag
```

## Main Functions

| Function | Description |
|----------|-------------|
| `husky_ros_setup` | Source catkin workspace, export `ROS_MASTER_URI`, mount data disk, set hardware IPs |
| `husky_launch_base` | Restart `husky_base.service` (motors, teleop, odometry) |
| `husky_launch_sensors` | Sync LIDAR PTP clock, restart `sensors.service` |
| `husky_launch_localization` | Restart `localization.service` (dual EKF + NavSat) |
| `husky_launch_multiespectral_camera` | Launch multiespectral cameras (Basler RGB + FLIR LWIR) |
| `husky_launch_fisheye_cameras` | Launch fisheye cameras (front + rear Basler) |
| `husky_check_sensors` | Live frequency table for all sensor topics |
| `husky_record_rosbag` | Record all currently published topics to a bag file |

**F-key shortcuts** (terminal): F5=base, F6=sensors, F7=localization, F8=multiespectral, F9=fisheye, F10=sensor check.

## Files

| File | Purpose |
|------|---------|
| `husky_setup.sh` | Main entry point — sources everything, defines user-facing functions and F-key bindings |
| `husky_private_functions.sh` | Internal helpers: hardware IP resolution, LIDAR time sync, URDF setup |
| `log_utils.sh` | Logging helpers with color formatting |
| `husky_conky_monitor.sh` | Launcher for the Conky desktop widgets |

## Verifying LIDAR Time Sync

```sh
rostopic echo /ouster/points | grep "secs"
rostopic echo /imu/data | grep "secs"
```

Both should show timestamps within a few milliseconds of each other.

## GPS Base Station Setup

Use the antenna configuration GUI (see [python/](../python/)) to set the base station coordinates over serial:

```sh
python gui_antena_config.py [--port PORT] [--baudrate BAUDRATE]
```

## Systemd Services

These functions wrap `systemctl restart` calls. The service files are defined in [systemctl_services/](../systemctl_services/) — see its README for symlink and enable commands.
