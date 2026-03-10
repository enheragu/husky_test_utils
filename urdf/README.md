# URDF Extras

`husky_extras.urdf.xacro` adds fixed-joint sensor frames to the Husky TF tree:

| Frame | Parent | Position (x, y, z) |
|-------|--------|---------------------|
| `os_sensor` (Ouster LIDAR) | `top_plate_link` | -0.075, 0, 0.58 |
| `gps_link` (GPS antenna) | `top_plate_front_link` | 0.08, 0, 0.245 |

Loaded automatically by `husky_setup.sh` via `_husky_setup_urdf()`.
