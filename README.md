# pairs_octomap_server

Builds and maintains a 3D occupancy map (OctoMap) of the UAV's surroundings from onboard
sensors. It fuses point clouds from 3D LiDARs, 2D laser scanners and depth cameras into a
global and a rolling local OctoMap, which the PAIRS planning and collision-avoidance stack
consumes for safe flight. This is the mapping front-end of the PAIRS UAV stack.

## Contents
- `pairs_octomap_server/PairsOctomapServer` — nodelet (`pairs_octomap_server::OctomapServer`)
  that integrates sensor data into the occupancy map, publishes the full and binary OctoMaps
  (global and local), and exposes map reset / save / load services.
- `PoseWithSize` message — a pose with width and height, used to clear obstacles inside a
  bounding box.

## Branches
- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 1 Noetic)
```bash
sudo apt install ros-noetic-pairs-octomap-server
```

## Usage
```bash
roslaunch pairs_octomap_server octomap.launch
```
Remap the sensor input topics (`lidar_3d_*`, `lidar_2d_*`, `depth_camera_*`, `camera_info_*`)
to your platform's sensors. Defaults are in `config/default.yaml`; override them with the
`custom_config` argument.

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_octomap_server` package; the original
copyright is retained in [LICENSE](LICENSE).
