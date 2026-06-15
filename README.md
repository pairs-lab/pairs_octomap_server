# pairs_octomap_server

Builds and maintains a 3D occupancy map (OctoMap) of the UAV's surroundings from onboard
sensors. It fuses point clouds from 3D LiDARs and depth cameras into a global and a rolling
local OctoMap, which the PAIRS planning and collision-avoidance stack consumes for safe
flight. This is the mapping front-end of the PAIRS UAV stack.

## Contents
- `pairs_octomap_server::OctomapServer` — a composable node (registered for use in an
  `rclcpp_components` container) that integrates sensor data into the occupancy map,
  publishes the full and binary OctoMaps (global and local), and exposes map reset / save /
  load services.

## Branches
- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 2 Jazzy)
```bash
sudo apt install ros-jazzy-pairs-octomap-server
```

## Usage
```bash
ros2 launch pairs_octomap_server octomap_server.launch.py
```
Remap the sensor input topics (`lidar_3d_*`, `depth_camera_*`, `camera_info_*`) to your
platform's sensors. Defaults are in `config/default.yaml`; override them with the
`custom_config` argument. The node can run standalone or be loaded into an existing
component container (`standalone:=false container_name:=<name>`).

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_octomap_server` package; the original
copyright is retained in [LICENSE](LICENSE).
