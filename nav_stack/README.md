# Navigation Stack

ROS Noetic catkin workspace containing all packages for autonomous off-road navigation.

## Build

```bash
source /opt/ros/noetic/setup.bash
cd nav_stack
catkin build
source devel/setup.bash
```

## Launch

All bringup is handled by `barakuda_bringup`. Three launch configurations are available:

```bash
# LiDAR tuned params for simulation
roslaunch barakuda_bringup lidar_sim_tuned.launch

# LiDAR with parameters used in the real robot
roslaunch barakuda_bringup lidar_real_params.launch

# Monocular configuration with parameters used in the real robot
roslaunch barakuda_bringup mono_vins.launch
```

Each launch file sets its module defaults and delegates to `barakuda_bringup/launch/base.launch.xml`. Modules can be overridden at launch time via args (`use_manager`, `use_dlio_odom`, `use_localisation`, `use_navigation`, `use_sensors`, `use_ptz`, `use_zed`).

In simulation, only the module `use_navigation` is used.

## Packages

### Core Navigation

| Package | Description                                                                                   |
|---------|-----------------------------------------------------------------------------------------------|
| `mbf_gridmap` | Navigation framework with global + local planners. It is based on move_base_flex and gridmap. |
| `teb_local_planner` | Timed-Elastic-Band local planner for dynamic obstacle avoidance                               |
| `elevation_mapping_cupy` | GPU-accelerated 2.5D elevation mapping from point clouds                                      |

### Robot Description and Control

| Package | Description                                        |
|---------|----------------------------------------------------|
| `barakuda_description` | URDF model and meshes for the Barakuda UGV         |
| `barakuda_manager` | High-level robot state management                  |
| `waypoint_manager` | Waypoint-based navigation management               |
| `barakuda_node` | Low-level robot control interface                  |
| `barakuda_bringup` | Launch files and configs for bringing up the robot |

### Perception

| Package | Description |
|---------|-------------|
| `dlio` | Direct LiDAR-Inertial Odometry |
| `barakuda_vision` | Vision processing pipeline |

### Hardware Drivers

| Package | Description                         |
|---------|-------------------------------------|
| `ouster-ros` | Ouster LiDAR driver (ROS interface) |
| `sbg_driver` | SBG IMU driver                      |
| `axis_ptz_driver` | Axis PTZ camera driver              |
| `zed-ros-wrapper` | ZED stereo camera ROS wrapper       |

### Other

| Package | Description                         |
|---------|-------------------------------------|
| `virtual_costmap_layer` | Virtual obstacle layer for costmaps |
| `video_gstreamer_ros` | GStreamer video streaming for ROS   |
| `mirador_msgs` | Messages for Mirador HCI            |
| `ntrip_client` | NTRIP client for RTK corrections    |


## Frame Convention

The navigation stack uses the following TF frames:
- `odom` — odometry frame (world-fixed)
- `base_link` — robot body frame
- Sensor frames are defined in the URDF (`barakuda_description`)
