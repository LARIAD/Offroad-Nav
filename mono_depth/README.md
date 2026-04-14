# Monocular Depth Estimation

ROS Noetic workspace for monocular depth estimation, used in the **mono** sensor mode of the navigation stack.

## Overview

This module uses a monocular camera feed to estimate dense depth maps, which are then converted to point clouds and fed into the elevation mapping pipeline. This enables navigation without a LiDAR sensor.

## Build

```bash
source /opt/ros/noetic/setup.bash
cd mono_depth
catkin build
source devel/setup.bash
```

## Docker

A dedicated Docker image is provided for the depth estimation pipeline:

```bash
cd docker/
docker build -t mono_depth:latest -f Dockerfile.monodepth .
```

The image includes TensorRT, Depth Anything V2, PyTorch, and all ROS dependencies. During the build, the [Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2) repository is cloned and the `vits` model checkpoint is downloaded from HuggingFace automatically.

## Packages

### `depth_rescaling`

Core package for monocular depth estimation and point cloud generation.

**Launch files:**

| Launch file | Description |
|-------------|-------------|
| `depth_estimation.launch` | Single-camera depth estimation |
| `nav_depth_estimation.launch` | Depth estimation for navigation |

**Configuration files** (in `config/`):

| Config | Description |
|--------|-------------|
| `depth_estimation.yaml` | Base depth estimation parameters |
| `nav_depth_cam0.yaml` | Camera 0 parameters |
| `nav_depth_cam1.yaml` | Camera 1 parameters |
| `nav_depth_cam2.yaml` | Camera 2 parameters |
| `nav_depth_estimation.yaml` | Navigation depth parameters |
| `pcl_registration_cfg.yaml` | Point cloud registration config |

## Usage with the Navigation Stack

When using `--sensor mono` with `launch_nav.sh`, the middle camera (`cam2`) is used. The depth estimation node must be running separately (either natively or in the mono_depth Docker container) to convert camera images to point clouds before they reach the elevation mapping node.

```bash
roslaunch depth_estimation nav_depth_estimation.launch
```

Alternatively, the monocular depth container can be launched via Docker Compose (the `mono_depth` service is gated behind the `on-demand` profile): (see [docker/README.md](docker/README.md))

```bash
cd docker/
docker compose up nav_stack
docker compose --profile on-demand up mono_depth
```

Make sure the navigation stack is launched with `--config monovins` so the monocular point cloud is consumed by the pipeline (this is already the default in [docker/docker-compose.yml](../docker/docker-compose.yml)).