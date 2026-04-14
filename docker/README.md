# Docker Setup

This directory contains Dockerfiles and orchestration for running the navigation stack in containers.

## Images

| Dockerfile | Image | Description |
|------------|-------|-------------|
| `Dockerfile.navstack` | `nav_stack:latest` | Full navigation stack: Isaac Sim 4.5.0, ROS Noetic, CUDA 11.6, PyTorch, elevation_mapping_cupy, move_base |
| `Dockerfile.monodepth` | `mono_depth:latest` | Monocular depth estimation: TensorRT, Depth Anything, ROS Noetic |

## Build

```bash
cd docker/

# Navigation stack image
docker build -t nav_stack:latest -f Dockerfile.navstack ..

# Monocular depth image (only needed for mono sensor mode)
docker build -t mono_depth:latest -f Dockerfile.monodepth ..
```

## Run

### Interactive container

```bash
./docker_run.sh
```

This starts an interactive shell inside the nav_stack container with:
- GPU passthrough (NVIDIA runtime)
- X11 forwarding for GUI applications (Isaac Sim, RViz)
- Isaac Sim cache volumes (persistent across runs)
- The repository mounted at `/workspace/Offroad-Nav`

### Using docker-compose

The `docker-compose.yml` provides a multi-container setup:

```bash
# Start the navigation stack
docker compose up nav_stack

# Start the mono depth container (on-demand profile)
docker compose --profile on-demand up mono_depth
```

> [!IMPORTANT]  
> Use `xhost +si:localuser:root` to ensure that the container can access the X11 server of the host.

## Inside the Container

Once inside the container, you can launch the navigation stack:

```bash
# Source the workspace
source /workspace/Offroad-Nav/nav_stack/devel/setup.bash

# Launch the full stack (headless)
/workspace/Offroad-Nav/scripts/launch_nav.sh \
    --stage /workspace/Offroad-Nav/assets/easy.usd \
    --headless --sensor lidar

# Or launch components manually — see the main README for details
```

## Requirements

- Docker >= 20.10
- NVIDIA Container Toolkit (`nvidia-container-toolkit`)
- NVIDIA GPU driver >= 525
