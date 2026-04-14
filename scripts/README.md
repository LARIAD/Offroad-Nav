# Scripts

Utility and launch scripts for the off-road navigation stack.

## Files

### `launch_nav.sh`

**Main launch script** — starts the full navigation pipeline (roscore, Isaac Sim, elevation mapping, move_base) in a single command.

```bash
# Headless mode (no GUI)
./scripts/launch_nav.sh --stage assets/easy.usd --headless --sensor lidar

# GUI mode (with Isaac Sim viewport)
./scripts/launch_nav.sh --stage assets/easy.usd --sensor lidar

# With RViz and a nav goal
./scripts/launch_nav.sh --stage assets/medium.usd --sensor mono --rviz \
    --goal '{header: {frame_id: "odom"}, pose: {position: {x: 10, y: 0, z: 0}, orientation: {w: 1}}}'
```

| Option | Default | Description |
|--------|---------|-------------|
| `--stage, -s` | *(required)* | Path to USD stage file |
| `--headless` | off (GUI) | Run Isaac Sim without a viewport |
| `--sensor` | `lidar` | Sensor mode: `lidar`, `mono`, or `stereo` |
| `--rviz` | off | Launch RViz |
| `--port` | `11311` | ROS master port |
| `--goal` | *(none)* | Navigation goal (PoseStamped YAML) |
| `--timeout` | `0` (unlimited) | Auto-stop after N minutes |

Press **Ctrl+C** to gracefully shut down all processes.

### `run_isaacenv.py`

Python script that runs Isaac Sim, loads a USD stage, starts the simulation, and manages the sensor graph (enabling/disabling cameras and lidar based on the chosen sensor mode).

```bash
# Headless
./isaac/python.sh scripts/run_isaacenv.py --stage assets/easy.usd --sensor lidar

# With GUI
./isaac/python.sh scripts/run_isaacenv.py --stage assets/easy.usd --sensor lidar --gui

# With trajectory recording
./isaac/python.sh scripts/run_isaacenv.py --stage assets/easy.usd --sensor lidar --record --output-dir ./results
```

| Option | Default | Description |
|--------|---------|-------------|
| `--stage, -s` | *(required)* | USD stage file path |
| `--sensor` | *(required)* | `lidar`, `mono`, or `stereo` |
| `--gui` | off | Run with Isaac Sim GUI instead of headless |
| `--odom-topic, -o` | `/imu/odometry` | Odometry topic for recording |
| `--output-dir, -d` | `/lvhome/run-stack-nav` | Output directory for trajectory CSV/PDF |
| `--record` | off | Enable trajectory recording |

### `install_isaac.sh`

Post-installation setup script. Run this after extracting Isaac Sim into `isaac/`:

1. Links the `terrain.generator` extension into `isaac/extsUser/`
2. Installs the `perlin_noise` Python dependency for terrain generation
3. Generates the dome lidar configuration (`osdome_singleshot_datasheet_spec.json`)

```bash
./scripts/install_isaac.sh
```

### `generate_dome_lidar.py`

Generates a custom Ouster OS0 dome lidar configuration file and places it in the Isaac Sim lidar configs directory. Called automatically by `install_isaac.sh`.
