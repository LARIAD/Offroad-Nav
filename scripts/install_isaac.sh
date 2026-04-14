#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
ISAAC_DIR="${ISAAC_DIR:-${PARENT_DIR}/isaac}"
EXTS_USER_DIR="${ISAAC_DIR}/extsUser"

echo "[INFO] Linking Terrain Generator"
# Create symlink from forest_generator to isaac/extUser
mkdir -p "$EXTS_USER_DIR"
LINK_TARGET="${EXTS_USER_DIR}/terrain.generator"

if [[ -L "$LINK_TARGET" || -e "$LINK_TARGET" ]]; then
    echo "[INFO] Removing existing extUser/forest_generator..."
    rm -rf "$LINK_TARGET"
fi

ln -s "../../terrain.generator" "$LINK_TARGET"
echo "[INFO] Installing Terrain Generator Dependencies"
env -u PYTHONPATH "${ISAAC_DIR}"/python.sh -m pip install perlin_noise

echo "[INFO] Installing ROS Python deps into Isaac's bundled Python"
env -u PYTHONPATH "${ISAAC_DIR}"/python.sh -m pip install rospkg pyyaml netifaces defusedxml catkin_pkg

echo "[INFO] Generating Dome Lidar configuration and adding to isaac..."
env -u PYTHONPATH "${ISAAC_DIR}"/python.sh -m pip install numpy
env -u PYTHONPATH "${ISAAC_DIR}"/python.sh "${SCRIPT_DIR}"/generate_dome_lidar.py

mkdir -p "${ISAAC_DIR}/exts/isaacsim.sensors.rtx/data/lidar_configs/Ouster/OS0/"
mv "${PARENT_DIR}"/osdome_singleshot_datasheet_spec.json "${ISAAC_DIR}/exts/isaacsim.sensors.rtx/data/lidar_configs/Ouster/OS0/"

echo "Done!"
