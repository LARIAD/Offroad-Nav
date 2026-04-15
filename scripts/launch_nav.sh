#!/bin/bash
#
# launch_nav.sh — Launch the off-road navigation stack with Isaac Sim
#
# Usage:
#   ./scripts/launch_nav.sh --stage <path_to_usd> [OPTIONS]
#
# Examples:
#   # Headless mode (no GUI), lidar sensor:
#   ./scripts/launch_nav.sh --stage assets/easy.usd --headless --sensor lidar
#
#   # GUI mode, mono camera:
#   ./scripts/launch_nav.sh --stage assets/medium.usd --sensor mono
#
#   # With RViz visualization:
#   ./scripts/launch_nav.sh --stage assets/hard.usd --sensor lidar --rviz

set -e

# ─── Defaults ────────────────────────────────────────────────────────────────
HEADLESS=false
SENSOR="lidar"
CONFIG="lidarreal"
RVIZ=false
ROS_PORT=11311
STAGE=""
TIMEOUT_MINUTES=0   # 0 = unlimited
GOAL=""             # optional nav goal (YAML string)

# ─── Resolve repo root ───────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

ISAAC_DIR="/workspace/Offroad-Nav/isaac"
ISAAC_PYTHON="${ISAAC_DIR}/python.sh"
ISAAC_SIM_SCRIPT="${SCRIPT_DIR}/run_isaacenv.py"
NAV_WS="${REPO_DIR}/nav_stack"

# ─── Colors ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

log()      { echo -e "${GREEN}[$(date +'%H:%M:%S')] $1${NC}"; }
warn_log() { echo -e "${YELLOW}[$(date +'%H:%M:%S')] WARNING: $1${NC}"; }
error_log(){ echo -e "${RED}[$(date +'%H:%M:%S')] ERROR: $1${NC}" >&2; }

# ─── Parse arguments ─────────────────────────────────────────────────────────
usage() {
    cat <<EOF
Usage: $(basename "$0") --stage <USD_FILE> [OPTIONS]

Required:
  --stage, -s <path>        Path to the USD stage file (e.g. assets/easy.usd)

Options:
  --headless                Run Isaac Sim without GUI (default: GUI mode)
  --sensor <type>           Sensor type: lidar or mono (default: lidar)
  --config <type>           Sensor type: lidarreal, lidarsim or monovins (default: lidarreal)
  --rviz                    Launch RViz for visualization
  --port <port>             ROS master port (default: 11311)
  --goal <yaml>             Send a navigation goal (geometry_msgs/PoseStamped YAML)
  --timeout <minutes>       Timeout in minutes (default: 0 = unlimited)
  -h, --help                Show this help message
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --stage|-s)     STAGE="$2"; shift 2 ;;
        --headless)     HEADLESS=true; shift ;;
        --sensor)       SENSOR="$2"; shift 2 ;;
        --config)        CONFIG="$2"; shift 2 ;;
        --rviz)         RVIZ=true; shift ;;
        --port)         ROS_PORT="$2"; shift 2 ;;
        --goal)         GOAL="$2"; shift 2 ;;
        --timeout)      TIMEOUT_MINUTES="$2"; shift 2 ;;
        -h|--help)      usage ;;
        *)              error_log "Unknown argument: $1"; usage ;;
    esac
done

if [[ -z "$STAGE" ]]; then
    error_log "Missing required argument: --stage"
    usage
fi

# Resolve stage to absolute path if relative
if [[ ! "$STAGE" = /* ]]; then
    STAGE="${REPO_DIR}/${STAGE}"
fi

if [[ ! -f "$STAGE" ]]; then
    error_log "Stage file not found: $STAGE"
    exit 1
fi

# ─── Validate prerequisites ──────────────────────────────────────────────────
if [[ ! -f "$ISAAC_PYTHON" ]]; then
    error_log "Isaac Sim not found at ${ISAAC_DIR}. Run ./scripts/install_isaac.sh first."
    exit 1
fi

if [[ ! -f "$ISAAC_SIM_SCRIPT" ]]; then
    error_log "run_isaacenv.py not found at ${ISAAC_SIM_SCRIPT}"
    exit 1
fi

# ─── Process tracking ────────────────────────────────────────────────────────
ROSCORE_PID=""
ISAAC_PID=""
BRINGUP_PID=""
RVIZ_PID=""
CLEANING_UP=0

is_running() { kill -0 "$1" 2>/dev/null; }

cleanup() {
    [[ "$CLEANING_UP" == "1" ]] && return
    CLEANING_UP=1
    trap - SIGINT SIGTERM EXIT

    log "Shutting down..."

    for label_pid in "RViz:$RVIZ_PID" "barakuda_bringup:$BRINGUP_PID"; do
        local label="${label_pid%%:*}"
        local pid="${label_pid##*:}"
        if [[ -n "$pid" ]] && is_running "$pid"; then
            log "Stopping $label (PID: $pid)..."
            kill -INT "$pid" 2>/dev/null
            sleep 2
            is_running "$pid" && kill -KILL "$pid" 2>/dev/null
        fi
    done

    if [[ -n "$ISAAC_PID" ]] && is_running "$ISAAC_PID"; then
        log "Stopping Isaac Sim (PID: $ISAAC_PID)..."
        kill -INT "$ISAAC_PID" 2>/dev/null
        sleep 3
        is_running "$ISAAC_PID" && kill -KILL "$ISAAC_PID" 2>/dev/null
    fi

    if [[ -n "$ROSCORE_PID" ]] && is_running "$ROSCORE_PID"; then
        log "Stopping roscore (PID: $ROSCORE_PID)..."
        kill -INT "$ROSCORE_PID" 2>/dev/null
    fi

    log "Shutdown complete."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# ─── 1. Start roscore ────────────────────────────────────────────────────────
log "Starting roscore on port $ROS_PORT..."

if [[ -f "${NAV_WS}/devel/setup.bash" ]]; then
    source "${NAV_WS}/devel/setup.bash"
else
    warn_log "Nav stack workspace not built yet — sourcing system ROS only."
    if [[ -f /opt/ros/noetic/setup.bash ]]; then
        source /opt/ros/noetic/setup.bash
    else
        error_log "No ROS installation found. Source your ROS setup before running."
        exit 1
    fi
fi

export ROS_MASTER_URI="http://localhost:${ROS_PORT}"
roscore -p "$ROS_PORT" &
ROSCORE_PID=$!
sleep 5

if ! is_running "$ROSCORE_PID"; then
    error_log "roscore failed to start"
    exit 1
fi
log "roscore started (PID: $ROSCORE_PID)"

rosparam set /use_sim_time true

# ─── 2. Launch Isaac Sim ─────────────────────────────────────────────────────
if $HEADLESS; then
    log "Starting Isaac Sim (headless)..."
else
    log "Starting Isaac Sim (GUI)..."
fi

ISAAC_ARGS="--stage $STAGE --sensor $SENSOR"
if ! $HEADLESS; then
    ISAAC_ARGS="$ISAAC_ARGS --gui"
fi

# Isaac Sim
"$ISAAC_PYTHON" "$ISAAC_SIM_SCRIPT" $ISAAC_ARGS &
ISAAC_PID=$!

# Wait for Isaac Sim to start publishing
log "Waiting for Isaac Sim to initialize..."
MAX_WAIT=900
ELAPSED=0
while [[ $ELAPSED -lt $MAX_WAIT ]]; do
    if rostopic list 2>/dev/null | grep -q "/clock"; then
        if timeout 10 rostopic echo -n1 /clock &>/dev/null; then
            log "Isaac Sim is publishing (${ELAPSED}s)"
            break
        fi
    fi
    sleep 10
    ELAPSED=$((ELAPSED + 10))
    [[ $((ELAPSED % 30)) -eq 0 ]] && log "Still waiting for Isaac Sim... (${ELAPSED}s / ${MAX_WAIT}s)"
done

if [[ $ELAPSED -ge $MAX_WAIT ]]; then
    error_log "Isaac Sim failed to start within ${MAX_WAIT}s"
    exit 1
fi

sleep 5

# ─── 3. Launch navigation stack ──────────────────────────────────────────────
log "Launching navigation stack (barakuda_bringup)..."

case "$CONFIG" in
    monovins)       BRINGUP_LAUNCH="mono_vins.launch" ;;
    lidarreal) BRINGUP_LAUNCH="lidar_real_params.launch" ;;
    lidarsim)          BRINGUP_LAUNCH="lidar_sim_tuned.launch" ;;
esac

roslaunch barakuda_bringup "$BRINGUP_LAUNCH" &
BRINGUP_PID=$!
sleep 15

if ! is_running "$BRINGUP_PID"; then
    error_log "barakuda_bringup failed to start"
    exit 1
fi
log "barakuda_bringup started (PID: $BRINGUP_PID)"

# ─── 5. (Optional) Launch RViz ───────────────────────────────────────────────
if $RVIZ; then
    log "Launching RViz..."
    rviz -d "${SCRIPT_DIR}/rviz_config.rviz" &
    RVIZ_PID=$!
fi

# ─── 6. (Optional) Send navigation goal ──────────────────────────────────────
if [[ -n "$GOAL" ]]; then
    log "Sending navigation goal via MBF action..."
    sleep 2
    rostopic pub /move_base_flex/move_base/goal mbf_msgs/MoveBaseActionGoal "$GOAL" --once
    # rostopic pub /move_base_flex/move_base/goal mbf_msgs/MoveBaseActionGoal '{ header: { seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: "" }, goal_id: { stamp: {secs: 0, nsecs: 0}, id: "" }, goal: { target_pose: { header: { seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: "odom" }, pose: { position: {x: 20.0, y: 13.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} } }, controller: "TebLocalPlannerROS", planner: "GlobalPlanner", recovery_behaviors: [""] } }' --once

    if [[ $? -eq 0 ]]; then
        log "Navigation goal sent"
    else
        warn_log "Failed to send navigation goal"
    fi
fi

# ─── 7. Main loop ────────────────────────────────────────────────────────────
log "Navigation stack is running. Press Ctrl+C to stop."
log "  Isaac Sim  : PID $ISAAC_PID $(if $HEADLESS; then echo '(headless)'; else echo '(GUI)'; fi)"
log "  Nav stack  : PID $BRINGUP_PID ($BRINGUP_LAUNCH)"
[[ -n "$RVIZ_PID" ]] && log "  RViz      : PID $RVIZ_PID"

START_TIME=$(date +%s)
while true; do
    sleep 5

    # Check critical processes
    if ! is_running "$ISAAC_PID"; then
        warn_log "Isaac Sim exited"
        break
    fi
    if ! is_running "$BRINGUP_PID"; then
        warn_log "barakuda_bringup exited"
        break
    fi

    # Timeout check
    if [[ $TIMEOUT_MINUTES -gt 0 ]]; then
        ELAPSED=$(( $(date +%s) - START_TIME ))
        if [[ $ELAPSED -ge $((TIMEOUT_MINUTES * 60)) ]]; then
            warn_log "Timeout reached (${TIMEOUT_MINUTES} min)"
            break
        fi
    fi
done

cleanup
