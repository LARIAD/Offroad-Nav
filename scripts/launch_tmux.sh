#!/usr/bin/env bash
# Launch the navigation stack components in a tmux session.
# Usage: ./scripts/launch_tmux.sh [--stage /workspace/off-road-navigation-stack/assets/easy.usd] [--sensor lidar|lidar_real|mono]

STAGE="/workspace/off-road-navigation-stack/assets/easy.usd"
SENSOR="lidar"
SESSION="navstack"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stage|-s) STAGE="$2"; shift 2 ;;
    --sensor)   SENSOR="$2"; shift 2 ;;
    --session)  SESSION="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: $0 [--stage <usd>] [--sensor lidar|lidar_real|mono] [--session <name>]"
      exit 0 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

case "$SENSOR" in
  lidar)      NAV_LAUNCH="lidar_sim_tuned.launch" ;;
  lidar_real) NAV_LAUNCH="lidar_real_params.launch" ;;
  mono)       NAV_LAUNCH="mono_vins.launch" ;;
  *) echo "Unknown sensor: $SENSOR"; exit 1 ;;
esac

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed."; exit 1
fi

# Resolve repo root (parent of scripts/)
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' already exists. Attach with: tmux attach -t $SESSION"
  exit 1
fi

SETUP="cd $REPO_ROOT && source nav_stack/devel/setup.bash"

# Window 0: roscore
tmux new-session  -d -s "$SESSION" -n roscore   -c "$REPO_ROOT"
tmux send-keys    -t "$SESSION:roscore" "$SETUP && roscore" C-m

# Window 1: Isaac Sim
tmux new-window   -t "$SESSION" -n isaac -c "$REPO_ROOT"
tmux send-keys    -t "$SESSION:isaac" \
  "./isaac/python.sh scripts/run_isaacenv.py --stage $STAGE --sensor $SENSOR --gui" C-m

# Window 1b: Isaac Sim GUI (standalone, no Python API)
tmux new-window   -t "$SESSION" -n isaac-gui -c "$REPO_ROOT"
# tmux send-keys    -t "$SESSION:isaac-gui" \
#   "./isaac/isaac-sim.sh --allow-root" C-m

# Window 2: Navigation stack
tmux new-window   -t "$SESSION" -n navstack -c "$REPO_ROOT"
# tmux send-keys    -t "$SESSION:navstack" \
#   "sleep 6 && $SETUP && roslaunch barakuda_bringup $NAV_LAUNCH" C-m

# Window 3: RViz
tmux new-window   -t "$SESSION" -n rviz -c "$REPO_ROOT"
# tmux send-keys    -t "$SESSION:rviz" "sleep 8 && $SETUP && rviz" C-m

# Window 4: Goals (commands ready, not auto-sent)
tmux new-window   -t "$SESSION" -n goal -c "$REPO_ROOT"
# tmux send-keys    -t "$SESSION:goal" "$SETUP" C-m
# tmux send-keys    -t "$SESSION:goal" \
#   "# 10m goal:
# rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: \"odom\" }, pose: { position: {x: 10.735741716354974, y: -6.221199084140879, z: -0.07207081933418766}, orientation: {x: 0.0007582089773353305, y: -0.0008468118832714624, z: -0.051846601730829, w: 0.998293494973446} } }' --once" ""

tmux select-window -t "$SESSION:navstack"
echo "Started tmux session '$SESSION'. Attach with: tmux attach -t $SESSION"
if [ -n "$TMUX" ]; then
  tmux switch-client -t "$SESSION"
else
  tmux attach -t "$SESSION"
fi
