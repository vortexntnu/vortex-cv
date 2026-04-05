#!/bin/bash
# Usage: ./pipeline_inspection.sh [sim|real_world]
set -euo pipefail

SCENARIO="${1:-sim}"
SESSION="pipeline_inspection_${SCENARIO}"
WS="$HOME/ros2_ws"
SRC="cd $WS && source install/setup.bash"

tmux kill-session -t "$SESSION" 2>/dev/null || true

# --- Always ---
tmux new-session -d -s "$SESSION" -n mission
tmux send-keys -t "$SESSION" "$SRC && ros2 launch pipeline_inspection_setup waypoint_landmark.launch.py" C-m

tmux new-window -t "$SESSION" -n fsm
tmux send-keys -t "$SESSION" "$SRC && ros2 launch pipeline_inspection_setup pipeline_inspection_fsm.launch.py" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$SRC && ros2 run yasmin_viewer yasmin_viewer_node" C-m

tmux new-window -t "$SESSION" -n localization
tmux send-keys -t "$SESSION" "$SRC && ros2 launch pipeline_inspection_setup bearing_localization.launch.py" C-m

# --- Sim only ---
if [[ "$SCENARIO" == "sim" ]]; then
    tmux new-window -t "$SESSION" -n aruco
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch pipeline_inspection_setup down_camera_aruco.launch.py" C-m
fi

# --- Real world only ---
if [[ "$SCENARIO" == "real_world" ]]; then
    tmux new-window -t "$SESSION" -n perception
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch perception_setup perception.launch.py enable_down_camera:=true enable_aruco:=true" C-m
fi

tmux attach-session -t "$SESSION"
