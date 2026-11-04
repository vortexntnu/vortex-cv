#!/bin/bash
set -euo pipefail

SCENARIO="${1:-sim}"
SESSION="visual_inspection_${SCENARIO}"
WS="$HOME/ros2_ws"
SRC="cd $WS && source install/setup.bash"

tmux kill-session -t "$SESSION" 2>/dev/null || true

# --- MISSION ---
tmux new-session -d -s "$SESSION" -n mission
tmux send-keys -t "$SESSION" "$SRC && ros2 launch visual_inspection_setup waypoint_landmark.launch.py" C-m

# --- FSM ---
tmux new-window -t "$SESSION" -n fsm
tmux send-keys -t "$SESSION" "$SRC && ros2 launch visual_inspection_setup valve_inspection_fsm.launch.py" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$SRC && ros2 run yasmin_viewer yasmin_viewer_node" C-m

tmux select-layout tiled
tmux attach-session -t "$SESSION"
