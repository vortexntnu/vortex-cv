#!/bin/bash
set -euo pipefail

SCENARIO="${1:-sim}"
SESSION="pipeline_inspection_${SCENARIO}"
WS="$HOME/ros2_ws"
SRC="cd $WS && source install/setup.bash"
ARGS="scenario:=${SCENARIO}"

tmux kill-session -t "$SESSION" 2>/dev/null || true

# --- MISSION ---
tmux new-session -d -s "$SESSION" -n mission
tmux send-keys -t "$SESSION" "$SRC && ros2 launch pipeline_inspection_setup waypoint_landmark.launch.py" C-m

# --- FSM ---
tmux new-window -t "$SESSION" -n fsm
tmux send-keys -t "$SESSION" "$SRC && ros2 launch pipeline_inspection_setup pipeline_inspection_fsm.launch.py" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$SRC && ros2 run yasmin_viewer yasmin_viewer_node" C-m

# --- PERCEPTION ---
tmux new-window -t "$SESSION" -n perception
tmux send-keys -t "$SESSION" "$SRC && ros2 launch pipeline_inspection_setup down_camera_aruco.launch.py $ARGS" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$SRC && ros2 launch pipeline_inspection_setup bearing_localization.launch.py $ARGS" C-m

tmux select-layout tiled
tmux attach-session -t "$SESSION"
