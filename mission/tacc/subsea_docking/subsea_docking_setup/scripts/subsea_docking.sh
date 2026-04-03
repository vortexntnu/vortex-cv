#!/bin/bash
set -euo pipefail

SCENARIO="${1:-sim}"
SESSION="subsea_docking_${SCENARIO}"
WS="$HOME/ros2_ws"
SRC="cd $WS && source install/setup.bash"
ARGS="scenario:=${SCENARIO}"

tmux kill-session -t "$SESSION" 2>/dev/null || true

# --- MISSION ---
tmux new-session -d -s "$SESSION" -n mission
tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup waypoint_landmark.launch.py" C-m

# --- FSM ---
tmux new-window -t "$SESSION" -n fsm
tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup subsea_docking_fsm.launch.py" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$SRC && ros2 run yasmin_viewer yasmin_viewer_node" C-m

# --- PERCEPTION ---
tmux new-window -t "$SESSION" -n perception
tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup front_camera_aruco.launch.py $ARGS" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup down_camera_aruco.launch.py $ARGS" C-m

# --- SONAR (launched in all scenarios, driver only starts in real_world) ---
tmux new-window -t "$SESSION" -n sonar
tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup sonar.launch.py $ARGS" C-m

tmux select-layout tiled
tmux attach-session -t "$SESSION"
