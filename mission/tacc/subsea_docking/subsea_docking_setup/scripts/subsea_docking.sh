#!/bin/bash
# Usage: ./subsea_docking.sh [sim|real_world]
set -euo pipefail

SCENARIO="${1:-sim}"
SESSION="subsea_docking_${SCENARIO}"
WS="$HOME/ros2_ws"
SRC="cd $WS && source install/setup.bash"

tmux kill-session -t "$SESSION" 2>/dev/null || true

# --- Always ---
tmux new-session -d -s "$SESSION" -n mission
tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup waypoint_landmark.launch.py" C-m

tmux new-window -t "$SESSION" -n fsm
tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup subsea_docking_fsm.launch.py" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$SRC && ros2 run yasmin_viewer yasmin_viewer_node" C-m

# --- Sim only ---
if [[ "$SCENARIO" == "sim" ]]; then
    tmux new-window -t "$SESSION" -n aruco
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup front_camera_aruco.launch.py" C-m
    tmux split-window -h -t "$SESSION"
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup down_camera_aruco.launch.py" C-m

    tmux new-window -t "$SESSION" -n sonar
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup sonar_docking.launch.py sim:=true" C-m
fi

# --- Real world only ---
if [[ "$SCENARIO" == "real_world" ]]; then
    tmux new-window -t "$SESSION" -n perception
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch perception_setup perception.launch.py enable_front_camera:=true enable_aruco:=true" C-m

    tmux new-window -t "$SESSION" -n sonar
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch subsea_docking_setup sonar_docking.launch.py sim:=false" C-m
fi

tmux attach-session -t "$SESSION"
