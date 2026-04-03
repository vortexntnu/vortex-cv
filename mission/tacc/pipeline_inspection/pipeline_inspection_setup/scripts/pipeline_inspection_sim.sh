#!/bin/bash

SESSION="pipeline_inspection_sim"
WS="$HOME/ros2_ws"

# Kill old session if it exists
tmux kill-session -t $SESSION 2>/dev/null


# --- MISSION ---
tmux new-session -d -s $SESSION -n mission

tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch pipeline_inspection_setup waypoint_landmark.launch.py" C-m


# --- FSM ---
tmux new-window -t $SESSION -n fsm

tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch pipeline_inspection_setup pipeline_inspection_fsm.launch.py" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 run yasmin_viewer yasmin_viewer_node" C-m


# --- PERCEPTION ---
tmux new-window -t $SESSION -n perception

tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch pipeline_inspection_setup down_camera_aruco.launch.py use_sim:=true" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch pipeline_inspection_setup bearing_localization.launch.py" C-m

# Nice layout
tmux select-layout tiled

# Attach to session
tmux attach-session -t $SESSION
