#!/bin/bash
# Launch subsea docking stack in a tmux session

SESSION="subsea_docking"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# =============================================
# Window 1: perception (cameras)
# =============================================
tmux new-session -d -s "$SESSION" -n "perception"

PANE_P1=$(tmux list-panes -t "$SESSION:perception" -F '#{pane_id}')
tmux send-keys -t "$PANE_P1" "source install/setup.bash && ros2 launch perception_setup subsea_docking_front_camera.launch.py" Enter

PANE_P2=$(tmux split-window -h -t "$PANE_P1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P2" "source install/setup.bash && ros2 launch perception_setup subsea_docking_down_camera.launch.py" Enter

PANE_P3=$(tmux split-window -v -t "$PANE_P1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P3" "source install/setup.bash && ros2 launch perception_setup subsea_docking_sonar.launch.py" Enter

PANE_P4=$(tmux split-window -v -t "$PANE_P2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P4" "source install/setup.bash" Enter

tmux select-layout -t "$SESSION:perception" tiled

# =============================================
# Window 2: auto
# =============================================
tmux new-window -t "$SESSION" -n "auto"

PANE_A1=$(tmux list-panes -t "$SESSION:auto" -F '#{pane_id}')
tmux send-keys -t "$PANE_A1" "source install/setup.bash && ros2 launch waypoint_manager waypoint_manager.launch.py" Enter

PANE_A2=$(tmux split-window -h -t "$PANE_A1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A2" "source install/setup.bash && ros2 launch landmark_server landmark_server.launch.py" Enter

PANE_A3=$(tmux split-window -v -t "$PANE_A1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A3" "source install/setup.bash && ros2 launch subsea_docking_setup subsea_docking_fsm.launch.py" Enter

PANE_A4=$(tmux split-window -v -t "$PANE_A2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A4" "source install/setup.bash" Enter

tmux select-layout -t "$SESSION:auto" tiled

# =============================================
# Window 3: misc
# =============================================
tmux new-window -t "$SESSION" -n "misc"

PANE_M1=$(tmux list-panes -t "$SESSION:misc" -F '#{pane_id}')
tmux send-keys -t "$PANE_M1" "source install/setup.bash" Enter

PANE_M2=$(tmux split-window -h -t "$PANE_M1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_M2" "source install/setup.bash" Enter

PANE_M3=$(tmux split-window -v -t "$PANE_M1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_M3" "source install/setup.bash" Enter

PANE_M4=$(tmux split-window -v -t "$PANE_M2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_M4" "source install/setup.bash" Enter

tmux select-layout -t "$SESSION:misc" tiled

# =============================================
# Focus perception window and attach
# =============================================
tmux select-window -t "$SESSION:perception"
tmux attach-session -t "$SESSION"
