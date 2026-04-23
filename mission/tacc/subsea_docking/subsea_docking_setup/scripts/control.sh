#!/bin/bash
# Usage: ./control.sh [sim|real_world]
set -euo pipefail

SCENARIO="${1:-sim}"
SESSION="subsea_docking_control_${SCENARIO}"
WS="$HOME/ros2_ws"
SRC="cd $WS && source install/setup.bash"
SIM_CONFIG="$WS/src/vortex-cv/mission/tacc/subsea_docking/subsea_docking_setup/config/sim_config.yaml"

tmux kill-session -t "$SESSION" 2>/dev/null || true

# --- Sim only ---
if [[ "$SCENARIO" == "sim" ]]; then
    tmux new-session -d -s "$SESSION" -n sim
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch stonefish_sim vortex_sim_launch.py scenario:=docking rendering_quality:=low scenario_config_override:=$SIM_CONFIG" C-m
else
    tmux new-session -d -s "$SESSION" -n control
fi

# --- Control (thruster, controller, guidance) ---
if [[ "$SCENARIO" == "sim" ]]; then
    tmux new-window -t "$SESSION" -n control
fi
tmux send-keys -t "$SESSION" "$SRC && ros2 launch auv_setup dp.launch.py" C-m

tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$SRC && ros2 launch keyboard_joy keyboard_joy_node.launch.py" C-m

# --- Real world only ---
if [[ "$SCENARIO" == "real_world" ]]; then
    tmux split-window -h -t "$SESSION"
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch auv_setup thruster.launch.py" C-m

    tmux split-window -h -t "$SESSION"
    tmux send-keys -t "$SESSION" "$SRC && ros2 launch operation_mode_manager operation_mode_manager.launch.py" C-m
fi

tmux select-layout -t "$SESSION" tiled

if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "$SESSION"
else
    tmux attach-session -t "$SESSION"
fi
