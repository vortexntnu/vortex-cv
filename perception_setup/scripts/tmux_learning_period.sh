#!/bin/bash
# Launch the learning_period onboarding FSM stack in a tmux session.
# Usage: ./tmux_learning_period.sh [OPTIONS]
#
# This mission has no camera/sonar perception of its own — it's a pure
# waypoint patrol built on vortex_yasmin_utils::WaypointGoalState — so unlike
# the other tmux_*.sh scripts here, there's no "perception" window and no
# --sim/--real flag to gate a camera driver.
#
# IMPORTANT: this script does NOT start the simulator. Nothing in this repo
# does — the other tmux_*.sh scripts' --sim flag only tells the *camera*
# launch files to skip the real driver and trust that something else is
# already publishing on those topics; it never starts that something else.
# Get your simulator running first (ask in the group channel if you don't
# already have this), then run this script on top of it.

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --drone <name>     Value. Robot name used as topic/TF namespace prefix (default: launch file default).
  --domain-id <id>   Value. ROS_DOMAIN_ID to use (default: 0).
  -h, --help         Show this help message.
EOF
}

DRONE=""
DOMAIN_ID="0"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --drone)      DRONE="$2"; shift 2 ;;
        --domain-id)  DOMAIN_ID="$2"; shift 2 ;;
        -h|--help)    usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

FSM_ARGS=""
[[ -n "$DRONE" ]] && FSM_ARGS+=" drone:=$DRONE"

S="source install/setup.bash && export ROS_DOMAIN_ID=$DOMAIN_ID"

SESSION="learning_period"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# =============================================
# Window 1: auto
# =============================================
tmux new-session -d -s "$SESSION" -n "auto"

PANE_A1=$(tmux list-panes -t "$SESSION:auto" -F '#{pane_id}')
tmux send-keys -t "$PANE_A1" "$S && ros2 launch waypoint_manager waypoint_manager.launch.py" Enter

PANE_A2=$(tmux split-window -h -t "$PANE_A1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A2" "$S && ros2 launch landmark_server landmark_server.launch.py" Enter

PANE_A3=$(tmux split-window -v -t "$PANE_A1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A3" "$S && ros2 launch learning_period learning_period_fsm.launch.py$FSM_ARGS" Enter

PANE_A4=$(tmux split-window -v -t "$PANE_A2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A4" "$S && ros2 run yasmin_viewer yasmin_viewer_node" Enter

tmux select-layout -t "$SESSION:auto" tiled

# =============================================
# Window 2: misc
# =============================================
tmux new-window -t "$SESSION" -n "misc"

PANE_M1=$(tmux list-panes -t "$SESSION:misc" -F '#{pane_id}')
tmux send-keys -t "$PANE_M1" "$S" Enter

PANE_M2=$(tmux split-window -h -t "$PANE_M1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_M2" "$S" Enter

PANE_M3=$(tmux split-window -v -t "$PANE_M1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_M3" "$S" Enter

PANE_M4=$(tmux split-window -v -t "$PANE_M2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_M4" "$S" Enter

tmux select-layout -t "$SESSION:misc" tiled

# =============================================
# Focus auto window and attach
# =============================================
tmux select-window -t "$SESSION:auto"
tmux attach-session -t "$SESSION"
