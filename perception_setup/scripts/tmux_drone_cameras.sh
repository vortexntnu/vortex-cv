#!/bin/bash
# Launch drone cameras only (RealSense front + Blackfly S down) in a tmux session.
# Usage: ./tmux_drone_cameras.sh [--drone <name>]

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --drone <name>   Robot name used as topic/TF namespace prefix (default: nautilus).
  -h, --help       Show this help message.
EOF
}

DRONE="nautilus"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --drone) DRONE="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

FRONT_TOPIC="/$DRONE/front_camera/image_color"
DOWN_TOPIC="/$DRONE/down_camera/image_color"

SESSION="drone_cameras"

tmux kill-session -t "$SESSION" 2>/dev/null

# =============================================
# Window 1: cameras
# =============================================
tmux new-session -d -s "$SESSION" -n "cameras"

PANE_C1=$(tmux list-panes -t "$SESSION:cameras" -F '#{pane_id}')
tmux send-keys -t "$PANE_C1" "source install/setup.bash && ros2 launch perception_setup realsense_d555.launch.py drone:=$DRONE" Enter

PANE_C2=$(tmux split-window -h -t "$PANE_C1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_C2" "source install/setup.bash && ros2 launch perception_setup blackfly_s.launch.py drone:=$DRONE" Enter

PANE_C3=$(tmux split-window -v -t "$PANE_C1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_C3" "source install/setup.bash && ros2 topic echo $FRONT_TOPIC" Enter

PANE_C4=$(tmux split-window -v -t "$PANE_C2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_C4" "source install/setup.bash && ros2 topic echo $DOWN_TOPIC" Enter

tmux select-layout -t "$SESSION:cameras" tiled

# =============================================
# Window 2: winword
# =============================================
tmux new-window -t "$SESSION" -n "winword"

PANE_W1=$(tmux list-panes -t "$SESSION:winword" -F '#{pane_id}')
tmux send-keys -t "$PANE_W1" "watch -n 1 'ethtool -S enP5p1s0f2 | grep -iE \"crc|rx_packets|rx_bytes\"'" Enter

PANE_W2=$(tmux split-window -h -t "$PANE_W1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_W2" "watch -n 1 'ethtool -S enP5p1s0f3 | grep -iE \"crc|rx_packets|rx_bytes\"'" Enter

tmux select-layout -t "$SESSION:winword" even-horizontal

# =============================================
# Focus cameras window and attach
# =============================================
tmux select-window -t "$SESSION:cameras"
tmux attach-session -t "$SESSION"
