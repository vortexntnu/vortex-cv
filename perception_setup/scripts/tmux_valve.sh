#!/bin/bash
# Launch valve intervention stack in a tmux session.
# Usage: ./tmux_valve.sh [OPTIONS]
#   GStreamer is enabled by default; pass --no-gst to disable.

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --sim                    Flag. Use simulator topics, skip real hardware.
  --real                   Flag. Launch real camera hardware (default).
  --gst                    Flag. Enable GStreamer streaming (default).
  --no-gst                 Flag. Disable GStreamer streaming.
  --destination-ip <ip>    Value. Destination IP for GStreamer RTP stream. e.g. --destination-ip 10.0.0.50
  --hw-encoder             Flag. Use NVIDIA hardware H.265 encoder (default).
  --sw-encoder             Flag. Use software x265 encoder.
  --isaac-ros              Flag. Use isaac_ros TensorRT OBB backend (default).
  --ultralytics            Flag. Use Ultralytics Python OBB backend.
  --drone <name>           Value. Robot name used as topic/TF namespace prefix (default: nautilus).
  --domain-id <id>         Value. ROS_DOMAIN_ID to use (default: 0).
  -h, --help               Show this help message.

Unspecified options use the defaults defined in the ROS 2 launch files.
EOF
}

# GStreamer and hw-encoder default to true for this script.
GST_ARG="true"
SIM_ARG=""
DESTINATION_IP=""
NVIDIA_ENCODER=""
BACKEND=""
DRONE=""
DOMAIN_ID="0"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim)            SIM_ARG="true"; shift ;;
        --real)           SIM_ARG="false"; shift ;;
        --gst)            GST_ARG="true"; shift ;;
        --no-gst)         GST_ARG="false"; shift ;;
        --destination-ip) DESTINATION_IP="$2"; shift 2 ;;
        --hw-encoder)     NVIDIA_ENCODER="true"; shift ;;
        --sw-encoder)     NVIDIA_ENCODER="false"; shift ;;
        --isaac-ros)      BACKEND="isaac_ros"; shift ;;
        --ultralytics)    BACKEND="ultralytics"; shift ;;
        --drone)          DRONE="$2"; shift 2 ;;
        --domain-id)      DOMAIN_ID="$2"; shift 2 ;;
        -h|--help)        usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

# Build arg string for valve_intervention.launch.py — only forward args that were explicitly set.
VALVE_ARGS=" enable_gstreamer:=$GST_ARG"
[[ -n "$SIM_ARG" ]]        && VALVE_ARGS+=" sim:=$SIM_ARG"
[[ -n "$DESTINATION_IP" ]] && VALVE_ARGS+=" destination_ip:=$DESTINATION_IP"
[[ -n "$NVIDIA_ENCODER" ]] && VALVE_ARGS+=" gst_nvidia_encoder:=$NVIDIA_ENCODER"
[[ -n "$BACKEND" ]]        && VALVE_ARGS+=" backend:=$BACKEND"
[[ -n "$DRONE" ]]          && VALVE_ARGS+=" drone:=$DRONE"

S="source install/setup.bash && export ROS_DOMAIN_ID=$DOMAIN_ID"

SESSION="valve"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# =============================================
# Window 1: perception
# =============================================
tmux new-session -d -s "$SESSION" -n "perception"

PANE_P1=$(tmux list-panes -t "$SESSION:perception" -F '#{pane_id}')
tmux send-keys -t "$PANE_P1" "$S && ros2 launch perception_setup valve_intervention.launch.py$VALVE_ARGS" Enter

PANE_P2=$(tmux split-window -h -t "$PANE_P1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P2" "$S" Enter

PANE_P3=$(tmux split-window -v -t "$PANE_P1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P3" "$S" Enter

PANE_P4=$(tmux split-window -v -t "$PANE_P2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P4" "$S" Enter

tmux select-layout -t "$SESSION:perception" tiled

# =============================================
# Window 2: auto
# =============================================
tmux new-window -t "$SESSION" -n "auto"

PANE_A1=$(tmux list-panes -t "$SESSION:auto" -F '#{pane_id}')
tmux send-keys -t "$PANE_A1" "$S && ros2 launch waypoint_manager waypoint_manager.launch.py" Enter

PANE_A2=$(tmux split-window -h -t "$PANE_A1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A2" "$S && ros2 launch landmark_server landmark_server.launch.py" Enter

PANE_A3=$(tmux split-window -v -t "$PANE_A1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A3" "$S && ros2 launch visual_inspection_setup visual_inspection_fsm.launch.py" Enter

PANE_A4=$(tmux split-window -v -t "$PANE_A2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A4" "$S" Enter

tmux select-layout -t "$SESSION:auto" tiled

# =============================================
# Window 3: misc
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
# Focus perception window and attach
# =============================================
tmux select-window -t "$SESSION:perception"
tmux attach-session -t "$SESSION"
