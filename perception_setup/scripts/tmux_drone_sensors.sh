#!/bin/bash
# Launch drone sensor stack (cameras + optional sonar) in a tmux session.
# Usage: ./tmux_drone_sensors.sh [OPTIONS]
#   GStreamer is enabled by default; pass --no-gst to disable.

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --gst                    Flag. Enable GStreamer streaming for all sensors (default).
  --no-gst                 Flag. Disable GStreamer streaming.
  --destination-ip <ip>    Value. Destination IP for GStreamer RTP stream. e.g. --destination-ip 10.0.0.50
  --hw-encoder             Flag. Use NVIDIA hardware H.265 encoder.
  --sw-encoder             Flag. Use software x265 encoder.
  --sonar                  Flag. Launch sonar (default).
  --no-sonar               Flag. Skip sonar, launch cameras only.
  -h, --help               Show this help message.

Unspecified options use the defaults defined in the ROS 2 launch files.
EOF
}

# GStreamer defaults to true for this script.
GST_ARG="true"
DESTINATION_IP=""
NVIDIA_ENCODER=""
LAUNCH_SONAR="true"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --gst)            GST_ARG="true"; shift ;;
        --no-gst)         GST_ARG="false"; shift ;;
        --destination-ip) DESTINATION_IP="$2"; shift 2 ;;
        --hw-encoder)     NVIDIA_ENCODER="true"; shift ;;
        --sw-encoder)     NVIDIA_ENCODER="false"; shift ;;
        --sonar)          LAUNCH_SONAR="true"; shift ;;
        --no-sonar)       LAUNCH_SONAR="false"; shift ;;
        -h|--help)        usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

# Build per-sensor arg strings — only forward args that were explicitly set.
REALSENSE_ARGS=" enable_gstreamer:=$GST_ARG"
BLACKFLY_ARGS=" enable_gstreamer:=$GST_ARG"
SONAR_ARGS=" enable_gstreamer:=$GST_ARG"

[[ -n "$DESTINATION_IP" ]] && REALSENSE_ARGS+=" destination_ip:=$DESTINATION_IP" \
                            && BLACKFLY_ARGS+=" destination_ip:=$DESTINATION_IP" \
                            && SONAR_ARGS+=" destination_ip:=$DESTINATION_IP"
[[ -n "$NVIDIA_ENCODER" ]] && REALSENSE_ARGS+=" gst_nvidia_encoder:=$NVIDIA_ENCODER" \
                            && BLACKFLY_ARGS+=" gst_nvidia_encoder:=$NVIDIA_ENCODER" \
                            && SONAR_ARGS+=" gst_nvidia_encoder:=$NVIDIA_ENCODER"

SESSION="drone_sensors"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# =============================================
# Window 1: sensors
# =============================================
tmux new-session -d -s "$SESSION" -n "sensors"

PANE_S1=$(tmux list-panes -t "$SESSION:sensors" -F '#{pane_id}')
tmux send-keys -t "$PANE_S1" "source install/setup.bash && ros2 launch perception_setup realsense_d555.launch.py$REALSENSE_ARGS" Enter

PANE_S2=$(tmux split-window -h -t "$PANE_S1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_S2" "source install/setup.bash && ros2 launch perception_setup blackfly_s.launch.py$BLACKFLY_ARGS" Enter

PANE_S3=$(tmux split-window -v -t "$PANE_S1" -P -F '#{pane_id}')
if [[ "$LAUNCH_SONAR" == "true" ]]; then
    tmux send-keys -t "$PANE_S3" "source install/setup.bash && ros2 launch perception_setup sonar.launch.py$SONAR_ARGS" Enter
else
    tmux send-keys -t "$PANE_S3" "source install/setup.bash" Enter
fi

PANE_S4=$(tmux split-window -v -t "$PANE_S2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_S4" "source install/setup.bash" Enter

tmux select-layout -t "$SESSION:sensors" tiled

# =============================================
# Window 2: misc
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
# Focus sensors window and attach
# =============================================
tmux select-window -t "$SESSION:sensors"
tmux attach-session -t "$SESSION"
