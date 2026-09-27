#!/bin/bash
# Launch subsea docking stack in a tmux session
# Usage: ./tmux_subsea_docking.sh [--sim | --real] [--gst | --no-gst]
#   --sim     Pass sim:=true to all launch files (skip hardware, use simulator topics)
#   --real    Pass sim:=false (default) — launch real cameras and sonar hardware
#   --gst     Enable GStreamer streaming for front and down camera (enable_gstreamer:=true)
#   --no-gst  Disable GStreamer streaming (default)

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --sim                    Flag. Use simulator topics, skip real hardware.
  --real                   Flag. Launch real cameras and sonar hardware.
  --gst                    Flag. Enable GStreamer streaming for all sensors.
  --no-gst                 Flag. Disable GStreamer streaming.
  --destination-ip <ip>    Value. Destination IP for GStreamer RTP stream. e.g. --destination-ip 10.0.0.50
  --hw-encoder             Flag. Use NVIDIA hardware H.265 encoder.
  --sw-encoder             Flag. Use software x265 encoder.
  --dock-distance <m>      Value. Waypoint distance [m] for YOLO direction waypoint. Default: launch file default (7.5).
  -h, --help               Show this help message.

Unspecified options use the defaults defined in the ROS 2 launch files.
EOF
}

SIM_ARG=""
GST_ARG=""
DESTINATION_IP=""
NVIDIA_ENCODER=""
DOCK_DISTANCE=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim)            SIM_ARG="true"; shift ;;
        --real)           SIM_ARG="false"; shift ;;
        --gst)            GST_ARG="true"; shift ;;
        --no-gst)         GST_ARG="false"; shift ;;
        --destination-ip) DESTINATION_IP="$2"; shift 2 ;;
        --hw-encoder)     NVIDIA_ENCODER="true"; shift ;;
        --sw-encoder)     NVIDIA_ENCODER="false"; shift ;;
        --dock-distance)  DOCK_DISTANCE="$2"; shift 2 ;;
        -h|--help)        usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

# Only forward args that were explicitly set — unset args fall back to launch file defaults.
FRONT_ARGS=""; DOWN_ARGS=""; SONAR_ARGS=""
[[ -n "$SIM_ARG" ]]        && FRONT_ARGS+=" sim:=$SIM_ARG"                      && DOWN_ARGS+=" sim:=$SIM_ARG"                    && SONAR_ARGS+=" sim:=$SIM_ARG"
[[ -n "$GST_ARG" ]]        && FRONT_ARGS+=" enable_gstreamer:=$GST_ARG"          && DOWN_ARGS+=" enable_gstreamer:=$GST_ARG"        && SONAR_ARGS+=" enable_gstreamer:=$GST_ARG"
[[ -n "$DESTINATION_IP" ]] && FRONT_ARGS+=" destination_ip:=$DESTINATION_IP"     && DOWN_ARGS+=" destination_ip:=$DESTINATION_IP"   && SONAR_ARGS+=" destination_ip:=$DESTINATION_IP"
[[ -n "$NVIDIA_ENCODER" ]] && FRONT_ARGS+=" gst_nvidia_encoder:=$NVIDIA_ENCODER" && DOWN_ARGS+=" gst_nvidia_encoder:=$NVIDIA_ENCODER" && SONAR_ARGS+=" gst_nvidia_encoder:=$NVIDIA_ENCODER"
[[ -n "$DOCK_DISTANCE" ]]  && FRONT_ARGS+=" waypoint_distance:=$DOCK_DISTANCE"

SESSION="subsea_docking"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# =============================================
# Window 1: perception (cameras)
# =============================================
tmux new-session -d -s "$SESSION" -n "perception"

PANE_P1=$(tmux list-panes -t "$SESSION:perception" -F '#{pane_id}')
tmux send-keys -t "$PANE_P1" "source install/setup.bash && ros2 launch perception_setup subsea_docking_front_camera.launch.py$FRONT_ARGS" Enter

PANE_P2=$(tmux split-window -h -t "$PANE_P1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P2" "source install/setup.bash && ros2 launch perception_setup subsea_docking_down_camera.launch.py$DOWN_ARGS" Enter

PANE_P3=$(tmux split-window -v -t "$PANE_P1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P3" "source install/setup.bash && ros2 launch perception_setup subsea_docking_sonar.launch.py$SONAR_ARGS" Enter

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
tmux send-keys -t "$PANE_A3" "source install/setup.bash && ros2 launch perception_setup subsea_docking_fsm.launch.py" Enter

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
