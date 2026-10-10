#!/bin/bash
# Start the RoboSub simulator stack in a tmux session.
# Run from the workspace root. Usage: ./tmux_robosub_sim.sh [OPTIONS]

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Detections:
  --profile <name>     Detection noise: none, realistic, unstable, erratic (default: none).
                       none = exact positions, every object always visible.
  --tasks <list>       Objects to publish, e.g. gate,slalom (default: all).
  --seed <n>           Role image seed, same for simulator and detections (default: 7).

Mission:
  --tree <name>        Tree to run when everything is up, e.g. TestGate.
                       Without it the command is typed in the mission pane, press Enter to run.
  --no-start           Leave the killswitch on and the mode manual.

Simulator:
  --no-gpu             No rendering, no course to look at.
  --no-keyboard-joy    Do not start the keyboard joystick.
  --no-foxglove        Do not start the Foxglove bridge.
  --no-debug           Do not publish landmark markers and NIS.

Other:
  --domain-id <id>     ROS_DOMAIN_ID (default: 0).
  --detach             Do not attach to the session.
  -h, --help           Show this help message.

Stop everything with: tmux kill-session -t robosub_sim
EOF
}

PROFILE="none"
TASKS=""
SEED="7"
TREE=""
START="true"
GPU="true"
KEYBOARD_JOY="true"
FOXGLOVE="true"
DEBUG="true"
DOMAIN_ID="0"
ATTACH="true"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)     PROFILE="$2"; shift 2 ;;
        --tasks)       TASKS="$2"; shift 2 ;;
        --seed)        SEED="$2"; shift 2 ;;
        --tree)        TREE="$2"; shift 2 ;;
        --no-start)    START="false"; shift ;;
        --no-gpu)      GPU="false"; shift ;;
        --no-keyboard-joy) KEYBOARD_JOY="false"; shift ;;
        --no-foxglove) FOXGLOVE="false"; shift ;;
        --no-debug)    DEBUG="false"; shift ;;
        --domain-id)   DOMAIN_ID="$2"; shift 2 ;;
        --detach)      ATTACH="false"; shift ;;
        -h|--help)     usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

case "$PROFILE" in
    none|realistic|unstable|erratic) ;;
    *) echo "Unknown profile: $PROFILE"; usage; exit 1 ;;
esac

# --- Commands ---------------------------------------------------------------

if [[ "$GPU" == "true" ]]; then
    SIM_ARGS="scenario:=robosub rendering_quality:=low window_res_x:=960 window_res_y:=540 robosub_icon_seed:=$SEED"
else
    SIM_ARGS="scenario:=nautilus_no_gpu rendering:=false"
fi
# mock_odom:=false: the relay below is the only publisher of odom -> base_link.
SIM_CMD="ros2 launch stonefish_sim vortex_sim_launch.py $SIM_ARGS mock_odom:=false keyboard_joy:=$KEYBOARD_JOY"

RELAY_CMD="ros2 run robosub_dummy_publisher sim_odom_relay_node --ros-args -r __ns:=/nautilus -p odom_in:=odom/stonefish -p odom_out:=odom -p pose_out:=pose -p twist_out:=twist"

DUMMY_CMD="ros2 launch robosub_dummy_publisher robosub_dummy_publisher.launch.py seed:=$SEED"
[[ "$PROFILE" != "none" ]] && DUMMY_CMD+=" profile:=$PROFILE"
[[ -n "$TASKS" ]]          && DUMMY_CMD+=" tasks:=$TASKS"

SERVER_CMD="ros2 launch landmark_server landmark_server.launch.py env:=sim debug:=$DEBUG"

START_CMD="until ros2 service list | grep -q set_operation_mode; do sleep 2; done"
START_CMD+=" && ros2 service call /nautilus/set_killswitch vortex_msgs/srv/SetKillswitch '{killswitch_on: false}'"
START_CMD+=" && ros2 service call /nautilus/set_operation_mode vortex_msgs/srv/SetOperationMode '{requested_operation_mode: {operation_mode: 1}}'"

MISSION_CMD="ros2 launch perception_setup robosub_mission.launch.py config:=sim main_tree:=${TREE:-Main}"

S="source install/setup.bash && export ROS_DOMAIN_ID=$DOMAIN_ID"
SESSION="robosub_sim"

tmux kill-session -t "$SESSION" 2>/dev/null

# --- Window 1: sim (simulator, controller, waypoint manager, odometry) ------

tmux new-session -d -s "$SESSION" -n "sim"

PANE_SIM=$(tmux list-panes -t "$SESSION:sim" -F '#{pane_id}')
tmux send-keys -t "$PANE_SIM" "$S && $SIM_CMD" Enter

PANE_DP=$(tmux split-window -h -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_DP" "$S && sleep 15 && ros2 launch auv_setup dp_quat.launch.py" Enter

PANE_WM=$(tmux split-window -v -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_WM" "$S && sleep 15 && ros2 launch waypoint_manager waypoint_manager.launch.py" Enter

PANE_RELAY=$(tmux split-window -v -t "$PANE_DP" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_RELAY" "$S && sleep 15 && $RELAY_CMD" Enter

tmux select-layout -t "$SESSION:sim" tiled

# --- Window 2: map (detections, landmark server, start, Foxglove) -----------

tmux new-window -t "$SESSION" -n "map"

PANE_DUMMY=$(tmux list-panes -t "$SESSION:map" -F '#{pane_id}')
tmux send-keys -t "$PANE_DUMMY" "$S && sleep 20 && $DUMMY_CMD" Enter

PANE_SERVER=$(tmux split-window -h -t "$PANE_DUMMY" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_SERVER" "$S && sleep 20 && $SERVER_CMD" Enter

PANE_START=$(tmux split-window -v -t "$PANE_DUMMY" -P -F '#{pane_id}')
if [[ "$START" == "true" ]]; then
    tmux send-keys -t "$PANE_START" "$S && $START_CMD" Enter
else
    tmux send-keys -t "$PANE_START" "$S" Enter
fi

PANE_FOX=$(tmux split-window -v -t "$PANE_SERVER" -P -F '#{pane_id}')
if [[ "$FOXGLOVE" == "true" ]]; then
    tmux send-keys -t "$PANE_FOX" "$S && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter
else
    tmux send-keys -t "$PANE_FOX" "$S" Enter
fi

tmux select-layout -t "$SESSION:map" tiled

# --- Window 3: mission ------------------------------------------------------

tmux new-window -t "$SESSION" -n "mission"

PANE_MISSION=$(tmux list-panes -t "$SESSION:mission" -F '#{pane_id}')
if [[ -n "$TREE" ]]; then
    tmux send-keys -t "$PANE_MISSION" "$S && sleep 45 && $MISSION_CMD" Enter
else
    tmux send-keys -t "$PANE_MISSION" "$S" Enter
    tmux send-keys -t "$PANE_MISSION" "$MISSION_CMD"
fi

PANE_FREE=$(tmux split-window -h -t "$PANE_MISSION" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_FREE" "$S" Enter

tmux select-window -t "$SESSION:sim"
[[ "$ATTACH" == "true" ]] && tmux attach-session -t "$SESSION"
