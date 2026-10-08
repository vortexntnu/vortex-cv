#!/bin/bash
# Start the RoboSub perception and mission chain in simulation, in a tmux
# session: landmark_slam, waypoint_manager, the dummy perception
# (robosub_dummy_publisher) with its Foxglove helpers, and the mission
# behavior tree (robosub_mission, mission_sim.yaml). When the vehicle is up
# it turns on autonomous mode; the tree starts the run itself.
#
# The simulator, the controller and the Foxglove bridge are not started here:
# start them first with vortex-auv's launch_drone_sim.sh, e.g.
#   src/vortex-auv/utility_scripts/launch_drone_sim.sh --scenario robosub --low-res --keyboard-joy false --detach
# Usage: ./tmux_robosub_sim.sh [OPTIONS]   (see --help)

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Start the simulator first (vortex-auv):
  src/vortex-auv/utility_scripts/launch_drone_sim.sh --scenario robosub --low-res --keyboard-joy false --detach
  src/vortex-auv/utility_scripts/launch_drone_sim.sh --headless --detach    (light, no images)

Options:
  --seed <n>            Seed for the course roles of the dummy perception;
                        the same as the simulator's --seed (default: 7)
  --domain-id <id>      ROS_DOMAIN_ID to use (default: 0)
  --profile <name>      Dummy perception profile
                        (config/robosub_dummy_publisher_<name>.yaml):
                        realistic (default), unstable, erratic, or none
  --no-fov              Dummy perception publishes everything, not only what
                        the cameras see from the vehicle
  --tasks <list>        Comma-separated course elements for the dummy
                        perception, e.g. gate,torpedo_board (default: all)
  --tree <name>         Behavior tree to run: Main (the course, default) or
                        one task, e.g. TestGate
  --no-mission          Do not start the behavior tree
  --no-autonomy         Do not turn on autonomous mode (the tree waits for it)
  --detach              Start the session without attaching to it
  -h, --help            Show this help message

Windows: mission (landmark_slam, waypoint_manager), perception (dummy
perception, Foxglove helpers: sim odometry relay, detection markers),
bt (the mission tree), tools (commands).
Foxglove layout: src/vortex-auv/mission/landmark_slam/foxglove/landmark_slam.json.
Detach with Ctrl-b d; stop everything with
  tmux kill-session -t robosub_sim
EOF
}

SEED="7"
DOMAIN_ID="0"
PROFILE="realistic"
FOV="true"
TASKS=""
TREE="Main"
MISSION="true"
AUTONOMY="true"
DETACH="false"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --seed)        SEED="$2";      shift 2 ;;
        --domain-id)   DOMAIN_ID="$2"; shift 2 ;;
        --profile)     PROFILE="$2";   shift 2 ;;
        --no-fov)      FOV="false";    shift ;;
        --tasks)       TASKS="$2";     shift 2 ;;
        --tree)        TREE="$2";      shift 2 ;;
        --no-mission)  MISSION="false"; shift ;;
        --no-autonomy) AUTONOMY="false"; shift ;;
        --detach)      DETACH="true";  shift ;;
        -h|--help)     usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

# The workspace is four levels up: <ws>/src/vortex-cv/perception_setup/scripts.
WS="$(cd "$(dirname "$(readlink -f "$0")")/../../../.." && pwd)"
if [[ ! -f "$WS/install/setup.bash" ]]; then
    echo "No install/setup.bash in $WS; build the workspace first."
    exit 1
fi

SESSION="robosub_sim"
S="cd $WS && source install/setup.bash && export ROS_DOMAIN_ID=$DOMAIN_ID"
DUMMY_CONFIG="install/robosub_dummy_publisher/share/robosub_dummy_publisher/config"

# Dummy perception: the profile on top of the defaults, the command line
# parameters on top of both.
DUMMY_CMD="ros2 run robosub_dummy_publisher robosub_dummy_publisher_node --ros-args -r __ns:=/nautilus"
DUMMY_CMD="$DUMMY_CMD --params-file $DUMMY_CONFIG/robosub_dummy_publisher_params.yaml"
if [[ "$PROFILE" != "none" ]]; then
    if [[ ! -f "$WS/$DUMMY_CONFIG/robosub_dummy_publisher_$PROFILE.yaml" ]]; then
        echo "No dummy profile '$PROFILE' ($DUMMY_CONFIG/robosub_dummy_publisher_$PROFILE.yaml); build robosub_dummy_publisher?"
        exit 1
    fi
    DUMMY_CMD="$DUMMY_CMD --params-file $DUMMY_CONFIG/robosub_dummy_publisher_$PROFILE.yaml"
fi
DUMMY_CMD="$DUMMY_CMD -p seed:=$SEED -p use_field_of_view:=$FOV"
if [[ -n "$TASKS" ]]; then
    DUMMY_CMD="$DUMMY_CMD -p tasks:=[$TASKS]"
fi

# landmark_slam on the simulator's odometry through the relay
# (odom_nav: <drone>/odom -> <drone>/base_link, as on the vehicle).
SLAM_CMD="ros2 launch landmark_slam landmark_slam.launch.py env:=sim odom_topic:=odom_nav"
HELPERS_CMD="ros2 launch robosub_dummy_publisher foxglove_helpers.launch.py"
BT_CMD="ros2 launch perception_setup robosub_mission.launch.py config:=sim main_tree:=$TREE"

# Once the vehicle publishes odometry: autonomous mode. The tree waits for it
# (WaitForStart) and then starts the run.
AUTONOMY_CMD="echo 'Waiting for /nautilus/odom ...'
until timeout 5 ros2 topic echo --no-daemon /nautilus/odom nav_msgs/msg/Odometry --once --no-arr >/dev/null 2>&1; do sleep 2; done
ros2 service call /nautilus/set_killswitch vortex_msgs/srv/SetKillswitch '{killswitch_on: false}'
ros2 service call /nautilus/set_operation_mode vortex_msgs/srv/SetOperationMode '{requested_operation_mode: {operation_mode: 1}}'
clear
echo 'Autonomous mode on. Try:'
echo '  ros2 topic echo /nautilus/landmark_slam/nis'
echo '  ros2 run tf2_ros tf2_echo nautilus/odom nautilus/gate_search_rescue_entrance'
echo '  ros2 topic pub --times 3 -w 1 /nautilus/mission/wipe std_msgs/msg/Empty   # start over'"
HELP_CMD="clear && echo 'Turn on autonomous mode yourself:' && echo \"  ros2 service call /nautilus/set_killswitch vortex_msgs/srv/SetKillswitch '{killswitch_on: false}'\" && echo \"  ros2 service call /nautilus/set_operation_mode vortex_msgs/srv/SetOperationMode '{requested_operation_mode: {operation_mode: 1}}'\""

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# =============================================
# Window 1: mission (2 panes)
# =============================================
tmux new-session -d -s "$SESSION" -n "mission"

PANE_MAP=$(tmux list-panes -t "$SESSION:mission" -F '#{pane_id}')
tmux send-keys -t "$PANE_MAP" "clear && $S && $SLAM_CMD" Enter

PANE_WM=$(tmux split-window -h -t "$PANE_MAP" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_WM" "clear && $S && ros2 launch waypoint_manager waypoint_manager.launch.py" Enter

# =============================================
# Window 2: perception (2 panes)
# =============================================
tmux new-window -t "$SESSION" -n "perception"

PANE_DUMMY=$(tmux list-panes -t "$SESSION:perception" -F '#{pane_id}')
tmux send-keys -t "$PANE_DUMMY" "clear && $S && $DUMMY_CMD" Enter

PANE_HELPERS=$(tmux split-window -v -t "$PANE_DUMMY" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_HELPERS" "clear && $S && $HELPERS_CMD" Enter

# =============================================
# Window 3: bt (the mission tree)
# =============================================
tmux new-window -t "$SESSION" -n "bt"
PANE_BT=$(tmux list-panes -t "$SESSION:bt" -F '#{pane_id}')
if [[ "$MISSION" == "true" ]]; then
    tmux send-keys -t "$PANE_BT" "clear && $S && sleep 5 && $BT_CMD" Enter
else
    tmux send-keys -t "$PANE_BT" "clear && $S && echo 'Start the tree: $BT_CMD'" Enter
fi

# =============================================
# Window 4: tools
# =============================================
tmux new-window -t "$SESSION" -n "tools"
PANE_TOOLS=$(tmux list-panes -t "$SESSION:tools" -F '#{pane_id}')
if [[ "$AUTONOMY" == "true" ]]; then
    tmux send-keys -t "$PANE_TOOLS" "clear && $S && $AUTONOMY_CMD" Enter
else
    tmux send-keys -t "$PANE_TOOLS" "$S && $HELP_CMD" Enter
fi

tmux select-window -t "$SESSION:bt"
if [[ "$DETACH" == "true" ]]; then
    echo "tmux session '$SESSION' started; attach with: tmux attach -t $SESSION"
else
    tmux attach-session -t "$SESSION"
fi
