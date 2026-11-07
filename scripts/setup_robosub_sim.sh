#!/bin/bash
# Everything the RoboSub simulator needs, from nothing or from a workspace you already have.
# Finds the workspace itself, or give it: setup_robosub_sim.sh ~/code/ros2_ws
# --jobs <n> builds less at a time, for a laptop that runs out of memory.
# Safe to run again.

set -eo pipefail

BRANCH="rework/landmark-members"
GITHUB="https://github.com/vortexntnu"

find_workspace() {
    local dir found
    if [[ -n "${1:-}" ]]; then
        echo "${1%/}"
        return
    fi
    # Inside a workspace
    dir="$PWD"
    while [[ "$dir" != "/" ]]; do
        if [[ -d "$dir/src/vortex-cv" ]]; then
            echo "$dir"
            return
        fi
        dir="$(dirname "$dir")"
    done
    found="$(find "$HOME" -maxdepth 5 -type d -name vortex-cv -path '*/src/vortex-cv' 2>/dev/null | sed 's|/src/vortex-cv$||' | sort -u)"
    if [[ -z "$found" ]]; then
        echo "$HOME/ros2_ws"
    elif [[ "$(echo "$found" | wc -l)" -eq 1 ]]; then
        echo "$found"
    else
        echo "Found more than one workspace, run this again with the one you want at the end:" >&2
        echo "$found" >&2
        return 1
    fi
}

GIVEN=""
if [[ $# -gt 0 && "$1" != -* ]]; then
    GIVEN="$1"
    shift
fi
WS="$(find_workspace "$GIVEN")"
JOBS=()
ARGS=("$@")
for ((i = 0; i < ${#ARGS[@]}; i++)); do
    [[ "${ARGS[$i]}" == "--jobs" ]] && JOBS=(--parallel-workers "${ARGS[$((i + 1))]}")
done
mkdir -p "$WS/src"
echo "==> Workspace: $WS"

MISSING=()
for pkg in tmux ros-humble-gtsam ros-humble-behaviortree-cpp ros-humble-foxglove-bridge; do
    dpkg -s "$pkg" >/dev/null 2>&1 || MISSING+=("$pkg")
done
if [[ ${#MISSING[@]} -gt 0 ]]; then
    echo "==> Installing ${MISSING[*]}"
    sudo apt-get update
    sudo apt-get install -y "${MISSING[@]}"
fi

cd "$WS/src"
[[ -d vortex-vkf ]] || git clone "$GITHUB/vortex-vkf.git"
[[ -d vortex-stonefish-sim ]] || git clone "$GITHUB/vortex-stonefish-sim.git"
git -C vortex-stonefish-sim switch main
git -C vortex-stonefish-sim pull --ff-only

# A build from another branch breaks the next one, so this goes before anything is built.
rm -rf "$WS/build" "$WS/install" "$WS/log"

# Puts the repos on the right branches and builds Stonefish. Stops if you have uncommitted changes.
vortex-stonefish-sim/scripts/update_stonefish_sim.sh --branch "$BRANCH" --ws "$WS" "$@"

cd "$WS"
set +u
source /opt/ros/humble/setup.bash
colcon build --symlink-install "${JOBS[@]}" --packages-up-to stonefish_sim stonefish_sim_interface \
    landmark_server vortex_bt_nodes robosub_mission robosub_dummy_publisher perception_setup \
    auv_setup waypoint_manager dp_adapt_backs_controller_quat reference_filter_dp_quat \
    thrust_allocator_auv operation_mode_manager joystick_interface_auv keyboard_joy

cat <<EOF

==> Done. Try it:
    cd $WS
    src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh --spawn torpedo --tree Main
EOF
