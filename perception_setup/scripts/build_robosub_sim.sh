#!/bin/bash
# Build only the packages the RoboSub simulator stack needs.
# Run from the workspace root. Usage: ./build_robosub_sim.sh [OPTIONS]

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS] [-- COLCON_ARGS]

  --jobs <n>     Parallel packages and make jobs (default: 2).
  --clean        Remove build/ and install/ of these packages first.
  -h, --help     Show this help message.

Everything after -- is passed to colcon build.
EOF
}

JOBS=2
CLEAN=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --jobs) JOBS="$2"; shift 2 ;;
        --clean) CLEAN=true; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; break ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
done

if [[ ! -d src ]]; then
    echo "Run this from the workspace root (the folder with src/)."
    exit 1
fi

# Dependencies of these are built too.
PACKAGES=(
    # Simulator
    stonefish_sim
    stonefish_sim_interface
    # Control
    auv_setup
    dp_adapt_backs_controller_quat
    reference_filter_dp_quat
    thrust_allocator_auv
    operation_mode_manager
    # Manual control
    joystick_interface_auv
    keyboard_joy
    # Navigation and map
    waypoint_manager
    landmark_server
    # Mission
    robosub_dummy_publisher
    robosub_mission
)

# behaviortree_cpp comes from apt (ros-humble-behaviortree-cpp).
SKIP=(behaviortree_cpp)

source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j$JOBS"

if $CLEAN; then
    mapfile -t ALL < <(colcon list --names-only --packages-up-to "${PACKAGES[@]}" \
        --packages-skip "${SKIP[@]}" 2>/dev/null)
    for pkg in "${ALL[@]}" perception_setup; do
        rm -rf "build/$pkg" "install/$pkg"
    done
fi

colcon build --symlink-install --parallel-workers "$JOBS" \
    --packages-up-to "${PACKAGES[@]}" \
    --packages-skip "${SKIP[@]}" "$@" || exit 1

# Built alone: its package.xml lists every mission package as a dependency.
colcon build --symlink-install --packages-select perception_setup "$@" || exit 1

echo "Done. Run: source install/setup.bash"
