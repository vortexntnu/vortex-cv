[![Industrial CI](https://github.com/vortexntnu/vortex-cv/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-cv/actions/workflows/industrial-ci.yml)
[![pre-commit](https://github.com/vortexntnu/vortex-cv/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/vortexntnu/vortex-cv/actions/workflows/pre-commit.yml)
[![codecov](https://codecov.io/gh/vortexntnu/vortex-cv/graph/badge.svg?token=GoXxR2nR0y)](https://codecov.io/gh/vortexntnu/vortex-cv)

# Introduction

```
TODO: Write a simple description / introduction to the repository
```

# Setup

The script clones the simulator and its repos, and builds Stonefish. It takes a
while the first time.

```bash
sudo apt install -y tmux ros-humble-gtsam ros-humble-behaviortree-cpp ros-humble-foxglove-bridge
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/vortexntnu/vortex-vkf.git
git clone https://github.com/vortexntnu/vortex-stonefish-sim.git
vortex-stonefish-sim/scripts/update_stonefish_sim.sh --branch rework/landmark-members
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to stonefish_sim stonefish_sim_interface \
  landmark_server vortex_bt_nodes robosub_mission robosub_dummy_publisher perception_setup \
  auv_setup waypoint_manager dp_adapt_backs_controller_quat reference_filter_dp_quat \
  thrust_allocator_auv operation_mode_manager joystick_interface_auv keyboard_joy
```

Build only these, the rest are not needed for the simulator. If your laptop runs
short on memory, add `--parallel-workers 2`.

Then start the simulator, see `mission/robosub/robosub_mission`. On the vehicle:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/vortex/fastdds.xml
```

## Parked packages

`vortex_yasmin_utils` and the docking, pipeline and visual inspection state
machines have a `COLCON_IGNORE` file, so colcon skips them. They use
`convergence_threshold` and the landmark polling and convergence actions,
which this branch no longer has. Delete the file in a package to build it
again once it is ported.

## Using pre-commit

This project uses [pre-commit](https://github.com/pre-commit/pre-commit) to manage automated checks.

### Install pre-commit
```bash
pip install pre-commit
```

### Run all hooks manually
```bash
pre-commit run --all-files
```

### Install the git hook
This will make the checks run automatically every time you `git commit`:
```bash
pre-commit install
```

### Update pre-commit hooks
```bash
pre-commit autoupdate
```

## GitHub Actions CI
- This project uses GitHub Actions for CI.
- Workflows are located in [`.github/workflows`](.github/workflows/).
- For more information, see [vortex-ci](https://github.com/vortexntnu/vortex-ci).
