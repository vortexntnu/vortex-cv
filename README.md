[![Industrial CI](https://github.com/vortexntnu/vortex-cv/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-cv/actions/workflows/industrial-ci.yml)
[![pre-commit](https://github.com/vortexntnu/vortex-cv/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/vortexntnu/vortex-cv/actions/workflows/pre-commit.yml)
[![codecov](https://codecov.io/gh/vortexntnu/vortex-cv/graph/badge.svg?token=GoXxR2nR0y)](https://codecov.io/gh/vortexntnu/vortex-cv)

# Introduction

```
TODO: Write a simple description / introduction to the repository
```

# Setup

Copy this into a terminal. It finds your workspace, installs what is missing, puts the
repos on the right branches and builds. It takes a while the first time, and you can run
it again. Commit what you have first, it stops on uncommitted changes.

```bash
bash <(curl -fsSL https://raw.githubusercontent.com/vortexntnu/vortex-cv/rework/landmark-members/scripts/setup_robosub_sim.sh)
```

With more than one workspace, it asks you to put the one you want at the end:
`bash <(curl ...) ~/code/ros2_ws`. If your laptop runs out of memory while building, add
`--jobs 2` after the workspace.

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
