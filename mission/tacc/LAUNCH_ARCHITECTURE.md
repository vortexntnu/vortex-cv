# FSM Launch Architecture

This document covers the launch pattern used for all TACC FSMs:
- Subsea Docking
- Pipeline Inspection
- Visual Inspection

---

## Core Idea

Every FSM has a `*_setup` package that contains launch files, scenario configs, and a bash script.

The key design decision: **one scenario YAML per environment holds everything that differs between sim and real world**. Launch files load that YAML and adapt. No scattered boolean flags, no separate board config files.

```
*_setup/
├── config/
│   └── scenarios/
│       ├── sim.yaml          ← all sim-specific params
│       └── real_world.yaml   ← all real-world-specific params
├── launch/
│   ├── *_fsm.launch.py
│   ├── waypoint_landmark.launch.py
│   └── <perception_component>.launch.py
└── scripts/
    └── *.sh                  ← single script, takes scenario as argument
```

---

## How to Launch

Build and source first:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Then run the bash script with a scenario argument:
```bash
# Subsea docking
./src/vortex-cv/mission/tacc/subsea_docking/subsea_docking_setup/scripts/subsea_docking.sh sim
./src/vortex-cv/mission/tacc/subsea_docking/subsea_docking_setup/scripts/subsea_docking.sh real_world

# Pipeline inspection
./src/vortex-cv/mission/tacc/pipeline_inspection/pipeline_inspection_setup/scripts/pipeline_inspection.sh sim
./src/vortex-cv/mission/tacc/pipeline_inspection/pipeline_inspection_setup/scripts/pipeline_inspection.sh real_world

# Visual inspection
./src/vortex-cv/mission/tacc/visual_inspection/visual_inspection_setup/scripts/visual_inspection.sh sim
./src/vortex-cv/mission/tacc/visual_inspection/visual_inspection_setup/scripts/visual_inspection.sh real_world
```

Each script opens a tmux session with separate windows for mission, FSM, and perception. The session is named `<fsm>_<scenario>` so you can have sim and real running side-by-side.

To launch a single component manually:
```bash
ros2 launch subsea_docking_setup front_camera_aruco.launch.py scenario:=sim
ros2 launch subsea_docking_setup front_camera_aruco.launch.py scenario:=real_world
```

---

## How Scenario Files Work

A scenario YAML lives at `config/scenarios/<name>.yaml`. It contains every value that differs between environments.

Example — `subsea_docking_setup/config/scenarios/sim.yaml`:
```yaml
scenario:
  name: sim
  use_camera_driver: false       # do not launch Spinnaker driver in sim

  aruco_board:
    marker_size: 0.150           # TAC board
    xDist: 0.430
    yDist: 0.830
    ids: [28, 7, 96, 19]

  front_camera:
    image_topic: "/{ns}/front_camera"
    camera_info_topic: "/{ns}/front_camera/camera_info"
    camera_frame: "/{ns}/front_camera_link"

  down_camera:
    image_topic: "/{ns}/down_camera/image_color"
    camera_info_topic: "/{ns}/down_camera/camera_info"
    camera_frame: "/{ns}/down_camera_link"

  sonar:
    launch_driver: false
```

The `{ns}` placeholder gets replaced by the actual namespace at launch time (see `load_scenario()` below).

Example — `config/scenarios/real_world.yaml`:
```yaml
scenario:
  name: real_world
  use_camera_driver: true        # launch Spinnaker driver

  aruco_board:
    marker_size: 0.140           # Vortex docking plate
    xDist: 0.448
    yDist: 0.853
    ids: [28, 7, 96, 19]

  front_camera:
    image_topic: "/{ns}/front_camera/image_raw"
    camera_info_topic: "/{ns}/front_camera/camera_info"
    camera_frame: "/{ns}/front_camera_link"
    serial_number: "23494259"    # only needed when driver is active

  down_camera:
    image_topic: "/{ns}/down_camera/image_raw"
    camera_info_topic: "/{ns}/down_camera/camera_info"
    camera_frame: "/{ns}/down_camera_link"
    serial_number: "23494258"

  sonar:
    launch_driver: true
```

---

## How a Launch File Works — Full Walkthrough

We will go through `front_camera_aruco.launch.py` from scratch.

### The goal

This launch file starts a composable node container with:
- In **sim**: only an ArUco detector node (subscribes to a sim topic)
- In **real_world**: a camera driver node + ArUco detector node in the same process (intra-process comms, no image serialisation overhead)

### Step 1 — Boilerplate imports

```python
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
```

- `declare_drone_and_namespace_args` / `resolve_drone_and_namespace` — standard helpers from `auv_setup` that handle the `drone` and `namespace` launch arguments
- `OpaqueFunction` — lets you run arbitrary Python at launch time (needed because argument values are only known at runtime, not at parse time)
- `ComposableNodeContainer` / `ComposableNode` — ROS 2 composable node API

### Step 2 — The scenario loader helper

```python
def load_scenario(scenario, namespace):
    path = os.path.join(
        get_package_share_directory("subsea_docking_setup"),
        "config", "scenarios", f"{scenario}.yaml",
    )
    with open(path) as f:
        raw = f.read().replace("{ns}", namespace)
    return yaml.safe_load(raw)["scenario"]
```

This:
1. Finds the installed location of the YAML using the package name
2. Substitutes `{ns}` with the actual namespace string
3. Parses the YAML and returns the `scenario` dict

After calling this, `cfg["aruco_board"]["marker_size"]` gives you `0.150`, etc.

### Step 3 — The launch_setup function

`OpaqueFunction` calls this at launch time, after all arguments are resolved.

```python
def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    scenario = context.launch_configurations["scenario"]
    cfg = load_scenario(scenario, namespace)
```

`context.launch_configurations` is a dict of all resolved launch argument values.

### Step 4 — Build the ArUco node

```python
    perception_share = get_package_share_directory("perception_setup")
    aruco_base_params = os.path.join(perception_share, "config", "aruco_detector_params.yaml")

    cam = cfg["front_camera"]
    board = cfg["aruco_board"]

    aruco_node = ComposableNode(
        package="aruco_detector",
        plugin="vortex::aruco_detector::ArucoDetectorNode",
        name="front_aruco_detector",
        namespace=namespace,
        parameters=[
            aruco_base_params,           # base defaults from perception_setup
            {                            # scenario-specific overrides
                "aruco.marker_size": board["marker_size"],
                "board.xDist":       board["xDist"],
                "board.yDist":       board["yDist"],
                "board.ids":         board["ids"],
                "subs.image_topic":       cam["image_topic"],
                "subs.camera_info_topic": cam["camera_info_topic"],
                "camera_frame":           cam["camera_frame"],
            },
        ],
        # intra-process comms only makes sense when the driver is in the same container
        extra_arguments=[{"use_intra_process_comms": cfg["use_camera_driver"]}],
    )

    nodes = [aruco_node]
```

Parameters are applied in order — the dict at the end overrides values from the YAML file.

### Step 5 — Conditionally add the camera driver

```python
    if cfg["use_camera_driver"]:
        camera_params = PathJoinSubstitution(
            [FindPackageShare("perception_setup"), "config", "front_camera_params.yaml"]
        )
        blackfly_config = PathJoinSubstitution(
            [FindPackageShare("perception_setup"), "config", "blackfly_s_params.yaml"]
        )
        calib_url = (
            f"file://{os.path.join(perception_share, 'config', 'front_camera_calib_downscale.yaml')}"
        )

        driver_node = ComposableNode(
            package="spinnaker_camera_driver",
            plugin="spinnaker_camera_driver::CameraDriver",
            name="front_camera",
            namespace=namespace,
            parameters=[
                camera_params,
                {
                    "parameter_file":  blackfly_config,
                    "serial_number":   cam["serial_number"],
                    "camerainfo_url":  calib_url,
                },
            ],
            remappings=[
                ("~/control",              "/exposure_control/control"),
                ("/flir_camera/image_raw", cam["image_topic"]),
                ("/flir_camera/camera_info", cam["camera_info_topic"]),
            ],
            extra_arguments=[{"use_intra_process_comms": False}],
        )
        nodes.insert(0, driver_node)   # driver goes first in the container
```

`PathJoinSubstitution` + `FindPackageShare` are used here because these values are passed directly into the node (not read in Python) and need to stay as lazy substitutions. The `calib_url` uses a regular `os.path.join` because we need the string value immediately to prepend `file://`.

### Step 6 — Create the container and return

```python
    container = ComposableNodeContainer(
        name="front_camera_aruco_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",   # multi-threaded container
        composable_node_descriptions=nodes,
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
    )

    return [container]
```

`component_container_mt` is the multi-threaded variant. All nodes in the list run in the same OS process.

### Step 7 — Declare arguments and wire up

```python
def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()    # adds drone + namespace args
        + [
            DeclareLaunchArgument(
                "scenario",
                default_value="sim",
                description="Scenario to load: sim, real_world",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
```

`generate_launch_description()` is the entry point ROS 2 calls. It declares arguments and then defers execution to `launch_setup` via `OpaqueFunction`.

---

## How to Add a New Composable Node

Example: adding an image filter between the camera driver and the ArUco detector.

### Step 1 — Add to scenario YAML

```yaml
  image_filter:
    enable: true
    erosion_size: 3
    dilation_size: 3
```

Do this in both `sim.yaml` and `real_world.yaml` (with different values if needed). In sim you might set `enable: false`.

### Step 2 — Read it in the launch file

In `launch_setup()`, after loading `cfg`:

```python
filter_cfg = cfg.get("image_filter", {"enable": False})
```

### Step 3 — Build the node and insert it into the list

```python
nodes = []

if cfg["use_camera_driver"]:
    nodes.append(driver_node)

if filter_cfg["enable"]:
    filter_params = os.path.join(perception_share, "config", "image_filtering_params.yaml")
    filter_node = ComposableNode(
        package="image_filtering",
        plugin="vortex::image_processing::ImageFilteringNode",
        name="image_filter",
        namespace=namespace,
        parameters=[
            filter_params,
            {
                "erosion_size":  filter_cfg["erosion_size"],
                "dilation_size": filter_cfg["dilation_size"],
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    nodes.append(filter_node)

nodes.append(aruco_node)
```

Order in the list matters when using intra-process comms: data flows driver → filter → aruco, so that is the order they go in.

### When to use intra-process comms

| Situation | `use_intra_process_comms` |
|---|---|
| Node publishes image that the next node in the same container subscribes to | `True` |
| Node has no in-container subscriber (e.g. the camera driver publishing out to other processes) | `False` |
| Node is the last in the chain (publishes out of the container) | `False` |

---

## How to Add a New Scenario

No Python changes needed. Just:

1. Create `config/scenarios/<name>.yaml` with all the values from the same structure as `sim.yaml`
2. Launch with `scenario:=<name>`

Example — `config/scenarios/pool_test.yaml`:
```yaml
scenario:
  name: pool_test
  use_camera_driver: true

  aruco_board:
    marker_size: 0.100      # smaller board used in pool
    xDist: 0.300
    yDist: 0.600
    ids: [28, 7, 96, 19]

  front_camera:
    image_topic: "/{ns}/front_camera/image_raw"
    camera_info_topic: "/{ns}/front_camera/camera_info"
    camera_frame: "/{ns}/front_camera_link"
    serial_number: "23494259"

  down_camera:
    image_topic: "/{ns}/down_camera/image_raw"
    camera_info_topic: "/{ns}/down_camera/camera_info"
    camera_frame: "/{ns}/down_camera_link"
    serial_number: "23494258"

  sonar:
    launch_driver: false
```

Then:
```bash
./subsea_docking.sh pool_test
# or
ros2 launch subsea_docking_setup front_camera_aruco.launch.py scenario:=pool_test
```

---

## How to Add New Parameters

Example: expose `visualize` and `dictionary` for the ArUco detector.

**In both scenario YAMLs**, add under `aruco_board`:
```yaml
  aruco_board:
    marker_size: 0.150
    xDist: 0.430
    yDist: 0.830
    ids: [28, 7, 96, 19]
    dictionary: "DICT_ARUCO_ORIGINAL"
    visualize: true
```

**In the launch file**, reference the new keys in the parameters dict:
```python
board = cfg["aruco_board"]

aruco_node = ComposableNode(
    ...
    parameters=[
        aruco_base_params,
        {
            "aruco.marker_size":  board["marker_size"],
            "board.xDist":        board["xDist"],
            "board.yDist":        board["yDist"],
            "board.ids":          board["ids"],
            "aruco.dictionary":   board["dictionary"],   # new
            "visualize":          board["visualize"],    # new
            "subs.image_topic":       cam["image_topic"],
            "subs.camera_info_topic": cam["camera_info_topic"],
            "camera_frame":           cam["camera_frame"],
        },
    ],
```

The parameter key (`"aruco.dictionary"`) must match what the node's code declares. The value comes from the scenario YAML, so it can differ per scenario.

---

## How to Create a New FSM Setup Package

When you add a new FSM (e.g. `object_retrieval`), follow these steps.

### 1 — Create the package structure

```
mission/tacc/object_retrieval/
├── object_retrieval_fsm/        ← C++ package with the FSM node
│   ├── config/
│   │   └── <fsm_specific_configs>.yaml
│   └── ...
└── object_retrieval_setup/      ← launch + config + scripts
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── scenarios/
    │       ├── sim.yaml
    │       └── real_world.yaml
    ├── launch/
    │   ├── object_retrieval_fsm.launch.py
    │   ├── waypoint_landmark.launch.py
    │   └── <perception>.launch.py
    └── scripts/
        └── object_retrieval.sh
```

### 2 — CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(object_retrieval_setup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/object_retrieval.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### 3 — package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>object_retrieval_setup</name>
  <version>0.0.0</version>
  <description>Launch package for the object retrieval mission</description>
  <maintainer email="todo@todo.com">todo</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>ros2launch</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 4 — Scenario YAMLs

Write `sim.yaml` and `real_world.yaml` with only the parameters your FSM actually needs. Not every FSM needs front camera, sonar, etc.

### 5 — Launch files

Copy the pattern from `subsea_docking_setup`. The only things that change per FSM:
- The package name in `load_scenario("object_retrieval_setup", namespace)`
- Which nodes you build (what perception components you need)
- The FSM node name, executable, and its config paths

### 6 — Bash script

Copy from an existing script, update the session name and the list of launch commands.

---

## Reference — Parameter Sources

When a node receives parameters, they are merged in order. Later entries override earlier ones.

```python
parameters=[
    "/path/to/base_defaults.yaml",    # 1. base defaults (from perception_setup)
    "/path/to/robot_config.yaml",     # 2. robot-level overrides
    {                                  # 3. scenario-specific overrides (highest priority)
        "some.param": value_from_cfg,
    },
],
```

Use this layering deliberately: keep stable defaults in the YAML files and only put scenario-varying values in the inline dict.

---

## Quick Reference

| Task | Where to change |
|---|---|
| Switch scenario | `./script.sh <scenario>` or `scenario:=<name>` on the CLI |
| Change board size | `config/scenarios/<name>.yaml` → `aruco_board` |
| Change camera serial | `config/scenarios/real_world.yaml` → `*_camera.serial_number` |
| Change topics | `config/scenarios/<name>.yaml` → `*_camera.image_topic` |
| Add a new environment | New `config/scenarios/<name>.yaml`, no Python changes |
| Add a new ROS parameter | Add to scenario YAML + read `cfg[...][...]` in launch file |
| Add a composable node | Append `ComposableNode(...)` to `nodes` list in launch file |
| Add a new launch component | New `launch/<name>.launch.py` + new pane in bash script |
| Add a new FSM | New `*_fsm/` + `*_setup/` package, follow the pattern above |
