# robosub_mission

Behavior tree for the RoboSub course (BehaviorTree.CPP v4): the tree runner
and the trees. The nodes are in [`vortex_bt_nodes`](../../vortex_bt_nodes),
including who writes which node and in what order. Launch and config live in
`perception_setup`, as for the TACC tasks.

```
robosub_mission/
├── src/main.cpp                 Registers the vortex_bt_nodes nodes, loads
│                                trees/root.xml and ticks it until it finishes
└── trees/
    ├── root.xml                 Main (safety, then Mission; SafeStop on failure),
    │                            Mission (task order), SafeStop
    └── <task>.xml               One subtree per task (gate.xml, slalom.xml, ...)

perception_setup/launch/mission/robosub/robosub_mission.launch.py
perception_setup/config/mission/robosub/mission.yaml
```

## Adding a task

1. `trees/<task>.xml`: a `<root BTCPP_format="4">` with one `BehaviorTree`
   whose `ID` is the task name.
2. In `trees/root.xml`: `<include path="<task>.xml"/>` at the top and
   `<SubTree ID="<Task>" _autoremap="true"/>` in `Mission`.
3. Its values in `perception_setup/config/mission/robosub/mission.yaml`.

Nodes do one thing and return SUCCESS or FAILURE; retries, timeouts,
searches and fallbacks are written in the XML.

## Run

```bash
ros2 launch perception_setup robosub_mission.launch.py
```

Parameters: `tree_file` (default: the installed `trees/root.xml`),
`tick_rate_hz` (10), `mission_config` (the yaml above). The runner spins the
ROS node between ticks, so action and topic callbacks of the nodes run
there. Logging with spdlog.

## Status

No tasks yet. The tree runs `Main` → `Mission` and finishes with SUCCESS.
Requires `ros-humble-behaviortree-cpp` (v4).
