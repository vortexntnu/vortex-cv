# robosub_mission

Behavior tree for the RoboSub course (BehaviorTree.CPP v4): the tree runner,
the custom nodes, and the trees. Launch and config live in `perception_setup`,
as for the TACC tasks.

```
mission/robosub/robosub_mission/
├── behavior_trees/
│   ├── main.xml          Main (safety, then Mission; SafeStop on failure),
│   │                     Mission (task order), SafeStop
│   └── tasks/            One subtree per task, <task>.xml
├── include/robosub_mission/
│   └── register_nodes.hpp
└── src/
    ├── main.cpp          Loads the tree and ticks it until it finishes
    └── register_nodes.cpp  Registers the custom nodes

perception_setup/launch/mission/robosub/robosub_mission.launch.py
perception_setup/config/mission/robosub/mission.yaml
```

```bash
ros2 launch perception_setup robosub_mission.launch.py
```

Parameters: `tree_file` (default: the installed `behavior_trees/main.xml`),
`tick_rate_hz` (10), `mission_config` (the yaml above).

## Status

Structure only: the tree runs `Main` → `Mission` with no tasks and finishes.
Tasks and custom nodes (pose/map checks, `ApproachLandmark`, `TaskSlot`, ...)
are added one at a time, following the RoboSub course tree plan.

Requires `ros-humble-behaviortree-cpp` (v4).
