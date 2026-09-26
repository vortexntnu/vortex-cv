# robosub_mission

Behavior tree for the RoboSub course (BehaviorTree.CPP v4): the tree runner,
the custom nodes, and the trees. Launch and config live in `perception_setup`,
as for the TACC tasks.

```
robosub_mission/
├── trees/
│   ├── root.xml                 Main (safety, then Mission; SafeStop on failure),
│   │                            Mission (task order), SafeStop
│   └── <task>.xml               One subtree per task (gate.xml, slalom.xml, ...)
├── include/robosub_mission/
│   ├── register_nodes.hpp
│   └── nodes/<area>/            register.hpp + one header per node
└── src/
    ├── main.cpp                 Loads trees/root.xml and ticks it until it finishes
    ├── register_nodes.cpp       Calls each area's register_nodes
    └── nodes/<area>/            register.cpp + one source per node

perception_setup/launch/mission/robosub/robosub_mission.launch.py
perception_setup/config/mission/robosub/mission.yaml
```

## Node areas

One folder per area, one owner per area, so members work in their own folder
and only touch their own `register.cpp`. Node names are from the RoboSub
course tree plan (section 8).

| Area | Owner | Nodes |
|---|---|---|
| `motion` | M1 | SetDepth, Surface, Turn, HoldPosition, GoTo, MoveRelative, GoToCourse, MoveCourse, FollowPoses, SelectGatePanel |
| `map` | M2 | PoseFeeder, MapFeeder, CourseFrameFeeder, LandmarkKnown, SelectLandmark, Search, MatchPipes, RecordLayer, AvoidSlalom |
| `approach` | M3 | ApproachLandmark, CommitEstimate, LookAtLandmark |
| `actuators` | M4 | DropMarker, FireTorpedo, SetGripper, MarkUsed |
| `mission` | M5 | VehicleHealthy, LoadMissionConfig, Wait, SetOperationMode, ResetWorld, LogError, SavePose, GoToSavedPose, StartRun, MissionClock, TaskSlot, RecordBag, ResolveRole, VerifyInside |

Code shared by several areas (for example a base class for nodes that send
goals to `waypoint_manager`, used by motion, approach and Search) goes in
`include/robosub_mission/common/` and `src/common/`; agree on it before two
areas write their own.

## Adding a node

1. `include/robosub_mission/nodes/<area>/<node>.hpp` and
   `src/nodes/<area>/<node>.cpp` (namespace `robosub_mission::nodes::<area>`).
2. One line in `src/nodes/<area>/register.cpp`:
   `factory.registerNodeType<MyNode>("MyNode", node);`
3. Rebuild. Sources under `src/nodes/` are picked up automatically; the
   CMakeLists does not change.

## Adding a task

1. `trees/<task>.xml`: a `<root BTCPP_format="4">` with one `BehaviorTree`
   whose `ID` is the task name.
2. In `trees/root.xml`: `<include path="<task>.xml"/>` at the top and
   `<SubTree ID="<Task>" _autoremap="true"/>` in `Mission`.
3. Its values in `perception_setup/config/mission/robosub/mission.yaml`.

Nodes do one thing and return SUCCESS or FAILURE; retries, timeouts, searches
and fallbacks are written in the XML.

## Run

```bash
ros2 launch perception_setup robosub_mission.launch.py
```

Parameters: `tree_file` (default: the installed `trees/root.xml`),
`tick_rate_hz` (10), `mission_config` (the yaml above). Logging with spdlog.

## Status

Structure only: no nodes and no tasks yet. The tree runs `Main` → `Mission`
and finishes with SUCCESS. Requires `ros-humble-behaviortree-cpp` (v4).
