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

Code shared by several areas goes in `include/robosub_mission/common/` and
`src/common/`; agree on it before two areas write their own.

## Shared base class for motion nodes (M1, first)

Every node that moves the vehicle does the same thing: send a
`WaypointManager` goal, wait for the result, and cancel the goal if the tree
halts the node. That is written once, in a base class in `common/`, by M1
before the motion nodes:

```cpp
// include/robosub_mission/common/nav_action.hpp
class NavAction : public BT::StatefulActionNode {
   protected:
    // The one thing each node writes: the waypoint(s) for this tick's goal,
    // from its input ports. nullopt -> FAILURE (e.g. a missing port).
    virtual std::optional<vortex_msgs::action::WaypointManager::Goal> make_goal() = 0;

   private:
    BT::NodeStatus onStart() override;    // make_goal(), send it       -> RUNNING
    BT::NodeStatus onRunning() override;  // result: SUCCEEDED/other    -> SUCCESS/FAILURE
    void onHalted() override;             // cancel the goal (Timeout, ReactiveSequence)
};
```

Shared ports on the base: `position_tolerance`, `orientation_tolerance_deg`,
`hold_s`. Depth, yaw and positions are not controlled here: waypoint_manager
and the reference filter already do that (`WaypointMode::ONLY_Z` for depth,
`POSITION_AND_YAW`, `FULL_POSE`, ..., tolerances and hold time per waypoint).

| Node | `make_goal()` fills in |
|---|---|
| `SetDepth` | One waypoint, mode `ONLY_Z`, `z` from the port |
| `Surface` | As `SetDepth`, with a small `z` (for example 0.2) |
| `Turn` | Mode `ONLY_ORIENTATION`, yaw = current + `relative_deg` |
| `GoTo` | The `pose` port, with its `mode` |
| `MoveRelative` | Offset, `frame` `BODY_RELATIVE` or `WORLD_RELATIVE` |
| `GoToCourse`, `MoveCourse` | Course frame point converted to odom |
| `HoldPosition` | The current pose, `hold_s` |

The other areas build on it too: `Search` (M2) sends its pattern as goals,
`ApproachLandmark` and `LookAtLandmark` (M3) send the goals `landmark_targets`
computes. Until `NavAction` exists, they can be written against its interface
above.

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
