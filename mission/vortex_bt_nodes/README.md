# vortex_bt_nodes

BehaviorTree.CPP v4 nodes for Vortex missions, set up like
`vortex_yasmin_utils`. The trees and the runner are in `robosub_mission`.

```
include/vortex_bt_nodes/
  common/nav_action.hpp    base for nodes that move the vehicle
  common/types.hpp         Pose types and how they are written in XML
  map/landmark_map.hpp   latest map and TF lookups, shared by the map nodes
  <area>/                  one header per node + register.hpp
src/<area>/                one source per node + register.cpp
test/bt_test_utils.hpp     test fixture + fake waypoint_manager
test/test_<node>.cpp       one test per node
```

Areas: `motion`, `map`, `mission`, `actuators`.

## Rules

- A node only depends on `common/` and `LandmarkMap`. If your node reads a
  key another person's node writes, set that key by hand in your test.
- A node is a header, a source file, a test and one line in
  `src/<area>/register.cpp`. Use snake_case file names and namespace
  `vortex_bt_nodes::<area>`.
- A node does one thing. Retries, timeouts and fallbacks go in the XML.
- Log with spdlog: `spdlog::warn("[{}] ...", name());`
- A node is done when its test passes and it is registered. The test covers
  success and every failure case listed below.

## Who makes what

Write your nodes in the order listed, then your target frames if you have
any, then your task tree (see the `robosub_mission` README).

| Person | Task | Nodes | Target frames in `landmark_server` |
|---|---|---|---|
| Amélie | Gate | Log, SetDepth, Surface, MoveRelative, SetGripper | |
| André | Slalom | LandmarkKnown, Search, CommitTarget, GoToPose | Slalom gaps |
| Johannes | Torpedo | FireTorpedo, GoToFrame | Torpedo openings |
| Karol | Bins | LoadMissionConfig, WaitForStart, DropMarker | |
| Ashish | Octagon | LandmarkConfirmed, LookAtFrame, Turn | |

Kind: sync = `SyncActionNode`, stateful = `StatefulActionNode` (never
blocks), nav = derives from `NavAction`, condition = `ConditionNode`. `→`
marks an output port.

### Amélie

| Node | Area | Kind | Ports | Does |
|---|---|---|---|---|
| Log | mission | sync | `message`, `level` | Logs with spdlog (info, warn or error) and returns SUCCESS |
| SetDepth | motion | nav | `z` | Goes to depth `z`, keeps x, y and heading |
| Surface | motion | nav | `z` (0.2) | As SetDepth, with a shallow default |
| MoveRelative | motion | nav | `offset`, `frame` (BODY or WORLD), `mode` | Moves by an offset from where the vehicle is when the node starts |
| SetGripper | actuators | stateful | `roll`, `pinch`, `mode`, `action` | Sends a gripper goal. FAILURE if it is rejected, cancels when halted |

Hints:
- Look at the `WaypointMode` message. There is a mode for depth only.
- The `WaypointManager` action has a `frame` field for relative goals.

### André

| Node | Area | Kind | Ports | Does |
|---|---|---|---|---|
| LandmarkKnown | map | condition | `id` or `type`/`subtype` | SUCCESS if the landmark is in the map |
| Search | motion | nav | `pattern` (ROTATE_STEPS or SCAN_ARC), `step_deg`, `arc_deg`, `pause_s` | Turns on the spot, pausing at each heading. FAILURE when the sweep is done, so the tree stops it when the target is found |
| CommitTarget | map | stateful | `frame`, `stable_m`, `stable_s` → `pose` | RUNNING while the frame is missing or still moving. Then writes its pose in odom and returns SUCCESS |
| GoToPose | map | nav | `pose`, `offset`, `mode` | Goes to a pose from the blackboard plus an offset in that pose's frame. Never updates the goal |

Hints:
- For `Search`, one goal can hold several waypoints.
- `CommitTarget` and `GoToPose` are for moments like the slalom, where the
  camera cannot see the pipes while passing between them. Think about what
  should happen to the target then.
- The slalom gap frames are yours too, see the `landmark_server` README.

### Johannes

| Node | Area | Kind | Ports | Does |
|---|---|---|---|---|
| FireTorpedo | actuators | stateful | `side` (left, right), `topic`, `settle_s` | Publishes `std_msgs/Int8` (0 left, 1 right), then waits `settle_s`. FAILURE on an unknown side |
| GoToFrame | map | nav | `frame`, `offset`, `mode`, `resend_m`, `freeze_within_m`, `max_step_m` | Goes to a TF frame plus an offset in that frame, and follows the frame while it moves. FAILURE if the frame is not in TF at the start |

Hints:
- `GoToFrame` is the node every tree uses most, so do it early.
- The controller works in `odom`. A frame that is fixed in `map` moves in
  `odom` whenever the map corrects drift. Look the frame up again while
  driving and send a new goal when it has moved more than `resend_m`.
  `NavAction::update_goal()` is made for this.
- Close to an object the detections get worse. Stop updating the goal within
  `freeze_within_m`.
- An offset is in the frame's own axes. Check which way the frame's X points
  before choosing the sign. A wrong sign puts the goal on the other side of
  the object.
- One long move can saturate the thrusters and flip the vehicle. Split moves
  longer than `max_step_m` into several waypoints.
- The torpedo opening frames are yours too, see the `landmark_server`
  README.

### Karol

| Node | Area | Kind | Ports | Does |
|---|---|---|---|---|
| LoadMissionConfig | mission | sync | `path` | Writes every key in the yaml to the blackboard. Nested keys become `a.b`. FAILURE if the file can't be read |
| WaitForStart | mission | stateful | `service` | RUNNING until the killswitch is off and the vehicle is in autonomous mode |
| DropMarker | actuators | stateful | `index` (0, 1), `topic`, `settle_s` | Publishes `std_msgs/Int8`, then waits `settle_s`. FAILURE on another index |

Hints:
- BehaviorTree.CPP checks port types. Writing the values as text lets every
  port read them with its own type.
- The operation mode is only published when it changes, so ask for it with
  the `get_operation_mode` service.

### Ashish

| Node | Area | Kind | Ports | Does |
|---|---|---|---|---|
| LandmarkConfirmed | map | condition | `id` or `type`/`subtype`, `max_sigma_xy` | SUCCESS if the landmark has been observed and its horizontal std is below `max_sigma_xy` |
| LookAtFrame | map | nav | `frame` | Turns on the spot to face a TF frame. FAILURE if it is not in TF |
| Turn | map | nav | `yaw_deg` or `relative_deg` | Turns to a heading in the map frame, or by an angle from the current heading |

Hints:
- `LandmarkMap` has what you need: `resolve()`, `confirmed()`,
  `lookup()`, `vehicle_in_odom()`.
- A heading in the map frame is not the same heading in `odom`.

The marker and torpedo topics are placeholders until the drone has an
interface for them.

## NavAction

Derive from `NavAction` and write `make_goal()`. Return `std::nullopt` on
bad input and the node fails. NavAction sends the goal, returns RUNNING, and
cancels the goal when the node is halted. Override `update_goal()` to
replace a running goal.

Every NavAction also has the ports `position_tolerance`,
`orientation_tolerance_deg` and `hold_s`.

- If only the position tolerance is set, the heading still has to be within
  about 6 degrees. Set both on moves that do not need a precise heading.
- The vehicle approaches a goal slowly at the end. A tolerance of 0.1 m
  takes much longer to reach than 0.4 m, so only be strict where it matters.

## Good to know

- XML formats (Pose, PoseList, type/subtype/mode names) are in the comment
  at the top of `common/types.hpp`, with the converters.
- `test/test_nav_action.cpp` shows how a node is tested with the fake
  waypoint_manager.
- A landmark is named by `id`, or by `type` + `subtype`.
  `LandmarkMap::ports()` and `resolve()` handle this for you.
- Interfaces (under `/nautilus`): action `waypoint_manager`,
  `landmark_server/landmarks` and its TF frames, `get_operation_mode`,
  TF `odom -> base_link`. The `landmark_server` README describes the map.
