# vortex_bt_nodes

BehaviorTree.CPP v4 nodes for the missions. The trees and the runner are in
`robosub_mission`.

The nodes are not written yet. This README lists what is needed and gives
hints. A version of every node here has been run through gate, slalom and
torpedo in the simulator, so the design works.

## What is already here

| File | What it gives you |
|---|---|
| `common/nav_action.hpp` | `NavAction`: base class for every node that moves the vehicle. You only write `make_goal()` |
| `common/types.hpp` | `Pose` and friends, and how a pose is written in XML: `"x;y;z;yaw_deg"` |
| `map/landmark_cache.hpp` | `LandmarkCache`: the latest map and TF lookups, shared by the map nodes |
| `<area>/register.cpp` | Where each node is registered |
| `test/bt_test_utils.hpp` | Fake `waypoint_manager` and a fixture for testing one node |

Read `nav_action.hpp` and `test/test_nav_action.cpp` first. The test has a
complete small node.

## Adding a node

1. Header in `include/vortex_bt_nodes/<area>/`, source in `src/<area>/`.
2. Register it in `src/<area>/register.cpp`.
3. Add a test in `test/test_<node>.cpp`.

A node does one thing. Retries, timeouts and fallbacks go in the XML. Log
with spdlog.

## Nodes to write

### motion

| Node | What it should do |
|---|---|
| `SetDepth` | Go to depth `z`, keep x, y and heading |
| `Surface` | Same as `SetDepth` with a shallow default |
| `MoveRelative` | Move by an offset from where the vehicle is now, in the body frame or along the odom axes |
| `Search` | Turn on the spot in steps, pausing at each heading. Returns FAILURE when the sweep is done, so the tree stops it when the target is found |

Hints:
- Look at the `WaypointMode` message. There is a mode for depth only.
- The `WaypointManager` action has a `frame` field for relative goals.
- For `Search`, a goal can hold several waypoints.

### map

| Node | What it should do |
|---|---|
| `LandmarkKnown` | SUCCESS if a landmark of this type and subtype is in the map |
| `LandmarkConfirmed` | SUCCESS if it has been observed and its position is certain enough |
| `GoToFrame` | Go to a TF frame plus an offset given in that frame |
| `CommitTarget` | Wait until a frame stops moving, then save its pose to the blackboard |
| `GoToPose` | Go to a saved pose plus an offset, without following the map |
| `LookAtFrame` | Turn to face a frame |
| `Turn` | Turn to a heading in the map frame, or by a relative angle |

Hints:
- `LandmarkCache` already has the lookups you need: `lookup()`,
  `vehicle_in_odom()`, `compose()`, `sigma_xy()`.
- The controller works in `odom`. A frame that is fixed in `map` moves in
  `odom` whenever the map corrects drift, so `GoToFrame` has to look the
  frame up again while driving and send a new goal when it has moved.
  `NavAction::update_goal()` is made for this.
- Close to an object the detections get worse. Decide when `GoToFrame`
  should stop updating its goal.
- An offset is in the frame's own axes. Check which way the frame's X points
  before choosing the sign. Getting this wrong sent the vehicle through the
  torpedo board in testing.
- One long move can saturate the thrusters and flip the vehicle. Split long
  moves into waypoints a couple of metres apart.
- `CommitTarget` and `GoToPose` exist because the camera cannot see the
  slalom pipes while passing between them. Think about what should happen
  to the target in that moment.

### mission

| Node | What it should do |
|---|---|
| `LoadMissionConfig` | Read a yaml file and write every key to the blackboard |
| `WaitForStart` | RUNNING until the killswitch is off and the vehicle is in autonomous mode |
| `Log` | Log a message and return SUCCESS |

Hints:
- BehaviorTree.CPP checks port types. Writing the values as text lets every
  port read them with its own type.
- The operation mode is only published when it changes, so ask for it with
  the `get_operation_mode` service.

### actuators

| Node | What it should do |
|---|---|
| `FireTorpedo` | Fire the left or right torpedo, then wait a moment |
| `DropMarker` | Drop marker 0 or 1, then wait a moment |
| `SetGripper` | Send roll and pinch to the gripper action |

The torpedo and dropper have no interface on the drone yet. Publish on a
placeholder topic for now.

## Tolerances

Every `NavAction` has the ports `position_tolerance`,
`orientation_tolerance_deg` and `hold_s`.

- If only the position tolerance is set, the heading still has to be within
  about 6 degrees. Set both on moves that do not need a precise heading.
- The vehicle approaches a goal slowly at the end. A tolerance of 0.1 m
  takes much longer to reach than 0.4 m, so only be strict where it matters.

## Interfaces

All under `/nautilus`: the `waypoint_manager` action,
`landmark_server/landmarks`, the TF frames from `landmark_server`,
`get_operation_mode`, and TF `odom -> base_link`.
