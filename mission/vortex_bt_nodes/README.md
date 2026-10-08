# vortex_bt_nodes

BehaviorTree.CPP v4 nodes for Vortex missions, set up like
`vortex_yasmin_utils`. The trees and the runner are in `robosub_mission`.

```
include/vortex_bt_nodes/
  common/nav_action.hpp    base for nodes that move the vehicle
  common/types.hpp         blackboard types, landmark and mode names
  <area>/                  one header per node + register.hpp
src/<area>/                one source per node + register.cpp
```

Areas: `motion` (moves that need no map), `map` (landmarks and moves to
map targets), `mission`, `actuators`.

## How a mission uses them

The map is `landmark_slam`: every landmark, the gate frames, a frame per
class and `start` are TF frames. A tree names targets by frame and never
computes geometry itself; `GoToFrame` follows the frame as the map corrects
it. Every task has the same shape:

```xml
<Sequence>
  <SetDepth z="{gate_depth}"/>                                   <!-- depth -->
  <GoToFrame frame="map" offset="{gate_search}"/>                <!-- search point -->
  <ReactiveFallback>                                             <!-- search until the map has it -->
    <LandmarkConfirmed type="GATE" subtype="{gate_panel_subtype}"/>
    <Search pattern="SCAN_ARC"/>
  </ReactiveFallback>
  <GoToFrame frame="{gate_entrance_frame}"/>                     <!-- do it -->
  <GoToFrame frame="{gate_exit_frame}"/>                         <!-- leave -->
</Sequence>
```

Retries, timeouts and skipping a task are BehaviorTree.CPP's own nodes
(`Timeout`, `ForceSuccess`, `RetryUntilSuccessful`, `ReactiveFallback`,
`Sleep`). A retry loop around a condition needs an async node in it (a
`Sleep` or a move), else `RetryUntilSuccessful` loops inside one tick and the
map never updates.

## Nodes

All moves go through `waypoint_manager` (`NavAction`: RUNNING until the goal
is reached, the goal is cancelled when halted). Every move also has the ports
`position_tolerance`, `orientation_tolerance_deg` and `hold_s`. Poses are in
odom unless said otherwise; a Pose port is `"x;y;z[;yaw_deg]"`.

### map

One `LandmarkCache` (`map/landmark_cache.hpp`: the latest
`landmark_slam/landmarks` and TF) is made in `map/register.cpp` and shared
by these nodes. A landmark is named by `id`, or by `type` + `subtype`
(`ANY` allowed): then the best of the class is used (lowest σ_xy, then most
observations).

| Node | Kind | Ports | Does |
|---|---|---|---|
| LandmarkKnown | condition | `id` or `type`/`subtype` | SUCCESS if the landmark is in the map (prior map or observed) |
| LandmarkConfirmed | condition | `id` or `type`/`subtype`, `max_sigma_xy` (0.3) | SUCCESS if observed and its horizontal std relative to the vehicle is below `max_sigma_xy` |
| GetLandmarkPose | sync | `id` or `type`/`subtype` → `pose` | `PoseStamped` in odom (through landmark_slam's `map → odom`). FAILURE if unknown |
| GetApproachPose | sync | `id` or `type`/`subtype`, `offset` (landmark frame, +X out of the front), `symmetry_deg` (0) → `pose` | `PoseStamped` in odom at the offset, facing the landmark, on the symmetric side closest to the vehicle (turned toward the vehicle if the yaw is unknown) |
| GoToFrame | move | `frame`, `offset` (in the frame), `mode` (POSITION_AND_YAW), `resend_m` (0.1), `freeze_within_m` (1.0) | Goes to a TF frame: a landmark (`<class>_<id>`), a class (`torpedo_board`), a gate frame, `start`, or `map` + offset for a fixed point. Looks it up in odom every tick and sends the goal again when it moved more than `resend_m`; within `freeze_within_m` the target is fixed (close up the detections are poor) |
| Turn | move | `yaw_deg` (heading in the map frame) or `relative_deg` | Turns on the spot: coin-flip alignment, an exit heading |
| LookAtFrame | move | `frame` | Turns to face a frame, so the cameras see it |

### motion

| Node | Kind | Ports | Does |
|---|---|---|---|
| SetDepth | move | `z` | Depth only (mode ONLY_Z), keeps x, y and heading |
| Surface | move | `z` (0.2) | As SetDepth, named for the tree |
| MoveRelative | move | `offset`, `frame` (BODY or WORLD), `mode` | Moves by an offset from where the vehicle is at the start: a blind drive, backing off |
| Search | move | `pattern` (ROTATE_STEPS or SCAN_ARC), `step_deg` (45), `arc_deg` (60), `pause_s` (1.5) | Turns on the spot, holding at each heading for the detectors. The tree stops it when the target is found; FAILURE when the sweep is done without being stopped |

### mission

| Node | Kind | Ports | Does |
|---|---|---|---|
| LoadMissionConfig | sync | `path` | Every key of the yaml (mission.yaml) to the blackboard as text; ports read it with their own type. Nested keys become `a.b` |
| WaitForStart | stateful | `service` (get_operation_mode) | RUNNING until the killswitch is off and the mode is autonomous |
| StartRun | stateful | `coin_flip_deg`, `slam_node`, `timeout_s` | Publishes `mission/wipe` and gives landmark_slam the coin flip (`start_yaw_offset_deg`) |
| Log | sync | `message`, `level` (info, warn, error) | spdlog, SUCCESS |

### actuators

The torpedo and dropper topics are placeholders until the drone has an
interface for them.

| Node | Kind | Ports | Does |
|---|---|---|---|
| FireTorpedo | stateful | `side` (left, right), `topic`, `settle_s` (1.0) | `std_msgs/Int8` 0/1, then waits. FAILURE on an unknown side |
| DropMarker | stateful | `index` (0, 1), `topic`, `settle_s` (2.0) | `std_msgs/Int8`, then waits. FAILURE on another index |
| SetGripper | stateful | `roll`, `pinch`, `mode` (ROLL_AND_PINCH, ONLY_ROLL, ONLY_PINCH), `action` | GripperReferenceFilterWaypoint; cancelled when halted |

## Adding a node

- A header and a source in `<area>/`, one line in `src/<area>/register.cpp`,
  snake_case file names, namespace `vortex_bt_nodes::<area>`.
- A node does one thing. Retries, timeouts and fallbacks go in the XML.
- A move derives from `NavAction` and writes `make_goal()` (return
  `std::nullopt` on bad input and the node fails). To replace the running
  goal (a target that moves), override `update_goal()` (see `GoToFrame`).
- A target that needs geometry (an opening, an approach point) is a frame in
  landmark_slam, not code in a node.
- Log with spdlog: `spdlog::warn("[{}] ...", name());`

## Good to know

- Interfaces (under `/nautilus`): `landmark_slam/landmarks` and its TF
  frames, action `waypoint_manager`, `mission/wipe`, `get_operation_mode`,
  TF `odom → base_link` (in the simulator from
  `robosub_dummy_publisher`'s `sim_odom_relay_node`).
- BehaviorTree.CPP 4.10 checks port types: a blackboard entry written as
  `double` can't be read by an `int` port. LoadMissionConfig writes text,
  which every port type can read.
- The runner publishes the tree to Groot2 (port 1667) for a live view.
