# vortex_bt_nodes

BehaviorTree.CPP v4 nodes for Vortex missions, built as a library the same
way `vortex_yasmin_utils` is for the FSMs. `robosub_mission` is the runner
and the trees; the nodes live here.

```
vortex_bt_nodes/
├── include/vortex_bt_nodes/
│   ├── register_nodes.hpp          Registers every area
│   ├── common/
│   │   ├── nav_action.hpp          Base for nodes that move the vehicle
│   │   └── types.hpp               Blackboard keys and types, landmark names
│   └── <area>/                     register.hpp + one header per node
├── src/
│   ├── register_nodes.cpp
│   ├── common/
│   └── <area>/                     register.cpp + one source per node
└── test/
    ├── bt_test_utils.hpp           Fake waypoint_manager + test fixture
    └── test_<node>.cpp             One test per node
```

Areas: `motion` (moving through waypoint_manager), `map` (reading
landmark_server, searching), `approach` (moving relative to landmarks),
`actuators` (markers, torpedoes, gripper), `mission` (flow, time, safety).

## Ground rules

- **Nobody waits for anybody.** Every node only depends on `common/`, which
  is done: `NavAction` and the types in `types.hpp`. Blackboard values that
  another person's node writes are set by hand in your test.
- **Blackboard keys and types come from `types.hpp`.** If you need a new
  key that someone else reads, add it there first, in its own small commit.
- **One node = header + source + test.** `include/vortex_bt_nodes/<area>/<node>.hpp`,
  `src/<area>/<node>.cpp`, `test/test_<node>.cpp` (snake_case file names,
  namespace `vortex_bt_nodes::<area>`), plus one line in
  `src/<area>/register.cpp`. CMake picks up new files by itself.
- **A node does one thing** and returns SUCCESS or FAILURE. Retries,
  timeouts, searches and fallbacks go in the XML.
- **Logging with spdlog**, prefixed with the node name: `spdlog::warn("[{}] ...", name());`
- **Done** = the test passes and the node is registered.

## Who makes what

Everyone writes nodes in several areas, and everyone also owns a task
tree in `robosub_mission/trees/`. The task specs (goal, what the tree must
do, when it is done) are in the
[robosub_mission README](../robosub/robosub_mission/README.md#tasks).

| Person | Task | Nodes, in order |
|---|---|---|
| Johannes | Torpedo | LogError, TaskSlot, SelectGatePanel, CommitEstimate, ApproachLandmark |
| André | Slalom | VehicleHealthy, Search, MatchPipes, RecordLayer, AvoidSlalom |
| Ashish | Octagon | PoseFeeder, MapFeeder, CourseFrameFeeder, SavePose, GoToSavedPose, VerifyInside, GoToCourse, MoveCourse, LookAtLandmark, RecordBag |
| Karol | Bins | Wait, SetOperationMode, LoadMissionConfig, MissionClock, ResetWorld, StartRun, LandmarkKnown, SelectLandmark, ResolveRole, FollowPoses |
| Amélie | Gate | SetDepth, Surface, GoTo, MoveRelative, Turn, HoldPosition, SetGripper, DropMarker, FireTorpedo, MarkUsed |

**Order:** nodes first, top to bottom, then the task tree. A tree uses other
people's nodes, so write it against the names and ports below; it runs once
those nodes are merged.

**Each node spec below** says what goes in, what comes out, when it returns
SUCCESS or FAILURE, and what its test must show. Ports marked `{key}` are
blackboard keys from `types.hpp`. "Test:" lists the cases
`test/test_<node>.cpp` must cover; more is welcome.

Kinds: *sync* = `BT::SyncActionNode` (answers in one tick), *stateful* =
`BT::StatefulActionNode` (RUNNING over several ticks, never blocks),
*condition* = `BT::ConditionNode`, *decorator* = `BT::DecoratorNode`,
*NavAction* = derives from `NavAction` (only `make_goal()`).

### Johannes · task: Torpedo

1. **LogError** (mission, sync)
   - In: `message` (string).
   - Logs `message` with `spdlog::error`, prefixed with the node name. Returns SUCCESS, so it can sit first in a fallback branch.
   - Test: returns SUCCESS; a missing `message` still returns SUCCESS and logs an empty line.

2. **TaskSlot** (mission, decorator)
   - In: `task` (string, for the log), `clock` (MissionClock, {mission_clock}), `budget_s`, `reserve_s`.
   - On start: if `clock.remaining_s(now) < reserve_s`, skip the child and log "skipped".
   - While running: tick the child; halt it once it has run `budget_s`.
   - Always returns SUCCESS when done; logs the outcome (SUCCESS, FAILURE, timeout, skipped) and the time used.
   - Test: child succeeds → SUCCESS; child fails → SUCCESS; child never finishes → halted after `budget_s`, SUCCESS; too little time left → child never ticked. Use a clock with a start in the past to fake elapsed time.

3. **SelectGatePanel** (motion, stateful)
   - In: `map` ({map}), `gate_id` (int), `preferred_role` ("survey_repair" / "search_rescue").
   - Out: `panel_id` (int), `role` ({role}), `gate_side` ({gate_side}: "left"/"right").
   - Looks for the gate's two role panels (GATE subtypes GATE_SURVEY_REPAIR / GATE_SEARCH_RESCUE) near `gate_id` in the map. Picks the panel with our role; its side of the gate centre (seen from the approach) is `gate_side`.
   - Only one panel seen: take it, set `role` to its role. None seen yet: RUNNING (the XML puts a Timeout around it).
   - Test: both panels → ours, correct side for each role; only the other panel → that one and its role; no panels → RUNNING; unknown `gate_id` → FAILURE.

4. **CommitEstimate** (approach, stateful)
   - In: `map`, `id`, `samples` (int), `position_std_m`, `yaw_std_deg`.
   - Each tick, store the map pose of `id` if the map message is new. SUCCESS once the last `samples` poses have a standard deviation below `position_std_m` (x, y, z) and `yaw_std_deg`. RUNNING before that. FAILURE if `id` is not in the map.
   - Test: steady poses → SUCCESS after `samples` ticks; noisy poses → RUNNING; noisy then steady → SUCCESS; unknown id → FAILURE.

5. **ApproachLandmark** (approach, NavAction, uses `landmark_targets`)
   - In: `map`, `pose`, `id` (or `type`/`subtype`: nearest confirmed match), `frame` (LANDMARK / LANDMARK_ODOM_AXES), `x`, `y`, `z`, `yaw_deg` (offset in that frame), `mode`, `tool_frame` (optional), `freeze` (bool), `dead_reckoning_m`, `track_loss_timeout_s`, plus the NavAction ports.
   - Computes the goal pose from the landmark's map pose and the offset with `landmark_targets` (with `tool_frame`, the tool is put at the target, not the vehicle).
   - While running: re-send the goal when the landmark moves more than a few cm in the map, unless `freeze` or the vehicle is within `dead_reckoning_m`. FAILURE when the landmark has not been updated for `track_loss_timeout_s`.
   - Test: goal = landmark pose + offset for both frames; `type`/`subtype` picks the nearest; landmark moves → new goal; `freeze` → no new goal; stale landmark → FAILURE; unknown id → FAILURE.
   - NavAction sends one goal per start today. Re-sending needs a small hook in `NavAction` (for example a virtual `should_resend()` checked in `onRunning`); add it in `common/` in its own commit with a test, so the other NavAction nodes are unchanged.

Then: `trees/torpedo.xml`.

### André · task: Slalom

1. **VehicleHealthy** (mission, condition)
   - In: `pose` ({pose}), `max_depth_m`.
   - FAILURE (and `spdlog::error` once) when `pose.position.z > max_depth_m`; otherwise SUCCESS.
   - Test: shallow → SUCCESS; too deep → FAILURE; missing pose → FAILURE.

2. **Search** (map, NavAction)
   - In: `pose`, `pattern` (ROTATE_STEPS, SCAN_ARC, LAWNMOWER, EXPANDING_SQUARE) and its numbers: `step_deg`, `arc_deg`, `rounds`, `legs`, `leg_m`, `spacing_m`, `pause_s`.
   - Builds the whole pattern as waypoints around the pose at start (yaw steps for ROTATE_STEPS / SCAN_ARC, a lawnmower or square in the horizontal plane), with `hold_time_sec = pause_s` on each. SUCCESS when the pattern is done.
   - It does not look at the map: the XML stops it (`ReactiveFallback` with `LandmarkKnown` first).
   - Test: waypoint count and positions for each pattern; unknown pattern → FAILURE; halt cancels the goal.

3. **MatchPipes** (map, sync, uses `landmark_targets::match_pipes`)
   - In: `map`, `pose`, `gate_side`, `exclude` (IdList, {passed_red_ids}), `offset` (bidirectional), `min_forward_m`, `collinearity_m`, `min_separation_m`, `inward_deg`.
   - Out: `gap_pose` (Pose, {gap_pose}), `red_id` (int).
   - Splits the confirmed SLALOM_PIPE tracks into red and white, calls the library, writes the gap pose (x, y at the gap, z = current slalom depth, yaw = layer heading). FAILURE when no layer can be matched.
   - Test: full layer → gap on the `gate_side` side of red; red + one white → mirrored gap; excluded red → next layer; no pipes → FAILURE. See `~/slalom_guide.md`.

4. **RecordLayer** (map, sync)
   - In: `pose`, `red_id`, `layers` (bidirectional, own type), `passed` (IdList, bidirectional, {passed_red_ids}).
   - Appends `red_id` to `passed` and the layer (red id, pose when passed) to `layers`. SUCCESS.
   - Test: ids accumulate over three calls; missing `red_id` → FAILURE.

5. **AvoidSlalom** (map, sync, uses `landmark_targets::avoid_slalom_waypoints`)
   - In: `map`, `course_frame`, `layers` ({slalom_layers}), `side` ("left"/"right"), `clearance_m`.
   - Out: `path` (PoseList, {avoid_path}).
   - Poses that go around the whole slalom on `side` with `clearance_m` to the outermost pipe, for Return Home. FAILURE with no pipes and no layers.
   - Test: path stays `clearance_m` outside every pipe; both sides; empty map → FAILURE.

Then: `trees/slalom.xml`.

### Ashish · task: Octagon

1. **PoseFeeder** (map, sync)
   - In: `topic` (default "pose"), `max_age_s`. Out: `pose` ({pose}).
   - Subscribes (in the constructor) to `geometry_msgs/PoseWithCovarianceStamped` on `topic` with `rclcpp::SensorDataQoS()` (the ESKF publishes best effort). Each tick writes the latest pose. FAILURE if none received or older than `max_age_s` (node clock).
   - Test: publish a pose, spin, tick → SUCCESS and the pose on the blackboard; nothing published → FAILURE; old stamp → FAILURE.

2. **MapFeeder** (map, sync)
   - In: `topic` (default "landmark_server/object_map"), `max_age_s`. Out: `map` ({map}).
   - As PoseFeeder for `vortex_msgs/LandmarkTrackArray` (reliable QoS, like the publisher). An empty map is fine; only age matters.
   - Test: as PoseFeeder.

3. **CourseFrameFeeder** (map, sync)
   - In: `frame` (default "nautilus/course"), `odom_frame`. Out: `course_frame` ({course_frame}, the course origin in odom).
   - TF lookup `odom_frame → frame` with a `tf2_ros::Buffer` owned by the node. FAILURE until the transform exists.
   - Test: publish a static transform, tick → the pose; no transform → FAILURE.

4. **SavePose** (mission, sync)
   - In: `pose`. Out: `saved`.
   - Copies `pose` to `saved` (in the tree: {start_pose}). FAILURE if `pose` is missing.
   - Test: copies; missing → FAILURE.

5. **GoToSavedPose** (mission, NavAction)
   - In: `saved`, `mode` (default POSITION_AND_YAW), plus the NavAction ports.
   - One waypoint at `saved`, frame WORLD. FAILURE if `saved` is missing or the mode unknown.
   - Test: goal pose and mode; missing → no goal sent.

6. **VerifyInside** (mission, condition)
   - In: `map`, `pose`, `type`, `subtype`, `radius_m`, `margin_m`.
   - SUCCESS if the horizontal distance from the vehicle to the nearest matching confirmed landmark is less than `radius_m − margin_m`.
   - Test: inside, on the edge, outside; no such landmark → FAILURE; unknown type name → FAILURE.

7. **GoToCourse** (motion, NavAction)
   - In: `course_frame`, `x`, `y`, `z` (depth in odom), `yaw_deg` (relative to the course x axis), `mode`, plus the NavAction ports.
   - Transforms (x, y, yaw) from the course frame to odom; z is used as is. One waypoint, frame WORLD.
   - Test: identity course frame → same point; course frame rotated 90° and shifted → correct odom point and yaw; missing course frame → FAILURE.

8. **MoveCourse** (motion, NavAction)
   - In: `course_frame`, `pose`, `dx`, `dy`, `z` (optional; default the current depth), plus the NavAction ports.
   - Goal = current pose + (dx, dy) along the course axes, yaw along the course x axis.
   - Test: rotated course frame → offset in the right direction; `z` given and not given.

9. **LookAtLandmark** (approach, NavAction)
   - In: `map`, `pose`, `id`, plus the NavAction ports.
   - Mode ONLY_ORIENTATION, yaw = `atan2(dy, dx)` from the vehicle to the landmark, roll = pitch = 0.
   - Test: landmark ahead, left, behind → yaw 0, 90°, 180°; unknown id → FAILURE.

10. **RecordBag** (mission, decorator)
    - In: `profile` (e.g. "all"), `directory` (default `~/bags`).
    - Starts `ros2 bag record` for the profile's topics when the child starts; stops it (SIGINT) when the child finishes or is halted. Returns the child's status. A failed start logs a warning and still runs the child.
    - Test: child status is passed through; the process is started and stopped (check the directory exists afterwards).

Then: `trees/octagon.xml`.

### Karol · task: Bins

1. **Wait** (mission, stateful)
   - In: `seconds`. RUNNING until `seconds` have passed on the node clock, then SUCCESS. Halt resets it.
   - Test: RUNNING before, SUCCESS after; 0 → SUCCESS at once.

2. **SetOperationMode** (mission, stateful)
   - In: `mode` ("autonomous", "manual", ...). Calls `set_operation_mode` (`vortex_msgs/srv/SetOperationMode`) asynchronously; RUNNING until the response; SUCCESS if accepted. FAILURE on unknown mode, a missing service after 2 s, or a rejection.
   - Test: fake service accepts → SUCCESS with the right request; rejects → FAILURE; no service → FAILURE.

3. **LoadMissionConfig** (mission, sync)
   - In: `path` (default: the ROS parameter `mission_config`).
   - Reads the yaml (yaml-cpp) and writes every top-level key to the blackboard: numbers as double, the rest as string. FAILURE (with the reason) if the file cannot be read.
   - Test: a small yaml in the test → keys and types on the blackboard; missing file → FAILURE.

4. **MissionClock** (mission, sync)
   - In: `run_time_s`. Out: `clock` ({mission_clock}, `MissionClock{now, run_time_s}`). SUCCESS.
   - Test: start ≈ now, `run_time_s` set.

5. **ResetWorld** (mission, stateful)
   - Publishes `std_msgs/Empty` on `mission/wipe` and calls `landmark_server/clear` (`std_srvs/srv/Empty`). SUCCESS on the response, FAILURE if the service is missing after 2 s.
   - Test: fake service called and wipe received → SUCCESS; no service → FAILURE.

6. **StartRun** (mission, stateful)
   - In: `start_pose`, `heading_offset_deg`. Calls `landmark_server/set_course_frame` (`vortex_msgs/srv/SetCourseFrame`) with them. SUCCESS on the response.
   - Test: request contains the pose and the offset in radians; no service → FAILURE.

7. **LandmarkKnown** (map, condition)
   - In: `map`, `pose`, `type`, `subtype`, `max_age_s`, `min_forward_m`, `max_forward_m`, `exclude` (IdList). Out: `id`.
   - SUCCESS if a confirmed track matches type/subtype, is not excluded, was measured within `max_age_s`, and lies between `min_forward_m` and `max_forward_m` ahead of the vehicle (along its heading). Writes the nearest such id.
   - Test: each filter alone (type, subtype, ANY, age, forward window, exclude); unconfirmed ignored; unknown names → FAILURE.

8. **SelectLandmark** (map, sync)
   - In: `map`, `pose`, `type`, `subtype`, `exclude`, `sort` (NEAREST, LEFTMOST, RIGHTMOST). Out: `id`.
   - Like LandmarkKnown without age and window, choosing by `sort` (left/right relative to the vehicle heading). FAILURE if none.
   - Test: each sort order; exclude; none → FAILURE.

9. **ResolveRole** (mission, sync)
   - In: `role`. Out: `bin_subtype`, `torpedo_large`, `torpedo_large_other`, `torpedo_small`, `torpedo_small_other`, `octagon_image` (subtype names).
   - survey_repair → BIN_SURVEY_REPAIR, TORPEDO_TARGET_LARGE_SURVEY_REPAIR, ..._SEARCH_RESCUE as "other", OCTAGON_IMAGE_SURVEY; search_rescue the other way round. FAILURE on an unknown role.
   - Test: both roles give every output; unknown → FAILURE.

10. **FollowPoses** (motion, NavAction)
    - In: `poses` (PoseList), `mode`, plus the NavAction ports. One goal with one waypoint per pose. FAILURE on an empty list.
    - Test: count, order and mode of the waypoints; empty → no goal.

Then: `trees/bins.xml`.

### Amélie · task: Gate

1. **SetDepth** (motion, NavAction)
   - In: `z`. One waypoint, mode ONLY_Z, `pose.position.z = z`. (The example in "NavAction" below is this node.)
   - Test: z and mode in the goal; missing `z` → no goal.

2. **Surface** (motion, NavAction)
   - In: `z` (default 0.2). As SetDepth.
   - Test: default and given `z`.

3. **GoTo** (motion, NavAction)
   - In: `pose`, `mode` (default FULL_POSE, via `waypoint_mode_from_string`). One waypoint, frame WORLD.
   - Test: pose and mode; pose from the blackboard; missing pose or unknown mode → no goal.

4. **MoveRelative** (motion, NavAction)
   - In: `x`, `y`, `z`, `yaw_deg` (default 0), `frame` (BODY_RELATIVE or WORLD_RELATIVE), `mode`. One waypoint with the offset as its pose and `goal.frame` set; waypoint_manager resolves it at goal start.
   - Test: both frames; unknown frame → no goal.

5. **Turn** (motion, NavAction)
   - In: `pose`, `relative_deg`. Mode ONLY_ORIENTATION, yaw = current yaw + `relative_deg`, roll = pitch = 0.
   - Test: 90° from yaw 0 and from yaw 170° (wraps past 180°); missing pose → no goal.

6. **HoldPosition** (motion, NavAction)
   - In: `pose`, `seconds`. The current pose, mode FULL_POSE, `hold_time_sec = seconds`. `seconds` < 0: hold until halted (`goal.persistent = true`).
   - Test: hold time set; negative → persistent.

7. **SetGripper** (actuators, stateful)
   - In: `state` ("open" / "closed"). Sends a `GripperReferenceFilterWaypoint` goal (see `vortex_yasmin_utils/gripper_state`). SUCCESS on the result; FAILURE on an unknown state or a missing server.
   - Test: with a fake gripper server, as in `bt_test_utils.hpp`.

8. **DropMarker** (actuators, stateful)
   - In: `index` (0 or 1), `topic` (default "actuators/marker_dropper"). Publishes `std_msgs/Int8` with the index, waits `settle_s` (default 1.0), SUCCESS. FAILURE on an index outside 0–1.
   - Test: the message arrives with the index; bad index → FAILURE.

9. **FireTorpedo** (actuators, stateful)
   - In: `side` ("left" / "right"), `topic` (default "actuators/torpedo"). As DropMarker (0 = left, 1 = right).
   - Test: as DropMarker.

10. **MarkUsed** (actuators, sync)
    - In: `id`, `list` (IdList, bidirectional). Appends `id` if not already there. SUCCESS.
    - Test: appends; no duplicates; empty list created.

Then: `trees/gate.xml`.

The marker dropper and torpedoes have no ROS interface on the drone yet; the
topic is a port so the name can change without code changes.

## NavAction

Every node that moves the vehicle derives from `NavAction` and writes one
function: `make_goal()`, which builds the `WaypointManager` goal from its
ports. NavAction sends it, returns RUNNING until waypoint_manager answers,
and cancels the goal when the tree halts the node. `nullopt` from
`make_goal()` (a missing port, an unknown landmark) makes the node fail.

Every NavAction also has `position_tolerance`, `orientation_tolerance_deg`
and `hold_s`; they fill in each waypoint that does not set its own.
Timeouts are a `Timeout` in the XML.

```cpp
// include/vortex_bt_nodes/motion/set_depth.hpp
class SetDepth : public NavAction {
   public:
    using NavAction::NavAction;
    static BT::PortsList providedPorts() {
        return providedBasicPorts({BT::InputPort<double>("z")});
    }

   protected:
    std::optional<Goal> make_goal() override {
        const auto z = getInput<double>("z");
        if (!z) {
            spdlog::warn("[{}] missing z", name());
            return std::nullopt;
        }
        vortex_msgs::msg::Waypoint wp;
        wp.pose.position.z = *z;
        wp.waypoint_mode.mode = vortex_msgs::msg::WaypointMode::ONLY_Z;
        Goal goal;
        goal.waypoints.push_back(wp);
        goal.convergence_threshold = 0.1;
        return goal;
    }
};

// src/motion/register.cpp
factory.registerNodeType<SetDepth>("SetDepth", node);
```

## Testing a node alone

`test/bt_test_utils.hpp` has a fixture that builds a tree from an XML
string and ticks it the way the runner does, and a fake waypoint_manager
that succeeds, aborts, rejects or never finishes.

```cpp
// test/test_set_depth.cpp
#include "bt_test_utils.hpp"
#include "vortex_bt_nodes/motion/set_depth.hpp"

using namespace vortex_bt_nodes;
using test::BtNodeTest;

class SetDepthTest : public BtNodeTest {
   protected:
    void SetUp() override {
        BtNodeTest::SetUp();
        factory_.registerNodeType<motion::SetDepth>("SetDepth", client_node_);
    }
};

TEST_F(SetDepthTest, SendsTheDepth) {
    start_server();
    blackboard()->set("pose", make_pose(0.0, 0.0, 0.5));  // other nodes' keys
    auto tree = make_tree(R"(<SetDepth z="1.5"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(server_->last_goal.waypoints[0].pose.position.z, 1.5);
}
```

Nodes that do not move the vehicle need no server: set the blackboard,
`make_tree`, `run`, check the status and the output keys
(`blackboard()->get<int>("id")`). A `LandmarkMap` for a test is built by
hand: push `LandmarkTrack`s with type, subtype, pose and `confirmed = true`.

`test/test_nav_action.cpp` shows the fixture in use.

```bash
colcon build --packages-select vortex_bt_nodes
colcon test --packages-select vortex_bt_nodes && colcon test-result --verbose
```

## In the XML

- Pose: `"x;y;z"` or `"x;y;z;yaw_deg"`. PoseList: poses separated by `|`.
  IdList: `"3;7;12"`.
- Landmark types and subtypes by name, as in `LandmarkType.msg` and
  `LandmarkSubtype.msg`: `type="SLALOM_PIPE" subtype="SLALOM_PIPE_RED"`,
  `subtype="ANY"`. `landmark_type_from_string` and
  `landmark_subtype_from_string` in `types.hpp` convert them.
- Waypoint modes by name, as in `WaypointMode.msg`: `mode="POSITION_AND_YAW"`.
  `waypoint_mode_from_string` in `types.hpp` converts it (a wrapper around
  vortex_utils' `string_to_waypoint_mode`): `wp.waypoint_mode = *mode;`.
