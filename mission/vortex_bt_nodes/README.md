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

## What each person writes, in order

Follow the list from the top. Ports and blackboard keys are those in the
course tree plan (section 8); `→` marks outputs.

### Johannes

| # | Node | Area | Kind | Ports / blackboard | What it does |
|---|---|---|---|---|---|
| 1 | LogError | mission | Sync action | `message` | Logs `message` with spdlog as an error; returns SUCCESS |
| 2 | TaskSlot | mission | Decorator | `task`, `clock` ({mission_clock}), `budget_s`, `reserve_s` | Skips the child if less than `reserve_s` is left, halts it after `budget_s`; always SUCCESS, logs the outcome |
| 3 | SelectGatePanel | motion | Stateful action | {map}, `gate_id`, `preferred_role` → `panel_id`, `role`, `gate_side` | Waits for the gate's role panels in the map, picks ours; falls back to what is seen |
| 4 | CommitEstimate | approach | Stateful action | {map}, `id`, `samples`, `position_std_m`, `yaw_std_deg` | SUCCESS once the last `samples` map poses of `id` are within the spreads |
| 5 | ApproachLandmark | approach | NavAction | {map}, `id` or `type`/`subtype`, `frame`, `x y z yaw_deg`, `mode`, `freeze`, `dead_reckoning_m`, `track_loss_timeout_s` | Goes to an offset from a landmark (landmark_targets), re-sending as the map estimate moves |

### André

| # | Node | Area | Kind | Ports / blackboard | What it does |
|---|---|---|---|---|---|
| 1 | VehicleHealthy | mission | Condition | {pose}, `max_depth_m` | FAILURE when deeper than `max_depth_m` |
| 2 | Search | map | NavAction | `pattern` (ROTATE_STEPS, SCAN_ARC, LAWNMOWER, EXPANDING_SQUARE) and its numbers, {pose} | Drives the pattern around the current pose. Stopped from the XML (`ReactiveFallback` with `LandmarkKnown`) |
| 3 | MatchPipes | map | Sync action | {map}, {pose}, {gate_side}, `exclude` ({passed_red_ids}) → `gap_pose`, `red_id`, `offset` | Finds the next slalom layer (red + whites ahead) and the gap on our side. `~/slalom_guide.md` |
| 4 | RecordLayer | map | Sync action | {pose}, `red_id` → `layers`, `passed` ({passed_red_ids}) | Remembers the layer just passed, so it is not matched again |
| 5 | AvoidSlalom | map | Sync action | {map}, {course_frame}, {slalom_layers} → `path` ({avoid_path}) | Poses around the slalom for Return Home |

### Ashish

| # | Node | Area | Kind | Ports / blackboard | What it does |
|---|---|---|---|---|---|
| 1 | PoseFeeder | map | Sync action | `topic`, `max_age_s` → `pose` ({pose}) | Subscribes to the pose; FAILURE when older than `max_age_s` |
| 2 | MapFeeder | map | Sync action | `topic`, `max_age_s` → `map` ({map}) | Same for `landmark_server/object_map` |
| 3 | CourseFrameFeeder | map | Sync action | `frame` → `course_frame` ({course_frame}) | TF lookup odom → `nautilus/course` |
| 4 | SavePose | mission | Sync action | {pose} → `saved` | Copies the pose to another key ({start_pose}) |
| 5 | GoToSavedPose | mission | NavAction | `saved`, `mode` | Goes back to a saved pose |
| 6 | VerifyInside | mission | Condition | {map}, {pose}, `type`, `subtype`, `radius_m`, `margin_m` | SUCCESS if horizontally within `radius_m − margin_m` of the landmark |
| 7 | GoToCourse | motion | NavAction | {course_frame}, `x y z yaw_deg` | A point in the course frame, converted to odom |
| 8 | MoveCourse | motion | NavAction | {course_frame}, {pose}, `dx`, `dy`, `z` | Moves along the course axes from the current pose |
| 9 | LookAtLandmark | approach | NavAction | {map}, {pose}, `id` | Yaw towards a landmark, position unchanged |
| 10 | RecordBag | mission | Decorator | `profile` | Starts a rosbag while the child runs, stops it after |

### Karol

| # | Node | Area | Kind | Ports / blackboard | What it does |
|---|---|---|---|---|---|
| 1 | Wait | mission | Stateful action | `seconds` | RUNNING for `seconds`, then SUCCESS |
| 2 | SetOperationMode | mission | Stateful action | `mode` | Calls `set_operation_mode` (vortex_msgs/SetOperationMode) |
| 3 | LoadMissionConfig | mission | Sync action | `path` | Writes every key in `mission.yaml` to the blackboard |
| 4 | MissionClock | mission | Sync action | `run_time_s` → `clock` ({mission_clock}) | Starts the run clock (`MissionClock` in `types.hpp`) |
| 5 | ResetWorld | mission | Stateful action | – | Publishes `mission/wipe` and calls `landmark_server/clear` |
| 6 | StartRun | mission | Stateful action | `start_pose`, `heading_offset_deg` | Calls `landmark_server/set_course_frame` |
| 7 | LandmarkKnown | map | Condition | {map}, {pose}, `type`, `subtype`, `max_age_s`, `min_forward_m`, `max_forward_m`, `exclude` → `id` | SUCCESS if a matching confirmed landmark is in the map |
| 8 | SelectLandmark | map | Sync action | {map}, {pose}, `type`, `subtype`, `sort`, `exclude` → `id` | Picks one (nearest, leftmost, ...) |
| 9 | ResolveRole | mission | Sync action | `role` → `bin_subtype`, `torpedo_*`, `octagon_image` | Role → the subtype names each task looks for |
| 10 | FollowPoses | motion | NavAction | `poses` (PoseList), `mode` | Several waypoints in one goal |

### Amélie

| # | Node | Area | Kind | Ports / blackboard | What it does |
|---|---|---|---|---|---|
| 1 | SetDepth | motion | NavAction | `z` | One waypoint, mode `ONLY_Z` |
| 2 | Surface | motion | NavAction | `z` (default 0.2) | As SetDepth, near the surface |
| 3 | GoTo | motion | NavAction | `pose`, `mode` | One waypoint in odom |
| 4 | MoveRelative | motion | NavAction | `x y z yaw_deg`, `frame` (BODY_RELATIVE, WORLD_RELATIVE), `mode` | Offset from where the vehicle is when the goal starts |
| 5 | Turn | motion | NavAction | {pose}, `relative_deg` | Mode `ONLY_ORIENTATION`, yaw = current + `relative_deg` |
| 6 | HoldPosition | motion | NavAction | {pose}, `seconds` | The current pose with `hold_s = seconds` |
| 7 | SetGripper | actuators | Stateful action | `state` | Sends the gripper goal (see `vortex_yasmin_utils/gripper_state`) |
| 8 | DropMarker | actuators | Stateful action | `index`, `topic` | Triggers the marker dropper |
| 9 | FireTorpedo | actuators | Stateful action | `side`, `topic` | Fires a torpedo |
| 10 | MarkUsed | actuators | Sync action | `id` → `list` | Adds an id to a list (bins used, torpedoes fired) |

The marker dropper and torpedoes have no ROS interface yet. Make the topic
a port and publish a simple message; the name can change without code
changes.

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
