# vortex_bt_nodes

BehaviorTree.CPP v4 nodes for Vortex missions, set up like
`vortex_yasmin_utils`. The trees and the runner are in `robosub_mission`.

```
include/vortex_bt_nodes/
  common/nav_action.hpp    base for nodes that move the vehicle
  common/types.hpp         blackboard types, landmark and mode names
  <area>/                  one header per node + register.hpp
src/<area>/                one source per node + register.cpp
test/bt_test_utils.hpp     test fixture + fake waypoint_manager
test/test_<node>.cpp       one test per node
```

Areas: `motion`, `map`, `approach`, `actuators`, `mission`.

## Rules

- A node only depends on `common/`. If your node reads a key another
  person's node writes, set that key by hand in your test.
- Blackboard keys and their types are listed in `types.hpp`. A new key that
  other people read goes there first.
- A node is a header, a source file, a test and one line in
  `src/<area>/register.cpp`. Use snake_case file names and namespace
  `vortex_bt_nodes::<area>`.
- A node does one thing. Retries, timeouts and fallbacks go in the XML.
- Log with spdlog: `spdlog::warn("[{}] ...", name());`
- A node is done when its test passes and it is registered. The test covers
  success and every failure case listed below.

## Who makes what

Write your nodes in the order listed, then your task tree (see the
`robosub_mission` README).

| Person | Task | Nodes |
|---|---|---|
| Johannes | Torpedo | LogError, TaskSlot, LookAtLandmark, SelectGatePanel, CommitEstimate, RecordBag, ApproachLandmark |
| André | Slalom | SelectLandmark, LandmarkKnown, FollowPoses, Search, RecordLayer, MatchPipes, AvoidSlalom |
| Ashish | Octagon | PoseFeeder, MapFeeder, CourseFrameFeeder, SavePose, GoToSavedPose, VerifyInside, GoToCourse, MoveCourse |
| Karol | Bins | Wait, MissionClock, SetOperationMode, ResetWorld, StartRun, LoadMissionConfig, ResolveRole, DropMarker |
| Amélie | Gate | SetDepth, Surface, GoTo, MoveRelative, Turn, HoldPosition, FireTorpedo, SetGripper, MarkUsed |

Kind: sync = `SyncActionNode`, stateful = `StatefulActionNode` (never
blocks), nav = derives from `NavAction`. `→` marks an output port.

### Johannes

| Node | Kind | Ports | Does |
|---|---|---|---|
| LogError | sync | `message` | `spdlog::error`, returns SUCCESS |
| TaskSlot | decorator | `task`, `clock`, `budget_s`, `reserve_s` | Skips the child if less than `reserve_s` is left, halts it after `budget_s`. Always SUCCESS; logs the outcome |
| LookAtLandmark | nav | `map`, `pose`, `id` | Mode ONLY_ORIENTATION, yaw towards the landmark. FAILURE on an unknown id |
| SelectGatePanel | stateful | `map`, `gate_id`, `preferred_role` → `panel_id`, `role`, `gate_side` | Picks the gate panel with our role, or the only panel seen. RUNNING until a panel is in the map |
| CommitEstimate | stateful | `map`, `id`, `samples`, `position_std_m`, `yaw_std_deg` | SUCCESS once the last `samples` map poses are within the limits. FAILURE on an unknown id |
| RecordBag | decorator | `profile`, `directory` | Runs `ros2 bag record` while the child runs, then passes on the child's status |
| ApproachLandmark | nav | `map`, `pose`, `id` or `type`/`subtype`, `frame`, `x y z yaw_deg`, `mode`, `tool_frame`, `freeze`, `dead_reckoning_m`, `track_loss_timeout_s` | Goes to an offset from a landmark (`landmark_targets`). Sends the goal again when the landmark moves, except with `freeze` or within `dead_reckoning_m`. FAILURE on track loss |

ApproachLandmark needs NavAction to resend a goal. Add a
`virtual bool should_resend()` to `NavAction` in its own commit, with a
test.

### André

| Node | Kind | Ports | Does |
|---|---|---|---|
| SelectLandmark | sync | `map`, `pose`, `type`, `subtype`, `exclude`, `sort` → `id` | Picks a confirmed match by NEAREST / LEFTMOST / RIGHTMOST. FAILURE if none |
| LandmarkKnown | condition | `map`, `pose`, `type`, `subtype`, `max_age_s`, `min_forward_m`, `max_forward_m`, `exclude` → `id` | SUCCESS if a confirmed, recent match is in the forward window |
| FollowPoses | nav | `poses`, `mode` | One goal, one waypoint per pose. FAILURE on an empty list |
| Search | nav | `pose`, `pattern`, pattern numbers | ROTATE_STEPS, SCAN_ARC, LAWNMOWER or EXPANDING_SQUARE around the start pose. The XML stops it when a landmark is found |
| RecordLayer | sync | `pose`, `red_id`, `layers`, `passed` | Adds the passed layer to `layers` and `red_id` to `passed` |
| MatchPipes | sync | `map`, `pose`, `gate_side`, `exclude`, `offset` → `gap_pose`, `red_id` | Next slalom layer and the gap on our side (`landmark_targets::match_pipes`). FAILURE if no layer. See `~/slalom_guide.md` |
| AvoidSlalom | sync | `map`, `course_frame`, `layers`, `side`, `clearance_m` → `path` | Poses around the slalom for Return Home |

### Ashish

| Node | Kind | Ports | Does |
|---|---|---|---|
| PoseFeeder | sync | `topic`, `max_age_s` → `pose` | Latest `PoseWithCovarianceStamped` (SensorDataQoS). FAILURE if none or too old |
| MapFeeder | sync | `topic`, `max_age_s` → `map` | The same for `landmark_server/object_map` |
| CourseFrameFeeder | sync | `frame`, `odom_frame` → `course_frame` | TF lookup odom → course. FAILURE if there is no transform |
| SavePose | sync | `pose` → `saved` | Copies the pose |
| GoToSavedPose | nav | `saved`, `mode` | Goes to the saved pose |
| VerifyInside | condition | `map`, `pose`, `type`, `subtype`, `radius_m`, `margin_m` | SUCCESS if horizontally within `radius_m − margin_m` of the landmark |
| GoToCourse | nav | `course_frame`, `x y z yaw_deg`, `mode` | Point in the course frame, converted to odom |
| MoveCourse | nav | `course_frame`, `pose`, `dx`, `dy`, `z` | Moves along the course axes from the current pose |

### Karol

| Node | Kind | Ports | Does |
|---|---|---|---|
| Wait | stateful | `seconds` | RUNNING for `seconds` |
| MissionClock | sync | `run_time_s` → `clock` | Starts the run clock |
| SetOperationMode | stateful | `mode` | Calls `set_operation_mode`. FAILURE if rejected or there is no service |
| ResetWorld | stateful | none | Publishes `mission/wipe` and calls `landmark_server/clear` |
| StartRun | stateful | `start_pose`, `heading_offset_deg` | Calls `landmark_server/set_course_frame` |
| LoadMissionConfig | sync | `path` | Writes every key in the yaml to the blackboard. FAILURE if the file can't be read |
| ResolveRole | sync | `role` → `bin_subtype`, `torpedo_*`, `octagon_image` | Role → the subtype names each task looks for |
| DropMarker | stateful | `index`, `topic` | Publishes `std_msgs/Int8` and waits `settle_s`. FAILURE if the index is not 0 or 1 |

### Amélie

| Node | Kind | Ports | Does |
|---|---|---|---|
| SetDepth | nav | `z` | Mode ONLY_Z |
| Surface | nav | `z` (0.2) | As SetDepth |
| GoTo | nav | `pose`, `mode` | One waypoint in odom |
| MoveRelative | nav | `x y z yaw_deg`, `frame`, `mode` | `goal.frame` BODY_RELATIVE or WORLD_RELATIVE |
| Turn | nav | `pose`, `relative_deg` | Mode ONLY_ORIENTATION, yaw = current + `relative_deg` |
| HoldPosition | nav | `pose`, `seconds` | Current pose with a hold time; negative = until halted |
| FireTorpedo | stateful | `side`, `topic` | Publishes `std_msgs/Int8` (0 left, 1 right). FAILURE on an unknown side |
| SetGripper | stateful | `state` | Gripper goal (see `vortex_yasmin_utils/gripper_state`) |
| MarkUsed | sync | `id`, `list` | Adds `id` to the list once |

The marker and torpedo topics are placeholders until the drone has an
interface for them.

## NavAction

Derive from `NavAction` and write `make_goal()`. Return `std::nullopt` on
bad input and the node fails. NavAction sends the goal, returns RUNNING,
and cancels the goal when the node is halted. Every NavAction also has the
ports `position_tolerance`, `orientation_tolerance_deg` and `hold_s`.

```cpp
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

## Tests

```cpp
class SetDepthTest : public test::BtNodeTest {
   protected:
    void SetUp() override {
        BtNodeTest::SetUp();
        factory_.registerNodeType<motion::SetDepth>("SetDepth", client_node_);
    }
};

TEST_F(SetDepthTest, SendsTheDepth) {
    start_server();                                  // fake waypoint_manager
    blackboard()->set("pose", make_pose(0, 0, 0.5)); // keys other nodes write
    auto tree = make_tree(R"(<SetDepth z="1.5"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(server_->last_goal.waypoints[0].pose.position.z, 1.5);
}
```

Nodes that don't move the vehicle don't need `start_server()`. To give a
test a map, fill a `LandmarkMap` with `LandmarkTrack`s by hand. See
`test/test_nav_action.cpp` for more examples.

```bash
colcon build --packages-select vortex_bt_nodes
colcon test --packages-select vortex_bt_nodes && colcon test-result --verbose
```

## XML formats

- Pose `"x;y;z;yaw_deg"`, PoseList `"1;0;1 | 2;0;1"`, IdList `"3;7"`.
- Types, subtypes and modes by their msg names: `type="SLALOM_PIPE"`,
  `subtype="SLALOM_PIPE_RED"` or `"ANY"`, `mode="POSITION_AND_YAW"`. Convert
  them with the `*_from_string` functions in `types.hpp`.
