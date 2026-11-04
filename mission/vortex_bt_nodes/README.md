# vortex_bt_nodes

BehaviorTree.CPP v4 nodes for Vortex missions. The trees and the runner are
in `robosub_mission`.

```
include/vortex_bt_nodes/
  common/nav_action.hpp    base for nodes that move the vehicle
  common/types.hpp         Pose types and how they are written in XML
  map/landmark_map.hpp   latest map and TF lookups, shared by the map nodes
  <area>/                  one header per node + register.hpp
src/<area>/                one source per node + register.cpp
```

Areas: `motion`, `map`, `mission`, `actuators`.

## Rules

- A node only depends on `common/` and `LandmarkMap`.
- A node is a header, a source file and one line in
  `src/<area>/register.cpp`. Use snake_case file names and namespace
  `vortex_bt_nodes::<area>`.
- A node does one thing. Retries, timeouts and fallbacks go in the XML.
- Log with spdlog: `spdlog::warn("[{}] ...", name());`
- A node is done when it is registered and handles success and every
  failure case listed below.

## How a node is built

`Wait` in `mission/` is a finished node to copy from. It has the four parts
every node has.

### 1. Base class

Pick it from what the node does:

| Base | Use it when | You write |
|---|---|---|
| `BT::SyncActionNode` | It finishes at once | `tick()` |
| `BT::StatefulActionNode` | It takes time | `onStart()`, `onRunning()`, `onHalted()` |
| `NavAction` | It moves the vehicle | `make_goal()` |
| `BT::ConditionNode` | It only checks something | `tick()` |

The tree is ticked at 10 Hz from one thread, the same thread that spins ROS.
A node that sleeps or waits in a loop stops every other node and every ROS
callback. So `Wait` never sleeps. It writes down when it is done and answers
RUNNING until then:

| | Called | `Wait` does |
|---|---|---|
| `onStart()` | The first tick | Reads `seconds`, stores the end time, returns RUNNING |
| `onRunning()` | Every tick after that, while it returned RUNNING | SUCCESS if the time has passed, otherwise RUNNING |
| `onHalted()` | The tree stops the node before it is done | Nothing to stop |

The next time the tree reaches the node it starts over with `onStart()`, so
set every member there and do not rely on what the last run left behind.

`onHalted()` is called when something above the node gives up on it, for
example a `Timeout` that fires or a `Fallback` that moves on. A node that
started something has to stop it there: cancel the goal, close the gripper
command, stop the motor. Leave it out and the tree moves on while the
vehicle keeps doing the old thing.

### 2. Ports

The inputs and outputs of the node, listed in `providedPorts()`. They are
the only way it talks to the rest of the tree:

```cpp
BT::InputPort<double>("seconds", "Time to wait [s]"),               // required
BT::InputPort<std::string>("level", "info", "info, warn or error"), // has a default
BT::OutputPort<Pose>("pose"),
```

In XML a port is either a value or one blackboard key: `seconds="2.5"` or
`seconds="{settle_s}"`. It cannot be a mix, `"Finished {task}"` is passed
on as that exact text.

### 3. The work

Read ports with `getInput`, write them with `setOutput`:

```cpp
const auto seconds = getInput<double>("seconds");
if (!seconds) {
    spdlog::error("[{}] {}", name(), seconds.error());
    return BT::NodeStatus::FAILURE;
}
```

`getInput` fails when the port is missing or its blackboard key is not set.
Return FAILURE then, and for values that make no sense. Do not throw, an
exception stops the whole mission. A value of the wrong type written
straight in the XML, like `seconds="abc"`, is caught when the tree loads.

`Wait` needs the time, so it takes the ROS node as a third constructor
argument and keeps it. Use `node_->now()` for time, it follows the simulator
clock. A node with a subscriber, a service or an action client makes it in
the constructor the same way. Its callback only stores what arrived, and
`onRunning()` looks at it on the next tick.

### 4. Registering

One line in `src/<area>/register.cpp`. The string is the tag name in XML,
`node` is the third constructor argument:

```cpp
factory.registerNodeType<Wait>("Wait", node);
```

### Using it

```xml
<Wait seconds="2.5"/>
<Timeout msec="1000">
  <Wait name="Settle" seconds="{settle_s}"/>
</Timeout>
```

`name` is what shows in the log and in the tree viewer. It defaults to the
tag name. The second `Wait` is halted by the `Timeout` if `settle_s` is
more than one second.

To try a node alone, put it in a small XML file and run it:

```bash
ros2 run robosub_mission robosub_mission --ros-args -p tree_file:=/path/to/my_test.xml
```

## The nodes you have

Kind: sync = `SyncActionNode`, stateful = `StatefulActionNode` (never
blocks), nav = derives from `NavAction`, condition = `ConditionNode`.

| Node | Area | Kind | Ports | Does |
|---|---|---|---|---|
| Wait | mission | stateful | `seconds` | Waits, on the simulator clock if there is one |
| GoToFrame | map | nav | `frame`, `offset`, `tool_frame`, `mode`, `resend_m`, `dead_reckoning_threshold`, `max_step_m`, `face_travel`, `sideways_first`, `right_angles`, `settle_s`, `stuck_s` | Goes to a TF frame plus an offset in that frame, and follows the frame while it moves. With `tool_frame` set, that frame is put on the target instead of `base_link`. FAILURE if a frame is not in TF at the start. Within 0.2 m and 6° of the target and no closer for `settle_s` (10): SUCCESS. No closer for `stuck_s` (30) anywhere else: FAILURE. Legs longer than 1.5 m are driven facing the way they go; `face_travel="false"` keeps the heading, to back out from an object before turning. `sideways_first="true"` lines up along the target's Y before going along its X |
| Measure | map | stateful | `class_name`, `out_frame`, `duration_s`, `max_range_m`, `fallback_from`, `accept_fallback` | Calls the `landmark_server` measure action and sets `<class_name>/measured`. Hold still while it runs |
| LookAtFrame | map | nav | `frame` | Turns on the spot to face a TF frame. FAILURE if it is not in TF |
| Turn | map | nav | `yaw_deg` or `relative_deg` | Turns to a heading in the map frame, or by an angle from the current heading |
| LandmarkSeen | map | condition | `id` or `type`/`subtype`, `within_s` (2.0) | SUCCESS if it was detected in the last `within_s` seconds |
| FireTorpedo | actuators | stateful | `service` (`fire_torpedo`), `settle_s` (2), `timeout_s` (5) | Calls the `std_srvs/Trigger` service, then waits `settle_s`. FAILURE if the service is not there or says no |

These are enough to start on a task in the simulator. `GoToFrame` also changes
depth, with the Z of the offset.

Hints:
- An offset is in the frame's own axes. Check which way the frame's X points
  before choosing the sign. A wrong sign puts the goal on the other side of
  the object.
- `tool_frame` is for aiming: `tool_frame="torpedo_tube"` puts the tube in front of
  the opening, not the middle of the vehicle, and `marker_dropper` above the bin.
  They are in `auv_setup/description/nautilus.urdf.xacro`, and the simulator fires
  from them too. The positions are not measured yet, correct them there.
- The openings are small, about 7 and 5.5 cm in radius in the simulator, and a torpedo
  falls slowly on its way. Compare the tube with the opening in TF before you fire.
  The vehicle can hang a few centimetres off in depth, and a move counts as done at
  0.1 m unless you ask for less.
- The torpedo openings are classes with frames of their own
  (`torpedo_opening_large_left`, `_large_right`, `_small_top`, `_small_bottom`).
- A heading in the map frame is not the same heading in `odom`.
- In the simulator `fire_torpedo` and `drop_marker` are `std_srvs/Trigger` services
  with no limit on the number of shots. Try them by hand:
  `ros2 service call /nautilus/fire_torpedo std_srvs/srv/Trigger`

## Who does what

Everyone makes the tree for their task, and the measurement if `Measure` on one
class is not enough, see "A measurement for your task" in the `landmark_server`
README. The nodes below are not on this branch yet.

| Person | Task | Nodes |
|---|---|---|
| Amélie | Gate | Log, SetDepth, Surface, MoveRelative, SetGripper |
| André | Slalom | Search |
| Johannes | Torpedo | |
| Karol | Bins | LoadMissionConfig, WaitForStart, DropMarker |
| Ashish | Octagon | |

| Node | Area | Kind | Ports | Does |
|---|---|---|---|---|
| Log | mission | sync | `message`, `level` | Logs with spdlog (info, warn or error) and returns SUCCESS |
| SetDepth | motion | nav | `z` | Goes to depth `z`, keeps x, y and heading |
| Surface | motion | nav | `z` (0.2) | As SetDepth, with a shallow default |
| MoveRelative | motion | nav | `offset`, `frame` (BODY or WORLD), `mode` | Moves by an offset from where the vehicle is when the node starts |
| SetGripper | actuators | stateful | `roll`, `pinch`, `mode`, `action` | Sends a gripper goal. FAILURE if it is rejected, cancels when halted |
| Search | motion | nav | `pattern` (ROTATE_STEPS or SCAN_ARC), `step_deg`, `arc_deg`, `pause_s` | Turns on the spot, pausing at each heading. FAILURE when the sweep is done, so the tree stops it when the target is found |
| LoadMissionConfig | mission | sync | `path` | Writes every key in the yaml to the blackboard. Nested keys become `a.b`. FAILURE if the file can't be read |
| WaitForStart | mission | stateful | `service` | RUNNING until the killswitch is off and the vehicle is in autonomous mode |
| DropMarker | actuators | stateful | `service` (`drop_marker`), `settle_s` | As `FireTorpedo`, on the `drop_marker` service |

Hints:
- Look at the `WaypointMode` message. There is a mode for depth only.
- The `WaypointManager` action has a `frame` field for relative goals.
- Search until something is in view with `LandmarkSeen` in a `ReactiveFallback`,
  and give it a `Timeout`.
- BehaviorTree.CPP checks port types. Writing the values as text lets every
  port read them with its own type.
- The operation mode is only published when it changes, so ask for it with
  the `get_operation_mode` service.

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
- On `GoToFrame` be loose on a rough leg, e.g. `position_tolerance="0.5"
  orientation_tolerance_deg="15"` to get near enough to measure, and strict on
  the last move. Without a tolerance it counts as there at 0.2 m and 6 degrees
  once it stops getting closer.
- Something you must hit: approach in steps, measure, move, measure again,
  tighter each time. The last measurement from close up is the one to trust.

## Testing in the simulator

One script starts everything in a tmux session. Run it from the workspace
root:

```bash
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh
```

That gives the simulator with the RoboSub course, the controller, the
waypoint manager, the landmark server, and the vehicle armed in autonomous
mode. The simulator publishes the course landmarks itself (exact, no noise by
default). The command for your tree is typed in
the `mission` window, press Enter to run it.

Common uses:

```bash
# Run a tree as soon as everything is up
tmux_robosub_sim.sh --tree Main

# Start at your task, with noisy detections
tmux_robosub_sim.sh --spawn torpedo --profile realistic

# No GPU: no window and no RoboSub course, so no landmarks to measure
tmux_robosub_sim.sh --no-gpu
```

| Option | Default | |
|---|---|---|
| `--tree <name>` | | Tree to run when everything is up |
| `--spawn <name>` | | Start at a spawn point: `slalom`, `torpedo`, `bin`, `octagon` |
| `--profile <name>` | `none` | Detection errors, see below |
| `--blank` | | No prior map, as at a new pool. Starts the map GUI |
| `--no-start` | | Leave the killswitch on and the mode manual |
| `--no-gpu` | | No rendering and no RoboSub course, so no landmarks |
| `--no-keyboard-joy` | | No keyboard joystick. It is on by default: its keys drive the vehicle and also toggle the killswitch and the mode |
| `--no-foxglove` | | Do not start the Foxglove bridge |
| `--no-debug` | | No landmark markers or NIS |
| `--domain-id <id>` | 0 | `ROS_DOMAIN_ID` |

Windows: `sim` (simulator, controller, waypoint manager, odometry), `map`
(landmark server, start, Foxglove), `mission` (your tree and a
free pane). Switch with `Ctrl-b` and the window number. Stop everything
with `tmux kill-session -t robosub_sim`.

To run a part by hand, the commands are at the top of the script.

### Detection profiles

Pick the profile in the simulator window (the "Detection profile" dropdown,
top right), or set `profile` in `<ros_landmarks>` in
`vortex-stonefish-sim/stonefish_sim/scenarios/robosub.scn`, or pass `--profile`. To leave objects
out, untick "Publish this landmark" in the editor (Tab) and save.

| Profile | Detections |
|---|---|
| `none` | Perfect. Every object is published all the time, wherever the vehicle is |
| `realistic` | Only what the cameras could see. Good up close, noisy and unreliable far away |
| `unstable` | Noisy with dropouts at every distance |
| `erratic` | Worse than `unstable` (gate posts are not reported as slalom pipes yet) |

Start with `none` to check your logic, then use `realistic`.

The map only takes detections within 10 m of the vehicle, so even with
`none` the far objects show up as you get closer.

### Checking that it runs

```bash
ros2 topic echo --once /nautilus/landmark_server/landmarks   # the map
ros2 run tf2_ros tf2_echo nautilus/odom nautilus/prior_gate  # a target frame
ros2 run tf2_ros tf2_echo nautilus/odom nautilus/base_link   # the vehicle
```

To look at it, connect Foxglove to `ws://localhost:8765` and open the layout
in
`vortex-auv/mission/landmark_server/foxglove/landmark_server.json`. The
running tree shows in Groot2 or the VS Code BehaviorTree Viewer on port 1666.

### After changing code

The simulator can stay up. Rebuild, then run your tree again from the
`mission` window:

```bash
colcon build --symlink-install --packages-select vortex_bt_nodes robosub_mission
ros2 launch perception_setup robosub_mission.launch.py main_tree:=Main
```

Trees are read when the mission starts, so a change
to an XML or yaml file needs no rebuild. `Can't find a tree with name`
means the tree is not in `course.xml` or its file is not included there.

### Between runs

Run the script again, or drive the vehicle back and reset the map:

```bash
ros2 topic pub --once /nautilus/mission/wipe std_msgs/msg/Empty
```

## Good to know

- XML formats (Pose, PoseList, type/subtype/mode names) are in the comment
  at the top of `common/types.hpp`, with the converters.
- A landmark is named by `id`, or by `type` + `subtype`.
  `LandmarkMap::ports()` and `resolve()` handle this for you.
- Interfaces (under `/nautilus`): action `waypoint_manager`,
  `landmark_server/landmarks` and its TF frames, `get_operation_mode`,
  TF `odom -> base_link`. The `landmark_server` README describes the map.
