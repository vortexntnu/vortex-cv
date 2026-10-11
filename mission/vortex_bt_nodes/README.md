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
| GoToFrame | map | nav | `frame`, `offset`, `tool_frame`, `mode`, `resend_m`, `dead_reckoning_threshold`, `max_step_m` | Goes to a TF frame plus an offset in that frame, and follows the frame while it moves. With `tool_frame` set, that frame is put on the target instead of `base_link`. FAILURE if a frame is not in TF at the start |

Hints:
- `GoToFrame` is the node every tree uses most, so do it early.
- The controller works in `odom`. A frame that is fixed in `map` moves in
  `odom` whenever the map corrects drift. Look the frame up again while
  driving and send a new goal when it has moved more than `resend_m`.
  `NavAction::update_goal()` is made for this.
- Close to an object the detections get worse. Stop updating the goal within
  `dead_reckoning_threshold`.
- An offset is in the frame's own axes. Check which way the frame's X points
  before choosing the sign. A wrong sign puts the goal on the other side of
  the object.
- One long move can saturate the thrusters and flip the vehicle. Split moves
  longer than `max_step_m` into several waypoints.
- `tool_frame` is for aiming: the torpedo tube should end up in front of
  the opening, not the middle of the vehicle. Work out where `base_link`
  has to be for that. The tubes and the dropper have no link in the robot
  description yet, so they need to be added with measured positions.
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

## Testing in the simulator

One script starts everything in a tmux session. Run it from the workspace
root:

```bash
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh
```

That gives the simulator with the RoboSub course, the controller, the
waypoint manager, exact detections with no noise, the landmark server, and
the vehicle armed in autonomous mode. The command for your tree is typed in
the `mission` window, press Enter to run it.

Common uses:

```bash
# Run a tree as soon as everything is up
tmux_robosub_sim.sh --tree TestGate

# Detections like a real camera
tmux_robosub_sim.sh --profile realistic --tree TestGate

# Only some of the objects
tmux_robosub_sim.sh --tasks gate,slalom

# No GPU: no window, everything else the same
tmux_robosub_sim.sh --no-gpu
```

| Option | Default | |
|---|---|---|
| `--profile <name>` | `none` | Detection noise, see the table below |
| `--tasks <list>` | all | Objects to publish: `gate`, `slalom`, `torpedo_board`, `bin`, `octagon`, `table` |
| `--seed <n>` | 7 | Which role image is where, the same for simulator and detections |
| `--tree <name>` | | Tree to run when everything is up |
| `--no-start` | | Leave the killswitch on and the mode manual |
| `--no-gpu` | | No rendering |
| `--no-keyboard-joy` | | No keyboard joystick. It is on by default: its keys drive the vehicle and also toggle the killswitch and the mode |
| `--no-foxglove` | | Do not start the Foxglove bridge |
| `--no-debug` | | No landmark markers or NIS |
| `--domain-id <id>` | 0 | `ROS_DOMAIN_ID` |

Windows: `sim` (simulator, controller, waypoint manager, odometry), `map`
(detections, landmark server, start, Foxglove), `mission` (your tree and a
free pane). Switch with `Ctrl-b` and the window number. Stop everything
with `tmux kill-session -t robosub_sim`.

To run a part by hand, the commands are at the top of the script.

### Detection profiles

| `--profile` | Detections |
|---|---|
| `none` | Perfect. Every object is published all the time, wherever the vehicle is |
| `realistic` | Only what the cameras could see. Good up close, noisy and unreliable far away |
| `unstable` | Noisy with dropouts at every distance |
| `erratic` | Worse than `unstable`, and gate posts get reported as slalom pipes |

Start without a profile to check your logic, then use `realistic`.

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
ros2 launch perception_setup robosub_mission.launch.py config:=sim main_tree:=TestGate
```

Trees and the mission config are read when the mission starts, so a change
to an XML or yaml file needs no rebuild. `Can't find a tree with name`
means the tree is not in `root.xml` or its file is not included there.

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
