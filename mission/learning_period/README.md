# Learning Period: Waypoint Patrol FSM

Welcome to the Vortex NTNU Autonomy group. This exercise gets you comfortable
with our YASMIN + ROS 2 mission stack by having you finish a small state
machine that sends the drone to 4 fixed waypoints in sequence.

You will **not** write any new ROS 2 action-client code. All the goal-sending
logic already exists as a reusable state (`WaypointGoalState`) in our
`vortex_yasmin_utils` package. Your job is to compose existing building
blocks, plus write one tiny state of your own.

If you've never touched ROS 2 or YASMIN before, that's the point — read
[Concepts](#concepts) before you start if any of this is unfamiliar.

## 1. What you're building

Target state machine:

```mermaid
stateDiagram-v2
    [*] --> HELLO
    HELLO --> WP1: SUCCEED
    WP1 --> WP2: SUCCEED
    WP2 --> WP3: SUCCEED
    WP3 --> WP4: SUCCEED
    WP4 --> DONE: SUCCEED
    DONE --> [*]: SUCCEED

    HELLO --> ABORT: ABORT
    WP1 --> ABORT: ABORT
    WP1 --> ABORT: CANCEL
    WP2 --> ABORT: ABORT
    WP2 --> ABORT: CANCEL
    WP3 --> ABORT: ABORT
    WP3 --> ABORT: CANCEL
    WP4 --> ABORT: ABORT
    WP4 --> ABORT: CANCEL
    ABORT --> [*]
```

- **HELLO** — your own custom state (`HelloState`). Logs a greeting, always
  returns `SUCCEED`.
- **WP1..WP4** — four `WaypointGoalState` instances, each sending one of the
  four poses in [`config/waypoints.yaml`](config/waypoints.yaml) to the
  `WaypointManager` action server.
- **DONE** — a trivial terminal state that logs completion.
- **ABORT/CANCEL** — if any waypoint state comes back `ABORT` (the action
  server rejected/failed the goal) or `CANCEL` (someone canceled the FSM,
  e.g. Ctrl-C), the machine goes straight to the terminal `ABORT` outcome.
  Nothing tries to retry or recover mid-sequence — that's deliberate, and
  it's also stretch goal #3 below if you want to change it.

## 2. Git workflow

Branch off the shared exercise branch, using your own name:

```shell
git checkout autonomy-learning-period
git pull
git checkout -b learning-period-<your-name>
```

Commit as you go with plain, scoped messages (`git commit -m "wire HELLO and WP1 transitions"`,
not one giant commit at the end). When you're done (see the
[checklist](#done-when) below), push and open a PR **against
`autonomy-learning-period`**, not `main`:

```shell
git push -u origin learning-period-<your-name>
```

Then open a PR in GitHub with base branch `autonomy-learning-period`.

## 3. Environment setup

This assumes you already have the full Vortex NTNU ROS 2 workspace checked
out and built (this repo alone does not contain `auv_setup`, `waypoint_manager`,
or the simulator itself — see [Open questions](#open-questions)). From your
workspace root:

```shell
colcon build --packages-up-to learning_period vortex_yasmin_utils
source install/setup.bash
```

**Start the simulator first, on its own, before anything below.** It is not
part of this repo — it lives in `vortex-auv` and has its own tmux launcher,
[`launch_drone_sim.sh`](../../../vortex-auv/utility_scripts/launch_drone_sim.sh).
From your workspace root:

```shell
src/vortex-auv/utility_scripts/launch_drone_sim.sh --scenario default
```

`--scenario` picks the Stonefish world; `default` is fine for this exercise.
Run `launch_drone_sim.sh --help` for the full list (GPU scenarios like
`docking`, `pipeline`, `tacc`, plus `*_no_gpu` variants if you're on a
machine without a GPU). Under the hood this opens its own tmux session
(`drone_launch`, separate from `learning_period` below, so the two don't
collide) with:

- **`sim` window** — `stonefish_sim` itself, plus
  `auv_setup dp_quat.launch.py` (the DP/guidance controller —
  `WaypointGoalState` goals won't converge without this running).
- **`tools` window** — `foxglove_bridge` and a `message_publisher` node, and
  it also opens Foxglove Studio locally if it isn't already running.

Like the script below, re-running it kills any existing `drone_launch`
session first, and `Ctrl-b d` detaches without stopping it.

Once the simulator is up, launch the rest of the stack with the tmux script
written for this exercise, from your workspace root:

```shell
src/vortex-cv/perception_setup/scripts/tmux_learning_period.sh
```

This mirrors the convention every other mission in this repo uses (see
`tmux_pipeline_inspection.sh`, `tmux_valve.sh`,
`tmux_subsea_docking.sh` in the same folder) — one tmux session, `auto`
window, one pane per process. It opens 2 windows:

- **`auto`** (focused on start) — 4 panes:
  1. `ros2 launch waypoint_manager waypoint_manager.launch.py` — the DP/guidance
     action server your `WaypointGoalState`s actually send goals to.
  2. `ros2 launch landmark_server landmark_server.launch.py` — started
     alongside as part of the baseline "auto" stack in every mission here;
     this exercise's FSM doesn't use it directly, but launching without it
     hasn't been tested and mission stacks elsewhere always start it.
  3. `ros2 launch learning_period learning_period_fsm.launch.py` — your FSM.
  4. `ros2 run yasmin_viewer yasmin_viewer_node` — the web viewer (see
     [Inspecting the running FSM](#inspecting-the-running-fsm)).
- **`misc`** — 4 empty, sourced panes for whatever else you need
  (`ros2 topic echo`, `rqt`, etc.).

Detach with `Ctrl-b d` (session keeps running), reattach with
`tmux attach -t learning_period`, or just re-run the script — it kills any
existing `learning_period` session first. Useful flags:

```shell
./tmux_learning_period.sh --drone nautilus --domain-id 0
```

## 4. Task list

Work through these in order. Each step names the file and the
`// TODO(member)` marker to look for.

1. **Add your `HelloState` body.**
   File: [`mission/vortex_yasmin_utils/src/hello_state.cpp`](../vortex_yasmin_utils/src/hello_state.cpp).
   The class already compiles and returns `SUCCEED` — you're filling in the
   log line. Signature you're implementing:
   ```cpp
   std::string HelloState::execute(yasmin::Blackboard::SharedPtr blackboard) override;
   ```
   One line is enough:
   ```cpp
   YASMIN_LOG_INFO("Hello from the learning period FSM!");
   ```
   **Verify:** `colcon build --packages-select vortex_yasmin_utils` succeeds.
   (This will succeed even before step 2 below — the file isn't part of the
   build yet, so nothing actually compiles it. That's expected; step 2 is
   what turns it on.)

2. **Wire `hello_state.cpp` into the build.**
   File: [`mission/vortex_yasmin_utils/CMakeLists.txt`](../vortex_yasmin_utils/CMakeLists.txt),
   look for the `# TODO(member)` comment right above `add_library(vortex_yasmin_utils`.
   A `.cpp` file that isn't listed in `add_library()`/`add_executable()`
   never gets compiled, no matter how correct its header is — CMake has no
   idea it exists otherwise. Add `src/hello_state.cpp` to that list
   (alphabetically, next to its neighbors).
   `package.xml` does **not** need a new `<depend>` for this — `HelloState`
   only uses `yasmin`/`yasmin_ros`, which `vortex_yasmin_utils/package.xml`
   already depends on. You only need to add a `<depend>` line when a state
   uses a ROS package that isn't declared yet (e.g. if you'd used
   `nav_msgs`, you'd add `<depend>nav_msgs</depend>` there too).
   **Verify:** `colcon build --packages-select vortex_yasmin_utils` still
   succeeds — this step has no visible effect yet on its own. It matters for
   step 4 below: skip it, and linking `learning_period_fsm` will fail with
   `undefined reference to vortex_yasmin_utils::HelloState::HelloState()`.

3. **Construct the 5 states.**
   File: [`src/build_state_machine.cpp`](src/build_state_machine.cpp), `TODO(member) 1 of 3` and
   `TODO(member) 2 of 3`. Construct one `HelloState` and four
   `WaypointGoalState`s, pulling each `WaypointGoal` off the blackboard by
   key (`"waypoint_1_goal"` .. `"waypoint_4_goal"` — already loaded for you,
   see `src/blackboard.cpp`). For a real example of this exact
   pull-off-blackboard-and-construct pattern, read
   [`subsea_docking_fsm/src/build_state_machine.cpp`](../tacc/subsea_docking/subsea_docking_fsm/src/build_state_machine.cpp)
   (search for `dock_config_waypoint_goal`).
   **Verify:** it compiles (you'll have unused-variable warnings until step 4 — that's expected).

4. **Wire the transition table.**
   Same file, `TODO(member) 3 of 3`. Use `sm->add_state(name, state, {outcomes})`
   to build the chain shown in the diagram above. Remember: the state named
   in your *first* `add_state()` call becomes the machine's entry point.
   Delete the `"PLACEHOLDER"` state once your real states are wired in.
   **Verify:** `colcon build --packages-select learning_period` succeeds with
   no warnings about unused `hello`/`wp1`../`wp4` variables, and no
   `undefined reference` errors at link time (if you see one for
   `HelloState`, go back to step 2).

5. **Run it.**
   With the simulator, waypoint manager, and viewer running (see above),
   launch your FSM and confirm it visits all 4 waypoints and ends on
   `SUCCEED`. Watch it in the viewer at http://localhost:5000/.

## 5. Concepts

If you're new to ROS 2 / YASMIN, here's the minimum vocabulary you need:

- **State** — one node in the state machine's graph. Has an `execute()`
  method that does some work and returns an **outcome** string.
- **Outcome** — the string a state returns from `execute()` (e.g.
  `"SUCCEED"`, `"ABORT"`). It's just a label the state machine uses to
  decide where to go next.
- **Transition** — a mapping from `(state, outcome) -> next_state`, given as
  the third argument to `add_state()`. This is the actual "wiring" of the
  machine.
- **Blackboard** — a shared key-value store (`yasmin::Blackboard`) passed to
  every state's `execute()`. It's how states that run at different times
  share data without knowing about each other directly. In this exercise
  it's used to hand the 4 loaded `WaypointGoal` structs from
  `initialize_blackboard()` to `build_state_machine()`.
- **ActionState** (`yasmin_ros::ActionState<ActionT>`) — a state that wraps
  a ROS 2 action client: it sends a goal on `execute()`, blocks until the
  action server returns a result, and maps that result to an outcome.
  `WaypointGoalState` is one of these, specialized for
  `vortex_msgs::action::WaypointManager`.
- **Why use `WaypointGoalState` instead of writing an action client
  yourself?** Because a hand-rolled `rclcpp_action` client is ~80 lines of
  goal handles, futures, and callback boilerplate — and this repo already
  has that logic written, tested, and used in 3 other missions. The
  point of this exercise is composition: you decide *which* waypoints to
  visit and *in what order*, not *how* to talk to the action server.

## 6. Inspecting the running FSM

The **YASMIN viewer** shows your state machine's graph and current state
live in a browser:

```shell
ros2 run yasmin_viewer yasmin_viewer_node
# open http://localhost:5000/ and select LEARNING_PERIOD_FSM
```

Useful commands while debugging:

```shell
# Is your FSM node up, and what does it see?
ros2 node list
ros2 node info /learning_period_fsm

# Is the waypoint_manager action server actually there?
ros2 action list
ros2 action info /<namespace>/waypoint_manager

# Watch goals/results/feedback going to the action server directly
ros2 action send_goal /<namespace>/waypoint_manager \
  vortex_msgs/action/WaypointManager \
  "{waypoints: [{pose: {position: {x: 1.0, y: 0.0, z: -2.0}}}], convergence_threshold: 0.3, persistent: false}"

# Is odometry actually flowing? (WaypointManager can't converge without it)
ros2 topic hz /<namespace>/nucleus/odom
ros2 topic echo /<namespace>/nucleus/odom --once
```

## 7. Troubleshooting

- **Goal rejected immediately.** Check `ros2 action list` — if
  `/<namespace>/waypoint_manager` isn't listed, the waypoint manager node
  (Terminal 2 above) isn't running or is on a different namespace than your
  FSM's launch file resolved. Check the `action_servers.waypoint_manager`
  parameter your node actually got: `ros2 node info /learning_period_fsm`.
- **No odom / never converges.** `WaypointGoalState` will hang waiting for
  the action to complete if the vehicle never reaches the goal. Confirm
  odometry is publishing (`ros2 topic hz .../odom`) and that your waypoint
  poses in `config/waypoints.yaml` are actually reachable from the sim's
  spawn point — the 4 defaults here form a 3×3 m square at -2.0 m depth
  around the origin, which may not match your sim world.
- **State machine ends immediately with no waypoints visited.** You
  probably still have the `"PLACEHOLDER"` state in
  `build_state_machine.cpp` — it logs a warning and immediately returns
  `SUCCEED`. Delete it once your real states are added.
- **Link/build errors mentioning `HelloState` undefined reference.** You
  skipped (or mistyped) task step 2 — `src/hello_state.cpp` isn't in
  `vortex_yasmin_utils/CMakeLists.txt`'s `add_library()` sources list yet,
  so the symbol was declared in the header but never compiled anywhere.
  Same failure mode applies to any `.cpp` you add in the future: a file not
  listed in `add_library`/`add_executable` simply doesn't exist to the
  linker, and removing `install(DIRECTORY include/ ...)` / `ament_export_*`
  lines breaks it for downstream packages the same way.
- **`colcon build` can't find `vortex_yasmin_utils` (or `vortex_msgs`,
  `vortex_utils`) at all.** These come from external repos pulled in via
  [`dependencies.repos`](../../dependencies.repos) — make sure your
  workspace's `src/` actually has them checked out and built, not just this
  repo.

## 8. Done when

- [ ] `colcon build` succeeds with no errors or new warnings.
- [ ] `src/hello_state.cpp` is listed in `vortex_yasmin_utils/CMakeLists.txt`'s
      `add_library()` sources.
- [ ] `HelloState::execute()` logs a message and the FSM's terminal logs show
      it running first.
- [ ] The FSM visits all 4 waypoints in order (visible in the YASMIN viewer
      and/or the drone's actual movement in sim) and ends on `SUCCEED`.
- [ ] The `"PLACEHOLDER"` state is gone from `build_state_machine.cpp`.
- [ ] You can explain, in your PR description, what `SUCCEED`/`ABORT`/`CANCEL`
      mean for this specific machine and what happens on each.

**Stretch goals** (pick 0-3, not required):

1. Loop the 4 waypoints indefinitely instead of ending at `DONE`.
2. Add a state that records `now() - leg_start_time` around each waypoint
   leg and logs it — a first taste of `yasmin::Blackboard` being used for
   more than static config.
3. Instead of sending every `ABORT`/`CANCEL` straight to the terminal
   `ABORT` outcome, retry the failed waypoint exactly once before giving up.

## 9. Links

- YASMIN repo: <https://github.com/uleroboticsgroup/yasmin>
- YASMIN docs (viewer, states, demos): <https://uleroboticsgroup.github.io/yasmin/>
- `yasmin_ros` package index: <https://index.ros.org/p/yasmin_ros/>
- ROS 2 actions concept (Humble): <https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html>
- ROS 2 C++ action client tutorial (Humble): <https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html>
- `WaypointGoalState` (the state you're using):
  [`mission/vortex_yasmin_utils/include/vortex_yasmin_utils/waypoint_goal_state.hpp`](../vortex_yasmin_utils/include/vortex_yasmin_utils/waypoint_goal_state.hpp)
- A real mission FSM to reference for style/patterns:
  [`mission/tacc/subsea_docking/subsea_docking_fsm/`](../tacc/subsea_docking/subsea_docking_fsm/)

## Open questions

Things I (the scaffold author) couldn't verify from this repo alone — ask in
the group channel or update this section once you know:

- ~~**Exact simulator launch command.**~~ Resolved: it lives in `vortex-auv`,
  not `vortex-cv` — see [`launch_drone_sim.sh`](../../../vortex-auv/utility_scripts/launch_drone_sim.sh)
  and the [Environment setup](#3-environment-setup) section above. The
  `--sim`/`--real` flags on the `tmux_*.sh` scripts in
  `perception_setup/scripts/` are unrelated: they only tell the *camera*
  launch files whether to skip the real driver and trust that something else
  is already publishing on those topics (see e.g. the `sim` arg in
  `perception_setup/launch/mission/pipeline_inspection/pipeline_inspection_front_camera.launch.py`).
- **Goal frame.** Every other FSM in this repo sends `WaypointManager`
  goals in the `{namespace}/odom` frame by convention (poses are an
  unstamped `geometry_msgs/Pose`, so the frame isn't in the message itself
  — it's enforced by the receiving node). `config/waypoints.yaml` assumes
  this; confirm against the actual `waypoint_manager` node if goals land in
  unexpected places.
- **Orientation units (`roll`/`pitch`/`yaw`) in the waypoint YAML.** The
  `vortex_utils` loader docstring's own example uses what look like radians
  (`yaw: 3.14159`), which is what `config/waypoints.yaml` assumes here — but
  a comment in `visual_inspection_fsm/config/landmark_convergence.yaml`
  claims the loader treats yaw as **degrees**. These two sources disagree.
  Confirm empirically (send one waypoint, check which way the drone turns)
  before trusting the exact headings in `config/waypoints.yaml`.
- **Whether anything needs to "enable" DP/autonomous mode before
  `WaypointManager` goals are accepted.** No such service call appears
  anywhere in this repo's mission code — either none is needed, or it's
  handled inside the external `waypoint_manager` node itself.
