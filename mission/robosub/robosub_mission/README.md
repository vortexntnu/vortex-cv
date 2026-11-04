# robosub_mission

Runner and tree for RoboSub. The nodes are in `vortex_bt_nodes`, the map and
the measuring are `landmark_server` in vortex-auv.

| File | |
|---|---|
| `src/main.cpp` | Loads the tree file and ticks the main tree at 10 Hz |
| `trees/course.xml` | `Main`, and the `include` of each task's file |
| `trees/example.xml` | The smallest task: find, measure, go there |

Launch: `ros2 launch perception_setup robosub_mission.launch.py drone:=<drone> main_tree:=Main`.

## The three layers

| Layer | Who | Holds |
|---|---|---|
| Course | The numbers at the top of your tree, `config/course/<course>/premap.yaml` and `config/<competition>/classes.yaml` in `landmark_server` | Roles, offsets, priors, classes |
| Task | Your tree here | Find, measure, act, leave |
| Stack | `landmark_server`, `vortex_bt_nodes`, control | Matching, graph, measuring, moving. Knows no course |

A new course changes the first layer. A new kind of task changes the second,
and needs the third only if it has to measure something new.

## Getting started

Set up the workspace, see the `vortex-cv` README. Then run the example:

```bash
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh --spawn torpedo --tree Main
```

The vehicle goes to the torpedo board, measures it and stands off from it.

## From a clean branch to your task

**1. Your own tree.** Add `trees/<task>.xml`, one file per person, with your task and a
tree that runs it alone:

```xml
<?xml version="1.0"?>
<root BTCPP_format="4">
  <BehaviorTree ID="MyTask">
    <Sequence>
      <GoToFrame name="find" frame="prior_<task>" offset="x;y;z;yaw_deg" position_tolerance="0.5" orientation_tolerance_deg="15"/>
      <Wait seconds="2"/>
      <Measure class_name="<class>" duration_s="5" accept_fallback="false"/>
      <GoToFrame name="act" frame="<class>/measured" offset="x;y;z;yaw_deg"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="TestMyTask">
    <Sequence>
      <Wait seconds="5"/>
      <SubTree ID="MyTask"/>
    </Sequence>
  </BehaviorTree>
</root>
```

Add `<include path="<task>.xml"/>` to `course.xml`. Classes are in
`config/robosub/classes.yaml` in `landmark_server`.

**2. Run it.** A new file needs one build, after that a tree only needs a new run:

```bash
colcon build --symlink-install --packages-select robosub_mission
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh --spawn <task> --tree TestMyTask
```

There is no spawn point for the gate, it is near the start, so leave out `--spawn`.

Run it again from the `mission` window. Start with `--profile none`, then use
`--profile realistic`. The odometry in the simulator is perfect for now, so get the
task working first.

**3. Check the frame before you drive on it.** Look at it in Foxglove, or:

```bash
ros2 run tf2_ros tf2_echo nautilus/odom nautilus/<class>/measured
```

**4. A measurement of your own, if the task needs one.** `Measure` on a class gives one
object. See "A measurement for your task" in the `landmark_server` README. It adds a
name you can use as `class_name`, and frames like `<name>/measured/<child>` to go to.

**5. Into the course.** Call your tree from `Main` in `course.xml`, and open a pull
request against `rework/landmark-members`.

The nodes are in the `vortex_bt_nodes` README, the frames in the `landmark_server`
README. The priors in the simulator come from the course itself, so step 1 below is only
for a real pool.

## Targets

There are no target frames to set up in `landmark_server`. A target is a frame and an
offset in your tree:

| Step | What you do |
|---|---|
| Find | `GoToFrame` on `prior_<task>` or `<class>`, with an offset that puts the object in view |
| Measure | `Measure class_name="<class>"` sets `<class>/measured` |
| Target | `GoToFrame` on `<class>/measured`, with the offset for where the task needs the vehicle |
| Aim | Add `tool_frame` to put the tube or dropper on the target instead of the vehicle's middle |

```xml
<Measure class_name="<class>" duration_s="5" accept_fallback="false"/>
<GoToFrame name="stand off" frame="<class>/measured" offset="2;0;0;180"/>
<GoToFrame name="aim" frame="<class>/measured" offset="1;0;0;180" tool_frame="torpedo_tube"/>
```

You only need something outside the tree when:
- There is a new object: add the class in `config/robosub/classes.yaml`. The objects on
  the RoboSub course are there already.
- There is a new pool: the priors are in `config/course/<course>/premap.yaml`. In the
  simulator they come from the course.
- The task needs more than one object: write your own measurement, see "A measurement
  for your task" in the `landmark_server` README. It can add frames like
  `<name>/measured/left`, and you drive to them with `GoToFrame` the same way.

## Measure on each task

| Task | Classes | What `Measure` on a class gives you |
|---|---|---|
| Gate | `gate_search_rescue`, `gate_survey_repair` | One panel. The yaw of one small panel is noisy, and you may want it from more than that |
| Slalom | `slalom_pipe_red`, `slalom_pipe_white` | One pipe. Rows, gaps and the direction of the slalom are a measurement of your own |
| Torpedo | `torpedo_board`, `torpedo_opening_*` | The board or one opening. Measure the opening again from close up before you fire |
| Bins | `bin_search_rescue`, `bin_survey_repair`, `bin_structure` | The rig or one bin. The bins are seen by the down camera, so measure them from above |
| Octagon | `octagon`, `table` | One object |

Use a class on its own where you can. The sizes and positions on the course are in
the handbook and `objects/robosub_landmarks.scn` in `vortex-stonefish-sim`.

## From test day to a running task

| Step | What you do | Where |
|---|---|---|
| 1. Priors | Drive the course as an ROV, measure each task from the GUI, drag what looks wrong, send it to the vehicle | `competition_map_gui`, saved to `config/course/<course>/premap.yaml`. See "Making the prior map" in the `landmark_server` README |
| 2. Find | Go to `prior_<task>` with an offset that puts the object in view. A loose tolerance is enough here | `GoToFrame` in your tree |
| 3. Measure | Hold still and add `Measure` for the class. Look at the frame and at the numbers in the log | `Measure` in your tree. A new class goes in `config/<competition>/classes.yaml`, a new kind of measurement in `landmark_server` |
| 4. Target | `GoToFrame` on `<class>/measured`, with the offset that places the vehicle where the task needs it | Your tree |
| 5. Test | Run one task on its own, then all of `Main` | `main_tree:=Test<Task>`, `main_tree:=Main` |

- Start at the same marked spot every run: the priors are relative to the start.
- Offsets are `x;y;z;yaw_deg` in the target's frame: +X forward, +Y right, +Z down.
  An object with a front has +X out of the front, others +X from the vehicle to
  the object, so look at the frame before you trust an offset.
- A new pool changes step 1 and the numbers in the tree, not the code.
- Tune one thing at a time, and keep a bag of the runs: a bad frame can be looked
  at afterwards.

## How a task is built

1. Find: go to a rough frame (`prior_<task>` or the class's last sighting).
2. Measure: hold still and call `Measure`. The result is a frame in odom,
   good for a short while.
3. Act: `GoToFrame` with an offset from the measured frame.
4. Leave: get clear of the structure at the height you came in at, and only
   then change depth.

Rules the runs taught:
- Do not turn near pipes or structures. A body 0.9 m long sweeps a lot.
- No diagonals near structures: `right_angles` (default), with `sideways_first` to line up before going through something, and `face_travel="false"`.
- Measure again close to the task. A measurement from far away is a start.
- `prior_*`, `start` and `*/measured_in_map` are in map and go with the graph,
  so they are the frames for long legs and for coming home. `*/measured` is in
  odom and is the frame for precise moves.

## Not done

- Torpedo and markers exist in the simulator only, the drone has no driver for them
  yet. Where they leave the vehicle in the simulator is a guess until they are mounted.
- Gate style points, path markers and the pinger are not used.
- Slalom, gate and the other tasks: yours.
