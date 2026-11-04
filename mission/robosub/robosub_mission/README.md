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

The vehicle goes to the torpedo board, measures it and stands off from it. Then:

1. Write `trees/<task>.xml` with a tree per task, one file per person so you do not
   get in each other's way, and `include` it in `course.xml`. The tree that is your
   task is called by `Main`. A `Test<Task>` tree runs yours alone.
2. No rebuild is needed for a tree with `--symlink-install`, except for a new file.
3. Start with `--profile none`, then move on to `--profile realistic`. The odometry
   in the simulator is perfect for now, get the task working first.
4. The nodes are in the `vortex_bt_nodes` README, the frames and `Measure` in the
   `landmark_server` README.

The priors in the simulator come from the course itself, so step 1 below is only for
a real pool.

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
