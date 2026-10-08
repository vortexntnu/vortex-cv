# robosub_mission

Runner and trees for RoboSub. The nodes are in `vortex_bt_nodes`, the map in
`landmark_slam`.

```
src/main.cpp          loads trees/root.xml, ticks the main tree (10 Hz),
                      publishes it to Groot2 (port 1667)
trees/root.xml        Main: Setup, then every task in course order, skipped on
                      failure or timeout; Setup, transits, SafeStop;
                      Test<Task> to run one task alone
trees/<task>.xml      one subtree per task, BehaviorTree ID = task name
```

Launch: `perception_setup/launch/mission/robosub/robosub_mission.launch.py`
(`config:=pool|sim`, `main_tree:=Main|TestGate|...`).
Values: `perception_setup/config/mission/robosub/mission.yaml` (pool) and
`mission_sim.yaml` (the simulator course), same keys, read as `{key}`.

## The course (Main)

1. **Setup**: load the values, wait for killswitch off and autonomous mode,
   start the run (`mission/wipe` and the coin flip into landmark_slam), dive
   to travel depth.
2. **Gate**: find our role's panel, through its opening
   (`gate_<role>_entrance` → `gate_<role>_exit`).
3. **Torpedo**: around the slalom, find the board, stand off in front of it
   facing it (firing is left out until the launcher has an interface).
4. **Bins**: find the bin rig, hover above it (markers left out).
5. **Octagon**: find the table, go above it and surface inside the octagon.
6. **ReturnHome**: around the slalom, back through our opening, to `start`.

Each task has the same shape: depth, search point, search until the map has
it, do it, leave. The search point is an offset from where the object should
be (`prior_<class>`, landmark_slam's frame from the prior map), e.g.
`GoToFrame frame="prior_torpedo_board" offset="{torpedo_search}"` with
`torpedo_search: "-3.0;0;2.5;0"` (3 m before it, at 2.5 m depth); the move to
the object itself uses where it is (`torpedo_board`). So a new pool only
changes landmark_slam's `prior_map.yaml`; `mission.yaml` keeps how the tasks
are done. A task that fails or runs out of its budget
(`<task>_budget_ms`) is skipped; the run goes on. The slalom has no tree yet:
the transits go around it.

## Running it in the simulator

```bash
src/vortex-auv/utility_scripts/launch_drone_sim.sh --scenario robosub --low-res --keyboard-joy false --detach
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh            # realistic dummy, field of view
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh --tree TestGate   # one task
```

Foxglove layout: `src/vortex-auv/mission/landmark_slam/foxglove/landmark_slam.json`.

## Owners

The trees here are a working first version; each owner takes their task
further (actuators, roles, the real course numbers).

| Task | Owner |
|---|---|
| Gate | Amélie |
| Slalom | André |
| Bins | Karol |
| Torpedo | Johannes |
| Octagon | Ashish |

## Adding a task

A task tree is done when every move and wait has a `Timeout`, its numbers
come from the mission config, a failure never stops the run, and it succeeds
in the sim. Add it with `<include path="<task>.xml"/>` and a
`<SubTree ID="<Task>" _autoremap="true"/>` in `root.xml`, and a `Test<Task>`.
A target that needs geometry (an opening, a pass point) is a frame in
landmark_slam, not a node.
