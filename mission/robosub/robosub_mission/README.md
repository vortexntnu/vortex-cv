# robosub_mission

Runner and trees for RoboSub. The nodes are in `vortex_bt_nodes`, the map is
`landmark_server` in vortex-auv.

The task trees are not written yet. `trees/root.xml` is an empty skeleton.
This README gives the plan and hints. Trees built this way have passed gate,
slalom and torpedo in the simulator.

## Files

| File | |
|---|---|
| `src/main.cpp` | Loads `trees/root.xml` and ticks the main tree at 10 Hz |
| `trees/root.xml` | `Main` and the `Test<Task>` trees |
| `trees/<task>.xml` | One subtree per task, to be written |

Launch file and mission values are in `perception_setup`:
`launch/mission/robosub/robosub_mission.launch.py` and
`config/mission/robosub/mission.yaml` (`mission_sim.yaml` for the simulator).

## Owners

| Task | Owner |
|---|---|
| Gate | Amélie |
| Slalom | André |
| Bins | Karol |
| Torpedo | Johannes |
| Octagon | Ashish |

## How a task tree is built

Every task has the same shape:

1. Go to the task's depth.
2. Go to a search point near `prior_<task>`.
3. Search until the object is confirmed in the map.
4. Go to the object's frame and do the task.
5. Leave.

Rules:
- Targets are TF frames from `landmark_server`. A tree never computes
  geometry. If you need a point that is not an object, add a target frame to
  `landmark_server`.
- Numbers come from the mission config as `{key}`, not from the XML.
- Every move and wait has a `Timeout`.
- In `Main`, wrap each task so a failure or timeout skips it and the run
  goes on.
- Add a `Test<Task>` tree to run your task alone.

## Hints per task

**Setup**: load the config, wait for the start signal, dive.

**Gate**: `landmark_server` publishes `<panel>_entrance` and `<panel>_exit`
for each role panel. Find our panel, go to the entrance, then the exit.

**Slalom**: three rows, pass each on the same side of the red pipe as our
half of the gate. You need a frame for the gap in each row; that frame is
not in `landmark_server` yet, see its README. Line up in front of the gap
first. The pipes are out of view while you pass, so decide what the vehicle
should steer on in that moment. Plan for a row that is never found.

**Torpedo**: the board is one object, the openings are not. You need frames
for the openings, see the `landmark_server` README. Stand off in front of
the board first, then aim at each opening and hold still before firing.
Check which way `prior_torpedo` and `torpedo_board` point before choosing
offsets.

**Bins**: the front camera sees the rig and bins without a role. The role
only shows from above, through the down camera. So: find the rig, go above
it, wait for our role's bin, go above that bin, drop.

**Octagon**: find the table, come in under the octagon, surface inside it.
In the simulator the octagon frame is at about 1.6 m depth with plates
hanging down to 2.05 m, and the table top is at 2.7 m. Come in between them
and leave above the frame.

**Return home**: back through our gate opening, then to `start`.

## Things that went wrong in testing

- One long move flipped the vehicle. Keep moves short or split them.
- An offset with the wrong sign put the search point behind the torpedo
  board, and the vehicle drove through it.
- Driving at the octagon frame's depth hit the frame.
- Moves without a heading tolerance waited a long time to converge.

## Running

```bash
src/vortex-auv/utility_scripts/launch_drone_sim.sh --scenario robosub --low-res --keyboard-joy false --detach
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh --tree TestGate
```

Before each run, put the vehicle at the start facing the course and reset
the map:

```bash
ros2 topic pub --once /nautilus/mission/wipe std_msgs/msg/Empty
```

The runner publishes the tree on port 1666 for Groot2 or the VS Code
BehaviorTree Viewer.
