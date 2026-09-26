# robosub_mission

Behavior tree runner and trees for RoboSub. The nodes are in
`vortex_bt_nodes`. Launch and config files are in `perception_setup`.

```
src/main.cpp          loads trees/root.xml and ticks it
trees/root.xml        Main → Mission (task order) → SafeStop
trees/<task>.xml      one subtree per task

perception_setup/launch/mission/robosub/robosub_mission.launch.py
perception_setup/config/mission/robosub/mission.yaml
```

```bash
ros2 launch perception_setup robosub_mission.launch.py
```

Parameters: `tree_file`, `tick_rate_hz` (10), `mission_config`.

## Tasks

| Task | Owner | Tree |
|---|---|---|
| Gate | Amélie | `trees/gate.xml` |
| Slalom | André | `trees/slalom.xml` |
| Bins | Karol | `trees/bins.xml` |
| Torpedo | Johannes | `trees/torpedo.xml` |
| Octagon | Ashish | `trees/octagon.xml` |

Every task tree:
- has a `Timeout` around every move and every wait;
- takes its numbers from `mission.yaml` (`{key}`), and moves between tasks
  in the course frame (`GoToCourse`, `MoveCourse`);
- has a fallback for every way the task can fail;
- is done when it succeeds in the sim with seeds 1, 2 and 3.

To add a task: create `trees/<task>.xml` with the `BehaviorTree ID` set to
the task name, add `<include path="<task>.xml"/>` and a `SubTree` to
`root.xml`, and put its values in `mission.yaml`.

### Gate (Amélie)

Go through the gate on the half with our role.

1. `SetDepth {gate_depth}`.
2. Find the gate with `LandmarkKnown GATE/GATE_WHOLE`, or else `Search ROTATE_STEPS`.
3. `SelectGatePanel` → `{role}`, `{gate_side}`.
4. `ApproachLandmark` to 2 m in front of the panel, then `MoveRelative` 2.5 m forward.
5. If the gate is not found: set `{role}` and `{gate_side}` from the yaml,
   then `MoveCourse dx=6`.
6. Style: `Turn 90°` repeated `{style_yaw_turns}` times.

Writes `{role}` and `{gate_side}`, which the other tasks use.

### Slalom (André)

Pass all three layers on the `{gate_side}` side of the red pipe.

1. `SetDepth {slalom_depth}`.
2. Find pipes with `LandmarkKnown`, or else `Search SCAN_ARC`.
3. Three times: `MatchPipes` (retried a few times), `GoTo {gap_pose}`,
   `MoveRelative` 1 m forward, `RecordLayer`. If a layer fails,
   `MoveCourse dx=2`.
4. If no pipes are found at all: `MoveCourse dx=6`.

Writes `{passed_red_ids}` and `{slalom_layers}`. See `~/slalom_guide.md`.

### Bins (Karol)

Drop one marker in each of the two bins with our role.

1. `GoToCourse {bins_area_x/y}`, `SetDepth {bins_search_depth}`.
2. Find bins with `LandmarkKnown BIN/ANY`, or else `Search LAWNMOWER`.
3. Twice: `SelectLandmark {bin_subtype}`, skipping used bins, with any bin
   as the fallback. Then `ApproachLandmark` to 0.7 m above the bin, then
   0.4 m with `tool_frame="dropper_link"`. Then `DropMarker` and
   `MarkUsed {used_bins}`.

### Torpedo (Johannes)

Fire through the large opening, then the small one, both with our role, from
0.46 m or more.

1. `GoToCourse {torpedo_area_x/y}`, `SetDepth {torpedo_depth}`.
2. Find the board with `LandmarkKnown TORPEDO_BOARD/TORPEDO_BOARD_WHOLE`,
   or else `Search SCAN_ARC`.
3. `ApproachLandmark` to 2 m in front of the board, then `CommitEstimate`.
4. For each opening: pick our role's opening, then the other role's
   opening of the same size, then the board centre. Then
   `ApproachLandmark` to `{fire_distance_m}` with the torpedo's
   `tool_frame` and `freeze="true"`, then `FireTorpedo`. A failed first
   shot doesn't stop the second.

### Octagon (Ashish)

Surface fully inside the octagon, facing the image with our role.

1. `GoToCourse {octagon_area_x/y}`, `SetDepth {octagon_search_depth}`.
2. Find it with `LandmarkKnown TABLE` or `OCTAGON`, or else
   `Search EXPANDING_SQUARE`.
3. Centre over the octagon with `ApproachLandmark`, `mode=XY_AND_YAW`.
   Then `VerifyInside` (radius 1.35 m, margin 0.3 m), then `Surface`.
   Retry once, and never surface if `VerifyInside` fails.
4. `LandmarkKnown {octagon_image_subtype}`, or else `Search ROTATE_STEPS`.
   Then `LookAtLandmark`.
5. Table items are off while `{octagon_items_enabled}` is false.
6. `SetDepth {travel_depth}` for Return Home.
