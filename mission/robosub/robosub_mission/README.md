# robosub_mission

Behavior tree for the RoboSub course (BehaviorTree.CPP v4): the tree runner
and the trees. The nodes are in [`vortex_bt_nodes`](../../vortex_bt_nodes),
including who writes which node and in what order; the task trees and who
owns them are below. Launch and config live in
`perception_setup`, as for the TACC tasks.

```
robosub_mission/
├── src/main.cpp                 Registers the vortex_bt_nodes nodes, loads
│                                trees/root.xml and ticks it until it finishes
└── trees/
    ├── root.xml                 Main (safety, then Mission; SafeStop on failure),
    │                            Mission (task order), SafeStop
    └── <task>.xml               One subtree per task (gate.xml, slalom.xml, ...)

perception_setup/launch/mission/robosub/robosub_mission.launch.py
perception_setup/config/mission/robosub/mission.yaml
```

## Tasks

Each task is one subtree in `trees/<task>.xml`, owned by one person, who
also writes nodes in `vortex_bt_nodes` (see its README for the node
specs). The tree is written last, against the node names and ports in that
README; it runs once the nodes it uses are merged.

Rules for every task tree:

- The subtree's `ID` is the task name; it is run inside a `TaskSlot` in
  `root.xml`, which handles the time budget, so the tree itself never waits
  forever: every move and every wait sits in a `Timeout`.
- Numbers come from `mission.yaml` as `{keys}` (depths, areas, distances),
  not hard-coded in the XML. New keys are added to the yaml with a comment.
- Positions between tasks use the course frame (`GoToCourse`,
  `MoveCourse`), never fixed odom x/y.
- Every way the task can fail has a branch in the tree (a search, a
  fallback, a blind pass or giving up), listed in a table in the tree's
  header comment.
- Done = the tree loads (`ros2 launch perception_setup robosub_mission.launch.py`
  with the task in `Mission`), and the task succeeds in the simulator
  (`tmux_robosub_sim.sh`) three runs in a row with seeds 1, 2 and 3.

| Task | Owner | Tree |
|---|---|---|
| Gate | Amélie | `trees/gate.xml` |
| Slalom | André | `trees/slalom.xml` |
| Bins | Karol | `trees/bins.xml` |
| Torpedo | Johannes | `trees/torpedo.xml` |

### Gate (Amélie)

**Goal:** through the gate on the half with our role, and leave the role
and side on the blackboard for the rest of the run.

The tree must:
1. `SetDepth` to `{gate_depth}`.
2. Find the gate: `ReactiveFallback` of `LandmarkKnown type="GATE"
   subtype="GATE_WHOLE"` (→ `{gate_id}`) and `Search pattern="ROTATE_STEPS"`.
3. `SelectGatePanel` → `{gate_panel_id}`, `{role}`, `{gate_side}`.
4. `ApproachLandmark` 2 m in front of the panel, facing it, then
   `MoveRelative` 2.5 m forward through the gate.
5. Blind fallback if the gate is not found: set `{role}` from
   `{preferred_role}` and `{gate_side}` from `{default_gate_side}`
   (`Script`), then `MoveCourse dx="6.0"`.
6. Optional style: `Turn relative_deg="90"` repeated `{style_yaw_turns}`
   times, skipped when 0.

Must handle: gate not seen, only one panel seen, the panel lost close up,
the whole task failing (retried once from `root.xml`).
Writes: `{role}`, `{gate_side}` (read by Slalom, Bins, Torpedo).
Yaml: `gate_depth`, `preferred_role`, `default_gate_side`, `style_yaw_turns`.

### Slalom (André)

**Goal:** through all three layers on the same side of the red pipe as the
gate half (`{gate_side}`), at the right depth.

The tree must:
1. `SetDepth` to `{slalom_depth}` (between the pipe ends).
2. Find pipes: `LandmarkKnown` red or white ahead, else `Search
   pattern="SCAN_ARC"`.
3. Three times (`Repeat`): if the next red pipe is not already in the map
   1–3 m ahead, scan; `MatchPipes` (retried a few times, 100 ms apart, so
   the map can settle) → `{gap_pose}`, `{red_id}`; `GoTo {gap_pose}`
   (POSITION_AND_YAW); `MoveRelative` 1 m forward; `RecordLayer`.
4. A layer that cannot be solved: `MoveCourse dx="2.0"` (blind layer),
   then the next layer.
5. Nothing found at all: `MoveCourse dx="6.0"` past the slalom.

Must handle: no pipes seen, a layer with only red + one white, the same red
pipe matched twice (`{passed_red_ids}`), a layer that cannot be solved.
Reads: `{gate_side}`. Writes: `{passed_red_ids}`, `{slalom_layers}` (used
by AvoidSlalom in Return Home).
Yaml: `slalom_depth`. Background: `~/slalom_guide.md`.

### Bins (Karol)

**Goal:** one marker in each of the two bins with our role.

The tree must:
1. `GoToCourse` to `{bins_area_x}`, `{bins_area_y}` at `{travel_depth}`,
   in a `ForceSuccess` (search from where the vehicle ends up if not
   reached).
2. `SetDepth` to `{bins_search_depth}`; find bins: `LandmarkKnown
   type="BIN" subtype="ANY"`, else `Search pattern="LAWNMOWER"`.
3. Twice (`Repeat`, `{marker_index}` 0 then 1), each in a `ForceSuccess`:
   - pick a bin: `SelectLandmark` with `{bin_subtype}` not in
     `{used_bins}`, else any unused bin, else any bin;
   - `ApproachLandmark` 0.7 m above it, then 0.4 m with
     `tool_frame="dropper_link"` (the fine step in a `ForceSuccess`, so the
     marker is dropped from the coarse position if it times out);
   - `DropMarker index="{marker_index}"` (one retry); `MarkUsed` into
     `{used_bins}`.

Must handle: area not reached, no bins seen, the role of the bins unknown,
the bin lost close up, the dropper not answering.
Reads: `{bin_subtype}` (from `ResolveRole`, run after the gate in
`root.xml`). Writes: `{used_bins}`.
Yaml: `bins_area_x`, `bins_area_y`, `travel_depth`, `bins_search_depth`.

### Torpedo (Johannes)

**Goal:** a torpedo through the large opening, then the small one, both
with our role, from at least 0.46 m.

The tree must:
1. `GoToCourse` to `{torpedo_area_x}`, `{torpedo_area_y}` (`ForceSuccess`),
   `SetDepth` to `{torpedo_depth}`.
2. Find the board: `LandmarkKnown type="TORPEDO_BOARD"
   subtype="TORPEDO_BOARD_WHOLE"` → `{board_id}`, else `Search
   pattern="SCAN_ARC"`.
3. `ApproachLandmark` 2 m in front of the board, facing it; then
   `CommitEstimate` (Timeout 15 s) so the board pose is steady.
4. A `FireAtOpening` subtree, used twice (large with `side="left"`, then
   small with `side="right"`), each in a `ForceSuccess`:
   - opening: `LandmarkKnown` with our role's subtype, else the other
     role's opening of the same size, else the board centre;
   - `ApproachLandmark` to `{fire_distance_m}` from the opening with
     `tool_frame` set to that torpedo and `freeze="true"`;
   - `FireTorpedo` (one retry), `Wait seconds="2"`.

Must handle: board not found, the board pose moving, our opening not in the
map, the opening lost close up, the first shot failing (the second is still
tried).
Reads: `{torpedo_large_subtype}`, `{torpedo_small_subtype}` and their
`_other` variants (from `ResolveRole`).
Yaml: `torpedo_area_x`, `torpedo_area_y`, `torpedo_depth`,
`fire_distance_m`.

## Adding a task

1. `trees/<task>.xml`: a `<root BTCPP_format="4">` with one `BehaviorTree`
   whose `ID` is the task name.
2. In `trees/root.xml`: `<include path="<task>.xml"/>` at the top and
   `<SubTree ID="<Task>" _autoremap="true"/>` in `Mission`.
3. Its values in `perception_setup/config/mission/robosub/mission.yaml`.

Nodes do one thing and return SUCCESS or FAILURE; retries, timeouts,
searches and fallbacks are written in the XML.

## Run

```bash
ros2 launch perception_setup robosub_mission.launch.py
```

Parameters: `tree_file` (default: the installed `trees/root.xml`),
`tick_rate_hz` (10), `mission_config` (the yaml above). The runner spins the
ROS node between ticks, so action and topic callbacks of the nodes run
there. Logging with spdlog.

## Status

No tasks yet. The tree runs `Main` → `Mission` and finishes with SUCCESS.
Requires `ros-humble-behaviortree-cpp` (v4).
