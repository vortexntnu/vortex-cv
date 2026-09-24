# RoboSub Dummy Publisher

Publishes fake `vortex_msgs/LandmarkArray` detections for RoboSub course
elements (gate, slalom, torpedo board, bin), so `landmark_server` and mission
logic (map, scenarios) can be exercised without a running perception stack or
a simulator.

## What it publishes

3D positions **without orientation** (identity quaternion, rotation variance
1000 = "no orientation"). `landmark_server` derives the rest:

| Task | Published | Derived by landmark_server |
|---|---|---|
| gate | `GATE_WHOLE`, the two panels (`GATE_SURVEY_REPAIR`, `GATE_SEARCH_RESCUE`) and the three posts (`GATE_POLE_EDGE` x2, `GATE_POLE_MIDDLE`) | gate yaw (locked), synthetic gate, course frame |
| slalom | `SLALOM_PIPE_WHITE` / `_RED` | (map limits, memory) |
| torpedo_board | `TORPEDO_BOARD_WHOLE` and the four **icons** (`TORPEDO_ICON_*`) | board yaw, version, `TORPEDO_TARGET_*` openings |
| bin | `BIN_STRUCTURE` (the rig), `BIN_UNCLASSIFIED` (front camera) and the role bins (`BIN_*` role, down camera) | roleless duplicate hidden |
| octagon | `OCTAGON_WHOLE` (surface) and the four plate images (`OCTAGON_IMAGE_REPAIR/RESCUE/SEARCH/SURVEY`, drawn per run over the plates) | octagon at the surface |
| table | `TABLE_WHOLE` (table top, 0.7 m above the floor), the two baskets (`TABLE_BASKET_*`, down camera) and the four items (`TABLE_ITEM_*` on the jars/containers, down camera) | (memory) |

The icon positions are the openings minus `_TORPEDO_ICON_TO_HOLE`, the same
numbers as `rules.torpedo_targets_from_icons` in landmark_server's config
(keep them in sync; both are placeholder values until measured on our board).

**Field of view**: `use_field_of_view: true` publishes only what the cameras
could see from the pose on `odom_topic` (front: `front_range_m`,
`front_half_fov_deg`; down: `down_radius_m` below the vehicle). Positions are
then assumed to be in the frame of that odometry.

## Tests

- `test/test_course_layout.py`: the layout is consistent with landmark_server's rules
- `test/test_end_to_end_map.py`: dummy -> landmark_server -> `object_map` (gate yaw, openings within 5 cm, bin roles, pipes)
- `test/test_unstable_map.py`: the same chain with unstable detections (`profile:=unstable`); the map must stay stable
- `test/test_moving_items.py`: the jars and containers are moved during the run; the map must follow each move with the same id
- `test/test_scenarios.py`: gate, torpedo and bin scenarios of `landmark_targets` against the whole chain with a kinematic fake vehicle

## Known issue: slalom layout

In `course_layout.py` the pipes of each set lie along X (the course direction)
and the three sets are staggered in Y. That looks rotated 90° compared with the
handbook, where each set is a white-red-white row across the course and the sets
follow each other along it. The numbers come from the Stonefish meshes, so this
needs a check against the sim world before the layout is changed here.

## Why seeded

`vortex-stonefish-sim` randomizes which role image (Search & Rescue vs.
Survey & Repair, which bin says Blood vs. Fire, ...) lands on which course
slot, using `random.Random(seed)` over a fixed pool
(`stonefish_sim/metadata/robosub_icons.json`) -- see
`stonefish_sim/launch/robosub_icons.py`. This package draws from the exact
same manifest with the exact same recipe (`course_layout.draw_role_picks`),
so:

- launching with `seed:=<same value>` as the sim's `robosub_icon_seed`
  reproduces the same role assignment the sim actually rendered, letting you
  test `landmark_server` against ground truth that matches a specific replay;
- launching with no seed still gives a reproducible run -- the resolved seed
  is printed to the log the same way `robosub_icons.py` does.

If `stonefish_sim` isn't installed (e.g. testing against real hardware), the
role-dependent landmarks (gate, bin, torpedo_board) fall back to a fixed
default role instead of failing.

## Where the numbers come from

Every position -- both each task's anchor (`Task.base_pose`) and every
landmark's `offset` within it -- comes from the real course geometry in
`vortex-stonefish-sim/stonefish_sim/data/object_files/robosub_course/*.obj`,
run through the same Blender -> Stonefish transform
`tools/import_robosub_course.py` uses to place elements in the world. For
gate and bin, a single mesh bounding-box centre per element was enough. For
slalom, each colour's `.obj` merges every pole into one mesh, so getting
individual pole positions took clustering their vertices by (X, Y) and
matching each white pair to its nearest red pole (see comments in
`course_layout.py`) -- this also corrected an earlier assumption: it's a
sideways weave (three gates spread mainly across Y, each a
white-red-white row along the sub's heading), not gates spaced down the
course. For the torpedo board's target circles, there's no separate cutout
geometry to read off the mesh, so the two decal textures the sim actually
randomizes between (`stonefish_sim`'s manifest pool, `Task4_ver1.png` /
`Task4_ver2.png`) were each analysed pixel-by-pixel to find the four printed
red rings, scaled to metres using the board's handbook-specified
0.61 m x 0.61 m face size (RoboSub 2026 Team Handbook section 3.2.5 + the
design package's `Task04_Deploy.pdf`). The two textures put the rings in the
same position/size (within ~1%) -- only which icon sits next to each ring
differs, so:

- position/size of the four openings is version-independent and just
  hardcoded (`_TORPEDO_CIRCLE_OFFSETS`);
- which physical opening is Search & Rescue vs. Survey & Repair *does* flip
  between versions, so `_torpedo_board_landmarks` looks up
  `role_picks["torpedo_board"]` (which version got drawn) and picks the
  matching `TORPEDO_TARGET_{LARGE,SMALL}_{SEARCH_RESCUE,SURVEY_REPAIR}`
  subtype per opening (`_TORPEDO_ROLE_BY_VERSION`). Same seed-reproducibility
  rules as gate/bin apply here too.

Remaining known approximation: the torpedo circle offsets are relative to
the whole board *mesh's* bounding-box centre (which includes its mounting
stand below the face), not the face's own centre -- if that turns out to be
off, nudge the `z` offsets in `_TORPEDO_CIRCLE_OFFSETS`.

## Usage

```bash
ros2 launch robosub_dummy_publisher robosub_dummy_publisher.launch.py \
  drone:=orca tasks:=torpedo_board,bin seed:=12345
```

- `tasks`: comma-separated subset of `gate`, `slalom`, `torpedo_board`, `bin`,
  `octagon`, `table` (default: all) -- e.g. bring up only `torpedo_board` while testing the
  torpedo mission state, or only `slalom` while tuning line-up guidance.
- `seed`: see above.
- `rate`: publish rate in Hz (default 10.0; landmark_server needs a detector-like rate to confirm tracks).
- `frame_id` / `position_noise_std` / `topic`: set in
  `config/robosub_dummy_publisher_params.yaml` -- `frame_id` must be
  resolvable (directly, or via tf) to `landmark_server`'s `target_frame`; set
  it to that same frame to skip tf entirely.
- `profile`: an extra parameter file `config/robosub_dummy_publisher_<profile>.yaml`
  on top of the defaults. `profile:=unstable` gives a detector that is noisy
  and drops out (see below).

### Unstable detections

To test how robust the map is, the publisher can behave like a bad detector.
All of it is off by default; `profile:=unstable` turns on a moderate set.

| Parameter | Unstable profile | What it does |
|---|---|---|
| `position_noise_std` | 0.05 | Gaussian position noise [m] |
| `detection_probability` | 0.7 | chance a landmark is detected in a frame |
| `frame_drop_probability` | 0.05 | chance the whole message is lost |
| `dropout_rate_per_sec` / `dropout_duration_sec` | 0.05 / [1, 6] | occlusions: a landmark is gone for 1-6 s about every 20 s |
| `outlier_probability` / `outlier_std_m` | 0.02 / 1.5 | a detection far off its landmark |
| `false_positive_rate` / `false_positive_radius_m` | 0.1 / 2.0 | spurious detections per frame, a copy of a real class within the radius (id >= 1000) |
| `noise_seed` | -1 | seed for all of the above; -1 draws a new one each run |

**Moving objects**: the jars and containers on the table are loose (dynamic
bodies in the sim) and get moved during a run. `movable_move_interval_sec`
(0 = off) moves each of them to a random spot within `movable_move_radius_m`
of where it started, on average that often; each move is logged.

`test/test_unstable_map.py` runs this profile against `landmark_server` for a
minute and checks the map against the true layout (ids kept, no duplicates,
no tracks from clutter, positions within 0.5 m, no gaps while occluded). Run it
with `-s` to see a table per class, and with `UNSTABLE_PARAMS_FILE=<yaml>` to
try other settings.

## Landmark types

Added to `vortex_msgs` alongside the existing ARUCO/PIPELINE/VALVE ones:
`LandmarkType.GATE / SLALOM_PIPE / TORPEDO_BOARD / BIN`, with matching
`LandmarkSubtype` values for role, pipe colour, and (for the torpedo board)
combined size+role per opening. See `vortex-msgs/msg/LandmarkType.msg` and
`LandmarkSubtype.msg`.

## Not yet covered

Task 5: the octagon and table are published, but the path markers, the
pinger and the octagon's surfacing area are not. The basket images are paired
with the roles as red cross = Search & Rescue and warning = Survey & Repair
(like the bins); check that against the handbook.

## Viewing in Foxglove

`ros2 launch robosub_dummy_publisher foxglove_helpers.launch.py` publishes the
simulator frames and the detection markers. Use `nautilus/odom_zup` as the
display frame of the 3D panel: Foxglove always draws +Z up, so the NED frames
(`world_ned`, `nautilus/odom`, Z down) look upside down and mirrored.
`odom_zup` is the map frame turned 180 deg about X (X forward, Y left, Z up)
and is only for viewing.
