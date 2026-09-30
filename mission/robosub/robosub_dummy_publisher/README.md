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

The icon positions are read from the board textures (`_TORPEDO_ICON_OFFSETS`,
per board version). The icon -> opening offsets they imply are
`rules.torpedo_targets_from_icons` in landmark_server's `sim.yaml`; keep them
in sync.

**Field of view**: `use_field_of_view: true` publishes only what the cameras
could see from the pose on `odom_topic` (front: `front_range_m`,
`front_half_fov_deg`; down: `down_radius_m` below the vehicle). Positions are
then assumed to be in the frame of that odometry.

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
landmark's `offset` within it -- comes from the course geometry the sim
actually loads, `vortex-stonefish-sim/stonefish_sim/data/object_files/robosub_course/*.obj`,
run through the same world transform every course body gets in
`objects/robosub_course.scn` (`xyz="4.0 -1.57 3.432" rpy="pi 0 pi/2"`, the
octagon at z 1.8):

    X_w = Y_b + 4.0     Y_w = X_b - 1.57     Z_w = 3.432 - Z_b

That is the Stonefish world frame (X forward, Y right, Z down, surface at
Z=0), which is also the sim's `nautilus/odom` frame.

- **gate, slalom**: each colour's `.obj` merges several poles into one mesh,
  so the poles are that mesh's connected components. The slalom is the sim's
  version, turned 90 deg so each white-red-white set lies across the course
  and the three sets follow each other along X. All slalom poles, white and
  red, are 0.938 m tall and centred at Z 2.624.
- **torpedo board**: `base_pose` is the centre of the board's front face
  (X 17.043, facing the vehicle along -X). The openings are the grey discs
  behind the decal's cut-outs (`_TORPEDO_OPENING_OFFSETS`). The icons are the
  centres of each icon's bounding box in `Task4_ver1.png` / `Task4_ver2.png`,
  mapped through the decal's UVs onto the face (`_TORPEDO_ICON_OFFSETS`).
  Which version is up is drawn with the sim's seed, like the gate and bins.
- **bins**: `BIN_UNCLASSIFIED` (front camera) is each bin's centre; the role
  icon (down camera) is on the bin's floor. The rig is tilted, so the bins
  are at different depths.
- **octagon, table**: the plates, baskets and items are the centres of their
  meshes. The jars and containers are dynamic in the sim and settle about
  1 cm after spawning.

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
