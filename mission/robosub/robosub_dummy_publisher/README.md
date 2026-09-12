# RoboSub Dummy Publisher

Publishes fake `vortex_msgs/LandmarkArray` detections for RoboSub course
elements (gate, slalom, torpedo board, bin), so `landmark_server` and mission
logic (polling/converging on a landmark) can be exercised without a running
perception stack or a simulator.

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

- `tasks`: comma-separated subset of `gate`, `slalom`, `torpedo_board`, `bin`
  (default: all four) -- e.g. bring up only `torpedo_board` while testing the
  torpedo mission state, or only `slalom` while tuning line-up guidance.
- `seed`: see above.
- `rate`: publish rate in Hz (default 2.0).
- `frame_id` / `position_noise_std` / `topic`: set in
  `config/robosub_dummy_publisher_params.yaml` -- `frame_id` must be
  resolvable (directly, or via tf) to `landmark_server`'s `target_frame`; set
  it to that same frame to skip tf entirely.

## Landmark types

Added to `vortex_msgs` alongside the existing ARUCO/PIPELINE/VALVE ones:
`LandmarkType.GATE / SLALOM_PIPE / TORPEDO_BOARD / BIN`, with matching
`LandmarkSubtype` values for role, pipe colour, and (for the torpedo board)
combined size+role per opening. See `vortex-msgs/msg/LandmarkType.msg` and
`LandmarkSubtype.msg`.

## Not yet covered

Task 5 (octagon / return) isn't implemented -- `robosub_icons.json` already
has its role groups (`octagon_plates`, `basket_floors`, `table_items`) so
extending `course_layout.py` with an `octagon` task follows the same pattern
as `bin`.
