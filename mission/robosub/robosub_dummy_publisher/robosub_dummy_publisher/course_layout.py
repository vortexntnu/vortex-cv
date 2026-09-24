"""Seeded RoboSub course layout: which dummy landmark goes where.

Randomization mirrors the simulator's role-image draw
(vortex-stonefish-sim/stonefish_sim/launch/robosub_icons.py): the RoboSub Team
Handbook fixes *which* role images exist and *how many* of each, and leaves
only their placement (which slot gets which pool item) open, so each
randomized group is a permutation/choice draw from a fixed pool -- done with
the exact same ``random.Random(seed)`` recipe, reading the exact same
manifest (``stonefish_sim/metadata/robosub_icons.json``). Launch this package
with the same seed as vortex-stonefish-sim's ``robosub_icon_seed`` launch
argument and the dummy landmarks' roles (which bin says Blood vs Fire, which
gate upright says Search & Rescue, which torpedo board version is up and
therefore which physical opening is which role, ...) will agree with
whatever the sim actually rendered.

If stonefish_sim isn't available (e.g. running against real hardware with no
sim installed), the role-dependent landmarks (gate, bin, torpedo_board) fall
back to a fixed default role rather than failing.

The landmarks are what the perception gives: 3D positions without
orientation (the publisher marks the rotation as unknown), the two gate
panels and the whole gate, the icons of the torpedo board instead of its
openings, bins from the front camera without a role plus the role icons seen
by the down camera. landmark_server derives yaw, openings and bin roles from
these parts.

Every position below -- both each ``Task.base_pose`` and every landmark's
``offset`` -- comes from the real course geometry in vortex-stonefish-sim
(data/object_files/robosub_course/*.obj), not guesswork. Each element's
vertices were read directly out of its .obj and run through the same
Blender -> Stonefish transform tools/import_robosub_course.py uses to place
it in the world:

    X_w = Y_b + COURSE_ORIGIN[0]      COURSE_ORIGIN = (4.0, -1.57)
    Y_w = X_b + COURSE_ORIGIN[1]      POOL_FLOOR_Z  = 3.432
    Z_w = POOL_FLOOR_Z - Z_b

world frame: X forward (down the course), Y right, Z down, water surface at
Z=0 -- same convention as stonefish_sim/scenarios/robosub.scn. For gate and
slalom this meant clustering individual pole/upright vertices (each element's
.obj merges several physical poles into one mesh per colour/material, so a
single bounding box isn't enough); for the torpedo board's target circles,
there's no separate cutout geometry to read -- instead the actual decal
textures the sim randomizes between (stonefish_sim's manifest pool
Task4_ver1.png / Task4_ver2.png) were analysed pixel-by-pixel to find the
four printed red rings and convert their image position/size to metres via
the board's handbook-specified 0.61 m x 0.61 m face size. Ring position and
size come out identical between the two versions (within ~1%) -- only which
icon, and therefore which role, sits next to each ring changes. See each
task builder below for the specifics and remaining assumptions.
"""

from __future__ import annotations

import json
import os
import random
from dataclasses import dataclass

from vortex_msgs.msg import LandmarkSubtype, LandmarkType

try:
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )
except ImportError:  # pragma: no cover - ament_index_python always present under ROS 2
    get_package_share_directory = None
    PackageNotFoundError = Exception


Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class Landmark:
    """One dummy detection: a label plus a pose relative to its task's base_pose."""

    label: str
    landmark_type: int
    landmark_subtype: int
    offset: Vec3  # metres, in the task's (unrotated) world axes
    camera: str = "front"  # "front" (stereo) or "down" (mono, sees the floor)
    movable: bool = False  # loose object that can be moved during a run


@dataclass(frozen=True)
class Task:
    """One RoboSub course element and everything on/around it worth detecting."""

    name: str
    handbook: str
    base_pose: Vec3  # metres, Stonefish world frame
    build_landmarks: object  # Callable[[dict[str, str]], tuple[Landmark, ...]]

    def landmarks(self, role_picks: dict) -> tuple[Landmark, ...]:
        return self.build_landmarks(role_picks)


# Role-image filename -> our landmark subtype. The handbook names the two
# overarching mission roles "Search & Rescue" / "Survey & Repair"; the actual
# bin decal filenames (Blood / Fire) are that pair's Task-3-specific skin.
_GATE_ROLE_SUBTYPE = {
    "Task1_SearchRescue.png": LandmarkSubtype.GATE_SEARCH_RESCUE,
    "Task1_SurveyRepair.png": LandmarkSubtype.GATE_SURVEY_REPAIR,
}
_OCTAGON_IMAGE_SUBTYPE = {
    "Task5_Repair.png": LandmarkSubtype.OCTAGON_IMAGE_REPAIR,
    "Task5_Rescue.png": LandmarkSubtype.OCTAGON_IMAGE_RESCUE,
    "Task5_Search.png": LandmarkSubtype.OCTAGON_IMAGE_SEARCH,
    "Task5_Survey.png": LandmarkSubtype.OCTAGON_IMAGE_SURVEY,
}
# Basket floors: the red cross marks the Search & Rescue basket and the
# warning sign the Survey & Repair one (same pairing as the bins: blood for
# Search & Rescue, fire for Survey & Repair).
_TABLE_BASKET_SUBTYPE = {
    "Task5_RedCross.png": LandmarkSubtype.TABLE_BASKET_SEARCH_RESCUE,
    "Task5_Warning.png": LandmarkSubtype.TABLE_BASKET_SURVEY_REPAIR,
}
_TABLE_ITEM_SUBTYPE = {
    "Task5_BandAid.png": LandmarkSubtype.TABLE_ITEM_BANDAID,
    "Task5_Electric.png": LandmarkSubtype.TABLE_ITEM_ELECTRIC,
    "Task5_NutBolt.png": LandmarkSubtype.TABLE_ITEM_NUTBOLT,
    "Task5_Pill.png": LandmarkSubtype.TABLE_ITEM_PILL,
}
_BIN_ROLE_SUBTYPE = {
    "Task3_Blood.png": LandmarkSubtype.BIN_SEARCH_RESCUE,
    "Task3_Fire.png": LandmarkSubtype.BIN_SURVEY_REPAIR,
}


def _gate_landmarks(role_picks: dict) -> tuple[Landmark, ...]:
    # gate__icon_gate_1.obj / gate__icon_gate_2.obj bounding-box centres,
    # relative to the gate task's base_pose (gate__white.obj's centre):
    # gate_1 world (3.977, 0.771, 2.273), gate_2 world (3.977, -0.788, 2.273).
    # The uprights sit ~1.56 m apart and the role image is higher up the
    # gate than the panel-colour centroid (base_pose), hence the -0.445 z.
    slots = (("gate_1", (-0.023, 0.788, -0.445)), ("gate_2", (-0.023, -0.771, -0.445)))
    # The whole gate as the front camera sees it (label "gate"): its centre.
    landmarks = [
        Landmark("gate", LandmarkType.GATE, LandmarkSubtype.GATE_WHOLE, (0.0, 0.0, 0.0))
    ]
    for slot, offset in slots:
        role = role_picks.get(slot)
        subtype = _GATE_ROLE_SUBTYPE.get(role, LandmarkSubtype.GATE_SEARCH_RESCUE)
        landmarks.append(Landmark(slot, LandmarkType.GATE, subtype, offset))
    # The posts, from the vertical clusters of gate__white/red/black.obj
    # (world): outer uprights at Y -1.569 and 1.529 spanning Z 2.07-3.43, and
    # the short post between the openings at Y -0.019 hanging from the top bar,
    # Z 2.16-2.76. Positions are the centre of each post.
    landmarks += [
        Landmark(
            "gate_pole_left",
            LandmarkType.GATE,
            LandmarkSubtype.GATE_POLE_EDGE,
            (0.0, -1.552, 0.032),
        ),
        Landmark(
            "gate_pole_right",
            LandmarkType.GATE,
            LandmarkSubtype.GATE_POLE_EDGE,
            (0.0, 1.546, 0.032),
        ),
        Landmark(
            "gate_pole_middle",
            LandmarkType.GATE,
            LandmarkSubtype.GATE_POLE_MIDDLE,
            (-0.015, -0.002, -0.258),
        ),
    ]
    return tuple(landmarks)


def _slalom_landmarks(_role_picks: dict) -> tuple[Landmark, ...]:
    # Not randomized (handbook 3.2.3: pipe colour is a fixed navigation cue;
    # "three sets of WHITE-RED-WHITE" -- each set/gate is one white, one red,
    # one white pole in a row). slalom__pvc_white.obj / slalom__red.obj merge
    # every pole into one mesh per colour, so getting individual pole
    # positions took clustering their vertices by (X, Y) and matching each
    # white pair to its nearest red pole. The simulator turns the slalom 90 deg
    # to the left about its centroid (POSE_OVERRIDE in
    # vortex-stonefish-sim tools/import_robosub_course.py), so each set lies
    # across the course and the sets follow each other along X. Real world
    # positions (pole height ~0.9 m, matching the handbook spec):
    #   gate 1: white (8.145, -1.239, 2.791) / red (8.102, 0.345, 2.624)
    #           / white (8.145, 1.857, 2.791)
    #   gate 2: white (10.148, -0.643, 2.791) / red (10.105, 0.941, 2.624)
    #           / white (10.148, 2.452, 2.791)
    #   gate 3: white (12.151, -1.643, 2.791) / red (12.108, -0.059, 2.624)
    #           / white (12.151, 1.452, 2.791)
    # The vehicle passes the sets one after another along X; they weave a
    # little sideways (Y) from set to set. Left = smaller Y (Y is right).
    gates = (
        ((-1.96, -1.648, 0.167), (-2.003, -0.064, 0.0), (-1.96, 1.448, 0.167)),
        ((0.043, -1.052, 0.167), (0.0, 0.532, 0.0), (0.043, 2.043, 0.167)),
        ((2.046, -2.052, 0.167), (2.003, -0.468, 0.0), (2.046, 1.043, 0.167)),
    )
    landmarks = []
    for i, (white_left, red, white_right) in enumerate(gates):
        landmarks.append(
            Landmark(
                f"slalom_{i}_white_left",
                LandmarkType.SLALOM_PIPE,
                LandmarkSubtype.SLALOM_PIPE_WHITE,
                white_left,
            )
        )
        landmarks.append(
            Landmark(
                f"slalom_{i}_red",
                LandmarkType.SLALOM_PIPE,
                LandmarkSubtype.SLALOM_PIPE_RED,
                red,
            )
        )
        landmarks.append(
            Landmark(
                f"slalom_{i}_white_right",
                LandmarkType.SLALOM_PIPE,
                LandmarkSubtype.SLALOM_PIPE_WHITE,
                white_right,
            )
        )
    return tuple(landmarks)


# Ring centres/diameters, fixed regardless of decal version -- verified by
# thresholding both stonefish_sim/.../textures/Task4_ver1.png and
# Task4_ver2.png (2304x2304) for red pixels and clustering: the four rings
# land within ~1% of the same normalized position/size in both, e.g.
# large-left at (0.161, 0.411) in ver1 vs (0.155, 0.396) in ver2. Only the
# icons printed next to them move. Converted to metres using the board's
# handbook face size (0.61 m x 0.61 m); image x -> world Y, image y (down)
# -> world Z (down), both centred on the decal's own centre (x=0, coplanar
# with the board face). Offsets are relative to the board's whole mesh
# bounding-box centre (base_pose), which also includes its mounting stand --
# if the board's actual face sits noticeably off that centre, nudge z
# accordingly.
_TORPEDO_CIRCLE_OFFSETS = {
    "large_left": (0.0, -0.207, -0.056),
    "large_right": (0.0, 0.215, 0.215),
    "small_top": (0.0, 0.007, -0.191),
    "small_bottom": (0.0, -0.001, 0.190),
}

# Handbook 3.2.5: front side shows fire+firetruck for Survey & Repair and
# blood+ambulance for Search & Rescue; the larger opening is marked by the
# fire/blood icon, the smaller by the firetruck/ambulance icon. In
# Task4_ver1.png, fire sits top-left (next to the large-left ring) and
# firetruck top-right (next to the small-top ring); in Task4_ver2.png those
# two icons swap with their Search & Rescue counterparts, so the same
# physical rings carry the opposite role. Confirmed by comparing icon
# layout between the two textures directly.
_TORPEDO_ROLE_BY_VERSION = {
    "Task4_ver1.png": {
        "large_left": LandmarkSubtype.TORPEDO_TARGET_LARGE_SURVEY_REPAIR,
        "large_right": LandmarkSubtype.TORPEDO_TARGET_LARGE_SEARCH_RESCUE,
        "small_top": LandmarkSubtype.TORPEDO_TARGET_SMALL_SURVEY_REPAIR,
        "small_bottom": LandmarkSubtype.TORPEDO_TARGET_SMALL_SEARCH_RESCUE,
    },
    "Task4_ver2.png": {
        "large_left": LandmarkSubtype.TORPEDO_TARGET_LARGE_SEARCH_RESCUE,
        "large_right": LandmarkSubtype.TORPEDO_TARGET_LARGE_SURVEY_REPAIR,
        "small_top": LandmarkSubtype.TORPEDO_TARGET_SMALL_SEARCH_RESCUE,
        "small_bottom": LandmarkSubtype.TORPEDO_TARGET_SMALL_SURVEY_REPAIR,
    },
}


# Offset from an icon to its opening in the board frame (x out of the front,
# y right, z down), per board version. The same numbers as
# landmark_server's rules.torpedo_targets_from_icons (placeholder values,
# to be measured on our own board) -- keep them in sync.
_TORPEDO_ICON_TO_HOLE = {
    1: {
        "fire": (0.0, 0.0, -0.169),
        "blood": (0.0, 0.0, -0.169),
        "firetruck": (0.0, 0.1888, 0.0),
        "ambulance": (0.0, -0.192, 0.0),
    },
    2: {
        "fire": (0.0, 0.0, -0.218),
        "blood": (0.0, 0.0, -0.168),
        "firetruck": (0.0, -0.1555, -0.15),
        "ambulance": (0.0, 0.193, 0.0),
    },
}

# Opening (by size and role) -> the icon printed next to it: the large opening
# is marked by the fire/blood icon, the small one by the firetruck/ambulance
# icon (handbook 3.2.5).
_TORPEDO_ICON_FOR_HOLE = {
    LandmarkSubtype.TORPEDO_TARGET_LARGE_SURVEY_REPAIR: (
        "fire",
        LandmarkSubtype.TORPEDO_ICON_FIRE,
    ),
    LandmarkSubtype.TORPEDO_TARGET_LARGE_SEARCH_RESCUE: (
        "blood",
        LandmarkSubtype.TORPEDO_ICON_BLOOD,
    ),
    LandmarkSubtype.TORPEDO_TARGET_SMALL_SURVEY_REPAIR: (
        "firetruck",
        LandmarkSubtype.TORPEDO_ICON_FIRETRUCK,
    ),
    LandmarkSubtype.TORPEDO_TARGET_SMALL_SEARCH_RESCUE: (
        "ambulance",
        LandmarkSubtype.TORPEDO_ICON_AMBULANCE,
    ),
}


def torpedo_icon_offset(version: int, icon: str, hole: Vec3) -> Vec3:
    """World offset of an icon, given the offset of its opening.

    The board faces the vehicle, i.e. its front points along -X in the world
    (yaw pi), so the board frame is the world frame rotated by pi about z:
    (bx, by, bz) -> (-bx, -by, bz). hole = icon + R * off, hence
    icon = hole - R * off = hole + (0, +off_y, -off_z) (x offsets are 0).
    """
    off = _TORPEDO_ICON_TO_HOLE[version][icon]
    return (hole[0] + off[0], hole[1] + off[1], hole[2] - off[2])


def _torpedo_board_landmarks(role_picks: dict) -> tuple[Landmark, ...]:
    # Handbook 3.2.5: the board has two different-size openings. Perception
    # does not see the openings, it sees the role icons printed next
    # to them: fire/blood at the large opening, firetruck/ambulance at the
    # small one. landmark_server turns the icons back into openings. Which
    # physical opening carries which role flips between the two decal versions.
    version_name = role_picks.get("torpedo_board")
    if version_name not in _TORPEDO_ROLE_BY_VERSION:
        version_name = "Task4_ver1.png"
    roles = _TORPEDO_ROLE_BY_VERSION[version_name]
    version = 1 if version_name == "Task4_ver1.png" else 2

    landmarks = [
        Landmark(
            "torpedo_board",
            LandmarkType.TORPEDO_BOARD,
            LandmarkSubtype.TORPEDO_BOARD_WHOLE,
            (0.0, 0.0, 0.0),
        )
    ]
    for name, hole in _TORPEDO_CIRCLE_OFFSETS.items():
        icon_name, icon_subtype = _TORPEDO_ICON_FOR_HOLE[roles[name]]
        landmarks.append(
            Landmark(
                f"torpedo_icon_{icon_name}",
                LandmarkType.TORPEDO_BOARD,
                icon_subtype,
                torpedo_icon_offset(version, icon_name, hole),
            )
        )
    return tuple(landmarks)


def _bin_landmarks(role_picks: dict) -> tuple[Landmark, ...]:
    # Offsets are each bin's own mesh-bbox centre minus the bin field's
    # (bins_pipeline__white.obj) centre -- real relative spacing, extracted
    # the same way as every Task.base_pose below.
    slots = {
        "bin_1": (0.515, 0.028, -0.386),
        "bin_2": (0.015, -0.484, -0.317),
        "bin_3": (0.016, 0.531, -0.148),
        "bin_4": (-0.496, 0.015, -0.115),
    }
    # The rig the bins sit on (bins_pipeline__white.obj): its centre is the
    # task's base_pose.
    landmarks = [
        Landmark(
            "bin_rig", LandmarkType.BIN, LandmarkSubtype.BIN_STRUCTURE, (0.0, 0.0, 0.0)
        )
    ]
    for slot, offset in slots.items():
        role = role_picks.get(slot)
        subtype = _BIN_ROLE_SUBTYPE.get(role, LandmarkSubtype.BIN_SEARCH_RESCUE)
        # The front camera sees a bin without knowing its role ...
        landmarks.append(
            Landmark(
                f"{slot}_front",
                LandmarkType.BIN,
                LandmarkSubtype.BIN_UNCLASSIFIED,
                offset,
            )
        )
        # ... the down camera sees the role icon inside it.
        landmarks.append(
            Landmark(slot, LandmarkType.BIN, subtype, offset, camera="down")
        )
    return tuple(landmarks)


def _octagon_landmarks(role_picks: dict) -> tuple[Landmark, ...]:
    # base_pose is the centre of octagon__pvc_white.obj (the floating frame,
    # at the surface). The four image plates hang 0.24 m under it; offsets are
    # the centres of octagon__icon_octagon_<n>.obj. Which image is on which
    # plate is drawn per run (manifest group octagon_plates); the plates
    # themselves do not move.
    slots = {
        "octagon_1": (1.296, -0.001, 0.243),
        "octagon_2": (-0.917, 0.916, 0.243),
        "octagon_3": (0.000, -1.297, 0.243),
        "octagon_4": (0.917, 0.916, 0.243),
    }
    landmarks = [
        Landmark(
            "octagon",
            LandmarkType.OCTAGON,
            LandmarkSubtype.OCTAGON_WHOLE,
            (0.0, 0.0, 0.0),
        )
    ]
    for i, (slot, offset) in enumerate(slots.items()):
        subtype = _OCTAGON_IMAGE_SUBTYPE.get(
            role_picks.get(slot), LandmarkSubtype.OCTAGON_IMAGE_REPAIR + i
        )
        landmarks.append(Landmark(slot, LandmarkType.OCTAGON, subtype, offset))
    return tuple(landmarks)


def _table_landmarks(role_picks: dict) -> tuple[Landmark, ...]:
    # base_pose is the centre of the table top (restore_table__white.obj),
    # 0.7 m above the pool floor, straight under the octagon. The baskets
    # (restore_table__icon_restore_table_<n>) and the loose items on top
    # (restore_jar_<n>, restore_container_<n>, dynamic bodies in the sim) are
    # seen by the down camera. Which image is where is drawn per run; the
    # items can also be moved during a run (see movable_* parameters).
    baskets = {
        "restore_table_1": (0.010, 0.393, 0.027),
        "restore_table_2": (0.011, -0.395, 0.027),
    }
    items = {
        "restore_jar_1": (-0.120, -0.120, -0.089),
        "restore_jar_2": (-0.120, 0.120, -0.089),
        "restore_container_1": (0.120, -0.120, -0.077),
        "restore_container_2": (0.120, 0.120, -0.077),
    }
    landmarks = [
        Landmark(
            "table", LandmarkType.TABLE, LandmarkSubtype.TABLE_WHOLE, (0.0, 0.0, 0.0)
        )
    ]
    for i, (slot, offset) in enumerate(baskets.items()):
        subtype = _TABLE_BASKET_SUBTYPE.get(
            role_picks.get(slot), LandmarkSubtype.TABLE_BASKET_SURVEY_REPAIR + i
        )
        landmarks.append(
            Landmark(slot, LandmarkType.TABLE, subtype, offset, camera="down")
        )
    for i, (slot, offset) in enumerate(items.items()):
        subtype = _TABLE_ITEM_SUBTYPE.get(
            role_picks.get(slot), LandmarkSubtype.TABLE_ITEM_NUTBOLT + i
        )
        landmarks.append(
            Landmark(
                slot, LandmarkType.TABLE, subtype, offset, camera="down", movable=True
            )
        )
    return tuple(landmarks)


TASKS: dict[str, Task] = {
    "gate": Task("gate", "3.2.2", (4.0, -0.017, 2.718), _gate_landmarks),
    # Centroid of the three slalom gates' red poles (see _slalom_landmarks).
    "slalom": Task("slalom", "3.2.3", (10.105, 0.409, 2.624), _slalom_landmarks),
    "torpedo_board": Task(
        "torpedo_board", "3.2.5", (16.82, -5.204, 2.825), _torpedo_board_landmarks
    ),
    "bin": Task("bin", "3.2.4", (16.544, 4.206, 3.152), _bin_landmarks),
    "octagon": Task("octagon", "3.2.6", (19.254, 0.114, 0.0), _octagon_landmarks),
    "table": Task("table", "3.2.6", (19.254, 0.113, 2.717), _table_landmarks),
}


def _load_icon_manifest() -> dict | None:
    """Load the same role-image manifest vortex-stonefish-sim randomizes from."""
    if get_package_share_directory is None:
        return None
    try:
        share = get_package_share_directory("stonefish_sim")
    except PackageNotFoundError:
        return None
    manifest_path = os.path.join(share, "metadata", "robosub_icons.json")
    try:
        with open(manifest_path) as f:
            return json.load(f)
    except FileNotFoundError:
        return None


def draw_role_picks(seed) -> tuple[dict, int | str]:
    """Reproduce robosub_icons.icon_parameters()'s draw, keyed by slot name.

    Returns (picks, resolved_seed). ``picks`` maps slot name (e.g. "gate_1",
    "bin_3", "torpedo_board") to the pool filename drawn for it. Empty if the
    stonefish_sim manifest isn't available -- callers fall back to default
    subtypes in that case.
    """
    manifest = _load_icon_manifest()
    if manifest is None:
        return {}, seed

    if seed in (None, ""):
        seed = random.randrange(2**31)
    else:
        try:
            seed = int(seed)
        except (TypeError, ValueError):
            pass
    rng = random.Random(seed)

    picks = {}
    for group in manifest["groups"]:
        slots = group["slots"]
        pool = list(group["pool"])
        if not pool:
            continue
        if group["mode"] == "choice":
            drawn = [rng.choice(pool) for _ in slots]
        else:
            rng.shuffle(pool)
            drawn = pool
        for slot, tex in zip(slots, drawn, strict=True):
            picks[slot] = tex
    return picks, seed
