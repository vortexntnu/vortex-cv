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
Z=0 -- same convention as stonefish_sim/scenarios/robosub.scn (Stonefish
builds rpy as ZYX, so the scenario's rpy="pi 0 pi/2" is exactly this
mapping). That frame is also the sim's odom frame: the odometry is Stonefish's
own world pose, passed through unchanged. For gate and slalom the poles are
the connected components of each colour's mesh (each element's .obj merges
several physical poles into one mesh per colour/material, so a single
bounding box isn't enough). For the torpedo board the openings are the grey
discs behind the decal's cut-outs, and the icons come from the two decal
textures the sim randomizes between (Task4_ver1.png / Task4_ver2.png): each
icon's bounding box in the image, mapped through the decal's UVs onto the
board face. See each task builder below for the specifics.
"""

from __future__ import annotations

import json
import math
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
    decoy: bool = False  # another object a detector mistakes for this class
    # Yaw [rad, world] of the surface normal (+X out of the front) for objects
    # whose detector measures it; None = position only.
    normal_yaw: float | None = None


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
    # every pole into one mesh per colour, so the individual poles are the
    # connected components of those meshes. The white mesh also holds the
    # base pipes lying on the floor (Z 3.418), which are not poles. The
    # simulator turns the slalom 90 deg to the left about its centroid
    # (POSE_OVERRIDE in vortex-stonefish-sim tools/import_robosub_course.py),
    # so each set lies across the course and the sets follow each other along
    # X. Every pole, white or red, is 0.938 m tall (Z 2.155-3.093, centre
    # 2.624). World positions of the pole centres:
    #   gate 1: white (8.144, -1.240) / red (8.101, 0.345) / white (8.144, 1.858)
    #   gate 2: white (10.148, -0.645) / red (10.105, 0.940) / white (10.148, 2.454)
    #   gate 3: white (12.151, -1.645) / red (12.107, -0.060) / white (12.151, 1.454)
    # The vehicle passes the sets one after another along X; they weave a
    # little sideways (Y) from set to set. Left = smaller Y (Y is right).
    gates = (
        ((-1.961, -1.649, 0.0), (-2.004, -0.064, 0.0), (-1.961, 1.449, 0.0)),
        ((0.043, -1.054, 0.0), (0.0, 0.531, 0.0), (0.043, 2.045, 0.0)),
        ((2.046, -2.054, 0.0), (2.002, -0.469, 0.0), (2.046, 1.045, 0.0)),
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


# The board face in the sim (torpedo_board__grey.obj / __icon_torpedo_board.obj):
# a 0.6096 m square at world X 17.043 (the front, facing the vehicle along
# -X), centred on (Y -5.204, Z 2.554). That centre is the task's base_pose.
# The decal is UV-mapped straight onto that square: image x -> world Y (left
# to right as seen from the vehicle), image y (down) -> world Z (down).
#
# The openings are the grey discs behind the decal's cut-outs, read off
# torpedo_board__grey.obj: large 0.127 m, small 0.101 m across. They are the
# same in both decal versions (ver2's printed rings sit exactly on them,
# ver1's within ~1 cm).
_TORPEDO_OPENING_OFFSETS = {
    "large_left": (0.0, -0.210, -0.064),
    "large_right": (0.0, 0.214, 0.216),
    "small_top": (0.0, -0.003, -0.192),
    "small_bottom": (0.0, -0.006, 0.200),
}

# The icons, per decal version: centre of each icon's bounding box in the
# texture (stonefish_sim/.../textures/Task4_ver1.png, Task4_ver2.png,
# 2304x2304), mapped through the decal's UVs onto the board face. Handbook
# 3.2.5: fire/blood mark the large opening, firetruck/ambulance the small one.
# Version 1: fire above the large-left opening, firetruck right of the
# small-top one, blood above the large-right one, ambulance left of the
# small-bottom one. Version 2 swaps each Survey & Repair icon with its Search
# & Rescue counterpart.
_TORPEDO_ICON_OFFSETS = {
    "Task4_ver1.png": {
        "fire": (0.0, -0.206, -0.214),
        "firetruck": (0.0, 0.186, -0.193),
        "blood": (0.0, 0.219, 0.058),
        "ambulance": (0.0, -0.182, 0.202),
    },
    "Task4_ver2.png": {
        "blood": (0.0, -0.210, -0.222),
        "ambulance": (0.0, 0.178, -0.204),
        "fire": (0.0, 0.213, 0.056),
        "firetruck": (0.0, -0.183, 0.183),
    },
}
# The icon -> opening offsets these imply, in the board frame (x out of the
# front, y right as seen from the board, i.e. -Y world; z down), are
# landmark_server's rules.torpedo_targets_from_icons in its sim.yaml -- keep
# them in sync.

_TORPEDO_ICON_SUBTYPE = {
    "fire": LandmarkSubtype.TORPEDO_ICON_FIRE,
    "blood": LandmarkSubtype.TORPEDO_ICON_BLOOD,
    "firetruck": LandmarkSubtype.TORPEDO_ICON_FIRETRUCK,
    "ambulance": LandmarkSubtype.TORPEDO_ICON_AMBULANCE,
}


def _torpedo_board_landmarks(role_picks: dict) -> tuple[Landmark, ...]:
    # Handbook 3.2.5: the board has two different-size openings. Perception
    # does not see the openings, it sees the role icons printed next
    # to them: fire/blood at the large opening, firetruck/ambulance at the
    # small one. landmark_server turns the icons back into openings. Which
    # physical opening carries which role flips between the two decal versions.
    version_name = role_picks.get("torpedo_board")
    if version_name not in _TORPEDO_ICON_OFFSETS:
        version_name = "Task4_ver1.png"

    landmarks = [
        Landmark(
            "torpedo_board",
            LandmarkType.TORPEDO_BOARD,
            LandmarkSubtype.TORPEDO_BOARD_WHOLE,
            (0.0, 0.0, 0.0),
            # The front faces the vehicle coming down the course (-X world).
            normal_yaw=math.pi,
        )
    ]
    for icon_name, offset in _TORPEDO_ICON_OFFSETS[version_name].items():
        landmarks.append(
            Landmark(
                f"torpedo_icon_{icon_name}",
                LandmarkType.TORPEDO_BOARD,
                _TORPEDO_ICON_SUBTYPE[icon_name],
                offset,
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
    # The role icon lies on the floor of its bin (bin_<n>__icon_bin_<n>.obj),
    # 0.14 m below the bin's centre; the rig is tilted, so each floor is at
    # its own depth.
    icons = {
        "bin_1": (0.515, 0.028, -0.247),
        "bin_2": (0.015, -0.484, -0.178),
        "bin_3": (0.016, 0.531, -0.009),
        "bin_4": (-0.496, 0.015, 0.025),
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
            Landmark(slot, LandmarkType.BIN, subtype, icons[slot], camera="down")
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
    # Centre of the board's front face (see _TORPEDO_OPENING_OFFSETS).
    "torpedo_board": Task(
        "torpedo_board", "3.2.5", (17.043, -5.204, 2.554), _torpedo_board_landmarks
    ),
    "bin": Task("bin", "3.2.4", (16.544, 4.206, 3.152), _bin_landmarks),
    "octagon": Task("octagon", "3.2.6", (19.254, 0.114, 0.0), _octagon_landmarks),
    "table": Task("table", "3.2.6", (19.254, 0.113, 2.717), _table_landmarks),
}


# Decoys: other PVC poles on the course that a pipe detector takes for slalom
# pipes (seen with the real slalom detector: the gate's posts). World
# positions, the centres of the posts (gate__white/red/black.obj, see
# _gate_landmarks): the two uprights (red and black sleeves on white PVC) as
# white pipes, the short red post between the openings as a red pipe. Only
# published with decoy_probability > 0.
DECOYS: tuple[tuple[Landmark, Vec3], ...] = (
    (
        Landmark(
            "decoy_gate_upright_left",
            LandmarkType.SLALOM_PIPE,
            LandmarkSubtype.SLALOM_PIPE_WHITE,
            (0.0, 0.0, 0.0),
            decoy=True,
        ),
        (4.0, -1.569, 2.750),
    ),
    (
        Landmark(
            "decoy_gate_upright_right",
            LandmarkType.SLALOM_PIPE,
            LandmarkSubtype.SLALOM_PIPE_WHITE,
            (0.0, 0.0, 0.0),
            decoy=True,
        ),
        (4.0, 1.529, 2.750),
    ),
    (
        Landmark(
            "decoy_gate_middle_post",
            LandmarkType.SLALOM_PIPE,
            LandmarkSubtype.SLALOM_PIPE_RED,
            (0.0, 0.0, 0.0),
            decoy=True,
        ),
        (3.985, -0.019, 2.460),
    ),
)


# Classes a detector mixes up (class_confusion_* parameters): a pipe's colour
# under bad light, and the icons of one shape (the two red role icons, the two
# vehicles). (type, subtype) -> the classes it can be reported as.
CONFUSIONS: dict[tuple[int, int], tuple[tuple[int, int], ...]] = {
    (LandmarkType.SLALOM_PIPE, LandmarkSubtype.SLALOM_PIPE_WHITE): (
        (LandmarkType.SLALOM_PIPE, LandmarkSubtype.SLALOM_PIPE_RED),
    ),
    (LandmarkType.SLALOM_PIPE, LandmarkSubtype.SLALOM_PIPE_RED): (
        (LandmarkType.SLALOM_PIPE, LandmarkSubtype.SLALOM_PIPE_WHITE),
    ),
    (LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_ICON_FIRE): (
        (LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_ICON_BLOOD),
    ),
    (LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_ICON_BLOOD): (
        (LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_ICON_FIRE),
    ),
    (LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_ICON_FIRETRUCK): (
        (LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_ICON_AMBULANCE),
    ),
    (LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_ICON_AMBULANCE): (
        (LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_ICON_FIRETRUCK),
    ),
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
