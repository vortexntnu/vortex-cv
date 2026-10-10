"""Where each dummy landmark is on the RoboSub course.

Positions come from the course meshes in vortex-stonefish-sim, in the
simulator's world frame: X down the course, Y right, Z down, surface at Z=0.

Which role image goes where is drawn with the same seed and manifest as the
simulator (robosub_icons.json), so launching both with the same seed gives
matching roles. Without stonefish_sim installed the roles fall back to fixed
defaults.
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
    # Yaw of the surface normal in the world frame, None = position only.
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


# Role image filename -> landmark subtype.
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
    # Role panels, from the gate icon meshes.
    slots = (("gate_1", (-0.023, 0.788, -0.445)), ("gate_2", (-0.023, -0.771, -0.445)))
    landmarks = [
        Landmark("gate", LandmarkType.GATE, LandmarkSubtype.GATE_WHOLE, (0.0, 0.0, 0.0))
    ]
    for slot, offset in slots:
        role = role_picks.get(slot)
        subtype = _GATE_ROLE_SUBTYPE.get(role, LandmarkSubtype.GATE_SEARCH_RESCUE)
        landmarks.append(Landmark(slot, LandmarkType.GATE, subtype, offset))
    # Centres of the two outer posts and the short middle post.
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
    # Three rows of white, red, white. Pole centres in the world frame:
    #   row 1: white (8.144, -1.240) / red (8.101, 0.345) / white (8.144, 1.858)
    #   row 2: white (10.148, -0.645) / red (10.105, 0.940) / white (10.148, 2.454)
    #   row 3: white (12.151, -1.645) / red (12.107, -0.060) / white (12.151, 1.454)
    # The simulator turns the slalom 90 deg, so the rows follow each other
    # along X.
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


# The board is a 0.6096 m square facing -X. Offsets are (x, y, z) from its
# centre in the world frame, so y is to the right seen from the vehicle.
# landmark_server has the same opening offsets in torpedo.openings.
_TORPEDO_OPENING_OFFSETS = {
    "large_left": (0.0, -0.210, -0.064),
    "large_right": (0.0, 0.214, 0.216),
    "small_top": (0.0, -0.003, -0.192),
    "small_bottom": (0.0, -0.006, 0.200),
}

# Icon centres per decal version. Fire and blood mark the large openings,
# firetruck and ambulance the small ones. Version 2 swaps the roles.
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

_TORPEDO_ICON_SUBTYPE = {
    "fire": LandmarkSubtype.TORPEDO_ICON_FIRE,
    "blood": LandmarkSubtype.TORPEDO_ICON_BLOOD,
    "firetruck": LandmarkSubtype.TORPEDO_ICON_FIRETRUCK,
    "ambulance": LandmarkSubtype.TORPEDO_ICON_AMBULANCE,
}


def _torpedo_board_landmarks(role_picks: dict) -> tuple[Landmark, ...]:
    # Perception sees the icons, not the openings.
    version_name = role_picks.get("torpedo_board")
    if version_name not in _TORPEDO_ICON_OFFSETS:
        version_name = "Task4_ver1.png"

    landmarks = [
        Landmark(
            "torpedo_board",
            LandmarkType.TORPEDO_BOARD,
            LandmarkSubtype.TORPEDO_BOARD_WHOLE,
            (0.0, 0.0, 0.0),
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
    # Bin centres relative to the rig centre.
    slots = {
        "bin_1": (0.515, 0.028, -0.386),
        "bin_2": (0.015, -0.484, -0.317),
        "bin_3": (0.016, 0.531, -0.148),
        "bin_4": (-0.496, 0.015, -0.115),
    }
    # The role icon is on the floor of each bin, 0.14 m below its centre.
    icons = {
        "bin_1": (0.515, 0.028, -0.247),
        "bin_2": (0.015, -0.484, -0.178),
        "bin_3": (0.016, 0.531, -0.009),
        "bin_4": (-0.496, 0.015, 0.025),
    }
    landmarks = [
        Landmark(
            "bin_rig", LandmarkType.BIN, LandmarkSubtype.BIN_STRUCTURE, (0.0, 0.0, 0.0)
        )
    ]
    for slot, offset in slots.items():
        role = role_picks.get(slot)
        subtype = _BIN_ROLE_SUBTYPE.get(role, LandmarkSubtype.BIN_SEARCH_RESCUE)
        # The front camera sees a bin without its role.
        landmarks.append(
            Landmark(
                f"{slot}_front",
                LandmarkType.BIN,
                LandmarkSubtype.BIN_UNCLASSIFIED,
                offset,
            )
        )
        # The down camera sees the role icon.
        landmarks.append(
            Landmark(slot, LandmarkType.BIN, subtype, icons[slot], camera="down")
        )
    return tuple(landmarks)


def _octagon_landmarks(role_picks: dict) -> tuple[Landmark, ...]:
    # The four image plates hang 0.24 m under the octagon frame. Which image
    # is on which plate is drawn per run.
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
    # Baskets and loose items on the table, seen by the down camera.
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
    "slalom": Task("slalom", "3.2.3", (10.105, 0.409, 2.624), _slalom_landmarks),
    "torpedo_board": Task(
        "torpedo_board", "3.2.5", (17.043, -5.204, 2.554), _torpedo_board_landmarks
    ),
    "bin": Task("bin", "3.2.4", (16.544, 4.206, 3.152), _bin_landmarks),
    "octagon": Task("octagon", "3.2.6", (19.254, 0.114, 0.0), _octagon_landmarks),
    "table": Task("table", "3.2.6", (19.254, 0.113, 2.717), _table_landmarks),
}


# Gate posts that a pipe detector can take for slalom pipes. Only
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


# (type, subtype) -> the classes it can be misreported as.
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
