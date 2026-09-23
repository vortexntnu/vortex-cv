"""Dummy landmarks look like real perception output.

They must also be consistent with landmark_server's rules (icons -> openings,
panels -> gate, bin roles).
"""

import math

import pytest
from robosub_dummy_publisher.course_layout import (
    _TORPEDO_CIRCLE_OFFSETS,
    _TORPEDO_ICON_TO_HOLE,
    _TORPEDO_ROLE_BY_VERSION,
    TASKS,
)
from vortex_msgs.msg import LandmarkSubtype, LandmarkType

VERSIONS = {1: "Task4_ver1.png", 2: "Task4_ver2.png"}


def _landmarks(task, role_picks=None):
    return TASKS[task].landmarks(role_picks or {})


def _by_subtype(landmarks):
    return {lm.landmark_subtype: lm for lm in landmarks}


def test_gate_has_whole_gate_and_two_panels():
    landmarks = _landmarks("gate")
    whole = [
        lm for lm in landmarks if lm.landmark_subtype == LandmarkSubtype.GATE_WHOLE
    ]
    panels = [
        lm for lm in landmarks if lm.landmark_subtype != LandmarkSubtype.GATE_WHOLE
    ]
    assert len(whole) == 1
    assert len(panels) == 2


def test_gate_whole_is_the_midpoint_of_the_panels_in_y():
    panels = [
        lm
        for lm in _landmarks("gate")
        if lm.landmark_subtype != LandmarkSubtype.GATE_WHOLE
    ]
    assert len(panels) == 2
    mid_y = sum(p.offset[1] for p in panels) / 2
    whole = next(
        lm
        for lm in _landmarks("gate")
        if lm.landmark_subtype == LandmarkSubtype.GATE_WHOLE
    )
    assert abs(whole.offset[1] - mid_y) < 0.05


def test_torpedo_board_gives_icons_not_openings():
    landmarks = _landmarks("torpedo_board")
    subtypes = {lm.landmark_subtype for lm in landmarks}
    assert LandmarkSubtype.TORPEDO_BOARD_WHOLE in subtypes
    assert {
        LandmarkSubtype.TORPEDO_ICON_FIRE,
        LandmarkSubtype.TORPEDO_ICON_BLOOD,
        LandmarkSubtype.TORPEDO_ICON_FIRETRUCK,
        LandmarkSubtype.TORPEDO_ICON_AMBULANCE,
    } <= subtypes
    # The openings themselves are derived by landmark_server.
    assert not subtypes & {
        LandmarkSubtype.TORPEDO_TARGET_LARGE_SEARCH_RESCUE,
        LandmarkSubtype.TORPEDO_TARGET_LARGE_SURVEY_REPAIR,
        LandmarkSubtype.TORPEDO_TARGET_SMALL_SEARCH_RESCUE,
        LandmarkSubtype.TORPEDO_TARGET_SMALL_SURVEY_REPAIR,
    }


@pytest.mark.parametrize("version", [1, 2])
def test_fire_above_blood_only_for_version_1(version):
    """landmark_server reads the board version from the icon heights.

    Fire above blood is version 1. z is down, so above means a smaller z.
    """
    icons = _by_subtype(
        _landmarks("torpedo_board", {"torpedo_board": VERSIONS[version]})
    )
    fire_z = icons[LandmarkSubtype.TORPEDO_ICON_FIRE].offset[2]
    blood_z = icons[LandmarkSubtype.TORPEDO_ICON_BLOOD].offset[2]
    assert (fire_z < blood_z) == (version == 1)


@pytest.mark.parametrize("version", [1, 2])
def test_openings_are_recovered_from_the_icons(version):
    """Icon + R(pi) * offset (the board faces -X) is the opening it marks."""
    version_name = VERSIONS[version]
    roles = _TORPEDO_ROLE_BY_VERSION[version_name]
    icons = _by_subtype(_landmarks("torpedo_board", {"torpedo_board": version_name}))

    icon_for_subtype = {
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
    for hole_name, hole in _TORPEDO_CIRCLE_OFFSETS.items():
        icon_name, icon_subtype = icon_for_subtype[roles[hole_name]]
        icon = icons[icon_subtype].offset
        ox, oy, oz = _TORPEDO_ICON_TO_HOLE[version][icon_name]
        # R(pi): (x, y, z) -> (-x, -y, z)
        recovered = (icon[0] - ox, icon[1] - oy, icon[2] + oz)
        assert recovered == pytest.approx(hole, abs=1e-9), hole_name


def test_each_bin_has_a_front_detection_and_a_down_camera_role():
    bins = _landmarks("bin")
    front = [lm for lm in bins if lm.camera == "front"]
    down = [lm for lm in bins if lm.camera == "down"]
    assert len(front) == len(down) == 4
    assert all(lm.landmark_subtype == LandmarkSubtype.BIN_UNCLASSIFIED for lm in front)
    assert all(
        lm.landmark_subtype
        in (LandmarkSubtype.BIN_SEARCH_RESCUE, LandmarkSubtype.BIN_SURVEY_REPAIR)
        for lm in down
    )
    # Same physical bins.
    assert sorted(lm.offset for lm in front) == sorted(lm.offset for lm in down)


def test_all_landmark_types_are_known():
    known = {
        LandmarkType.GATE,
        LandmarkType.SLALOM_PIPE,
        LandmarkType.TORPEDO_BOARD,
        LandmarkType.BIN,
    }
    for task in TASKS.values():
        for lm in task.landmarks({}):
            assert lm.landmark_type in known
            assert not math.isnan(lm.offset[0])
