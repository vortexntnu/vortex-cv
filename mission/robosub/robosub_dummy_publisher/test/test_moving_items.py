"""Loose objects on the table (jars, containers) moved during a run.

Runs landmark_server and the dummy publisher with only the table and moving
items, and checks that the map follows every move: within 0.15 m of the new
spot in a few seconds, with the same id and no track left behind at the old
spot. Skipped when landmark_server is not installed.
"""

import math
import os
import random
import re
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("ament_index_python")
from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError,
    get_package_share_directory,
)

try:
    LANDMARK_SHARE = get_package_share_directory("landmark_server")
    AUV_SHARE = get_package_share_directory("auv_setup")
    DUMMY_SHARE = get_package_share_directory("robosub_dummy_publisher")
except PackageNotFoundError:
    pytest.skip("landmark_server / auv_setup not installed", allow_module_level=True)

from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from vortex_msgs.msg import LandmarkTrackArray, LandmarkType  # noqa: E402

NAMESPACE = "nautilus"
MOVE_INTERVAL_SEC = 8.0
RUN_SEC = 45.0
FOLLOW_WITHIN_SEC = 4.0
NEAR_M = 0.15


@pytest.fixture(scope="module")
def run():
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    env["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    config = os.path.join(DUMMY_SHARE, "config")
    server = subprocess.Popen(
        [
            "ros2",
            "run",
            "landmark_server",
            "landmark_server_node",
            "--ros-args",
            "-r",
            f"__ns:=/{NAMESPACE}",
            "--params-file",
            os.path.join(LANDMARK_SHARE, "config", "landmark_server_config.yaml"),
            "--params-file",
            os.path.join(LANDMARK_SHARE, "config", "sim.yaml"),
            "--params-file",
            os.path.join(AUV_SHARE, "config", "robots", "nautilus.yaml"),
        ],
        env=env,
        start_new_session=True,
    )
    dummy = subprocess.Popen(
        [
            "ros2",
            "run",
            "robosub_dummy_publisher",
            "robosub_dummy_publisher_node",
            "--ros-args",
            "-r",
            f"__ns:=/{NAMESPACE}",
            "--params-file",
            os.path.join(config, "robosub_dummy_publisher_params.yaml"),
            "-p",
            "seed:=7",
            "-p",
            "noise_seed:=5",
            "-p",
            "tasks:=[table]",
            "-p",
            f"movable_move_interval_sec:={MOVE_INTERVAL_SEC}",
            "-p",
            "movable_move_radius_m:=0.5",
        ],
        env=env,
        start_new_session=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    os.set_blocking(dummy.stdout.fileno(), False)

    rclpy.init()
    node = rclpy.create_node("moving_items_test")
    maps = []
    node.create_subscription(
        LandmarkTrackArray,
        f"/{NAMESPACE}/landmark_server/object_map",
        lambda m: maps.append((time.monotonic(), m)),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )
    subtype_of = {}
    moves = []  # (time, label, new position)
    buf = ""
    start = time.monotonic()
    while time.monotonic() - start < RUN_SEC:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            buf += dummy.stdout.read() or ""
        except (BlockingIOError, TypeError):
            pass
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            m = re.search(
                r"(restore_(?:jar|container)_\d)\s+type=11 subtype=(\d+)", line
            )
            if m:
                subtype_of[m.group(1)] = int(m.group(2))
            m = re.search(
                r"moved (restore_\w+) to \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)", line
            )
            if m:
                moves.append(
                    (time.monotonic(), m.group(1), tuple(map(float, m.group(2, 3, 4))))
                )

    yield subtype_of, moves, maps

    node.destroy_node()
    rclpy.shutdown()
    for p in (server, dummy):
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
    for p in (server, dummy):
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)


def _tracks(msg, subtype):
    return [
        t
        for t in msg.landmark_tracks
        if t.landmark.type.value == LandmarkType.TABLE
        and t.landmark.subtype.value == subtype
    ]


def _xyz(track):
    p = track.landmark.pose.pose.position
    return (p.x, p.y, p.z)


def test_map_follows_moved_items(run):
    subtype_of, moves, maps = run
    assert len(subtype_of) == 4, f"items not seen in the dummy log: {subtype_of}"
    # Judge a move only if the item then stays put long enough to be
    # followed: not moved again right away, and not at the end of the run.
    judged = []
    for t, label, pos in moves:
        until = min(
            [tt for tt, ll, _ in moves if ll == label and tt > t] + [maps[-1][0]]
        )
        if until - t >= FOLLOW_WITHIN_SEC:
            judged.append((t, label, pos))
    assert len(judged) >= 5, "too few moves to judge"

    problems = []
    for t, label, pos in judged:
        subtype = subtype_of[label]
        after = [m for tm, m in maps if t <= tm <= t + FOLLOW_WITHIN_SEC]
        if not any(
            math.dist(_xyz(tr), pos) < NEAR_M
            for m in after
            for tr in _tracks(m, subtype)
        ):
            problems.append(
                f"{label}: not within {NEAR_M} m of {pos} after {FOLLOW_WITHIN_SEC} s"
            )
        if any(len(_tracks(m, subtype)) > 1 for m in after):
            problems.append(f"{label}: a track stayed behind at the old spot")

    for label, subtype in subtype_of.items():
        ids = {tr.landmark.id for _, m in maps for tr in _tracks(m, subtype)}
        if len(ids) != 1:
            problems.append(f"{label}: ids changed {sorted(ids)}")
    assert not problems, "\n".join(problems)
