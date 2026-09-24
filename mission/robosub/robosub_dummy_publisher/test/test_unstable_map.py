"""Robustness of the map against an unstable detector.

Runs the dummy publisher with the `unstable` profile (noise, missed
detections, lost frames, occlusions of 1-6 s, outliers and clutter) against
landmark_server, samples object_map for a minute and compares it with the true
course layout: ids kept, no duplicates, no tracks from clutter, positions near
the truth and no gaps while the landmarks are occluded. Prints a table per
class (run with -s to see it). UNSTABLE_PARAMS_FILE=<yaml> runs it with other
instability settings. Skipped when landmark_server is not installed.
"""

import collections
import math
import os
import random
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

from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from robosub_dummy_publisher.course_layout import TASKS, draw_role_picks  # noqa: E402
from vortex_msgs.msg import (  # noqa: E402
    LandmarkSubtype,
    LandmarkTrackArray,
    LandmarkType,
)

NAMESPACE = "nautilus"
ODOM_FRAME = "nautilus/odom"
SEED = "7"
NOISE_SEED = 3
VEHICLE = (8.0, 0.0, 2.0)  # between the gate (x = 4) and the torpedo board
WARMUP_SEC = 10.0
RUN_SEC = 60.0
MAX_POSITION_ERROR_M = 0.5
# Classes perception sees once and the map keeps forever: one id each.
FOREVER = {LandmarkType.GATE, LandmarkType.TORPEDO_BOARD}


def _truth():
    """(type, subtype) -> true positions of the landmarks the dummy publishes."""
    picks, _ = draw_role_picks(SEED)
    truth = collections.defaultdict(list)
    for task in TASKS.values():
        for lm in task.landmarks(picks):
            if (
                lm.camera == "front"
                and lm.landmark_type == LandmarkType.BIN
                and lm.landmark_subtype != LandmarkSubtype.BIN_STRUCTURE
            ):
                continue  # front-camera bins become role bins in the map
            pos = tuple(task.base_pose[i] + lm.offset[i] for i in range(3))
            truth[(lm.landmark_type, lm.landmark_subtype)].append(pos)
    # The map rules move two classes on purpose: the gate to the midpoint of
    # its panels, the board to the centre of its icons.
    truth[(LandmarkType.GATE, LandmarkSubtype.GATE_WHOLE)] = [
        _centre(
            truth[(LandmarkType.GATE, LandmarkSubtype.GATE_SEARCH_RESCUE)]
            + truth[(LandmarkType.GATE, LandmarkSubtype.GATE_SURVEY_REPAIR)]
        )
    ]
    truth[(LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_BOARD_WHOLE)] = [
        _centre(
            [
                p
                for (t, sub), ps in truth.items()
                if t == LandmarkType.TORPEDO_BOARD
                and LandmarkSubtype.TORPEDO_ICON_FIRE
                <= sub
                <= LandmarkSubtype.TORPEDO_ICON_AMBULANCE
                for p in ps
            ]
        )
    ]
    # The bin rig is locked to the pool floor (rules.z_lock).
    rig = (LandmarkType.BIN, LandmarkSubtype.BIN_STRUCTURE)
    truth[rig] = [(x, y, 3.432) for x, y, _ in truth[rig]]
    return truth


def _centre(points):
    return tuple(sum(p[i] for p in points) / len(points) for i in range(3))


@pytest.fixture(scope="module")
def samples():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    config = os.path.join(DUMMY_SHARE, "config")
    procs = [
        subprocess.Popen(
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
                os.path.join(AUV_SHARE, "config", "robots", "nautilus.yaml"),
            ],
            env=env,
            start_new_session=True,
        ),
        subprocess.Popen(
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
                "--params-file",
                os.environ.get(
                    "UNSTABLE_PARAMS_FILE",
                    os.path.join(config, "robosub_dummy_publisher_unstable.yaml"),
                ),
                "-p",
                f"seed:={SEED}",
                "-p",
                f"noise_seed:={NOISE_SEED}",
            ],
            env=env,
            start_new_session=True,
        ),
    ]

    rclpy.init()
    node = rclpy.create_node("unstable_map_test")
    collected = []
    start = time.monotonic()

    def on_map(msg):
        if time.monotonic() - start >= WARMUP_SEC:
            collected.append(msg)

    node.create_subscription(
        LandmarkTrackArray,
        f"/{NAMESPACE}/landmark_server/object_map",
        on_map,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )
    odom_pub = node.create_publisher(
        Odometry, f"/{NAMESPACE}/odom", qos_profile_sensor_data
    )

    def publish_odom():
        odom = Odometry()
        odom.header.frame_id = ODOM_FRAME
        odom.header.stamp = node.get_clock().now().to_msg()
        p = odom.pose.pose.position
        p.x, p.y, p.z = VEHICLE
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

    node.create_timer(0.1, publish_odom)
    while time.monotonic() - start < WARMUP_SEC + RUN_SEC:
        rclpy.spin_once(node, timeout_sec=0.05)

    yield collected

    node.destroy_node()
    rclpy.shutdown()
    for p in procs:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)


def _measured(msg):
    for t in msg.landmark_tracks:
        if not t.derived:
            yield t


def _key(track):
    return (track.landmark.type.value, track.landmark.subtype.value)


def _xyz(track):
    p = track.landmark.pose.pose.position
    return (p.x, p.y, p.z)


def _stats(samples):
    """Per class statistics of the maps against the true layout.

    Ids seen, the most tracks at once, the share of maps with all of them
    present, the worst error to the nearest true landmark, and track samples
    that are not near any true landmark (clutter in the map).
    """
    truth = _truth()
    ids = collections.defaultdict(set)
    max_count = collections.Counter()
    complete = collections.Counter()
    worst = collections.defaultdict(float)
    clutter = collections.Counter()
    for msg in samples:
        count = collections.Counter()
        for t in _measured(msg):
            key = _key(t)
            if key not in truth:
                clutter[key] += 1
                continue
            count[key] += 1
            ids[key].add(t.landmark.id)
            err = min(math.dist(_xyz(t), p) for p in truth[key])
            if err > MAX_POSITION_ERROR_M:
                clutter[key] += 1
            worst[key] = max(worst[key], err)
        for key in truth:
            max_count[key] = max(max_count[key], count[key])
            if count[key] >= len(truth[key]):
                complete[key] += 1
    return truth, ids, max_count, complete, worst, clutter


def test_map_is_stable_under_unstable_detections(samples):
    assert len(samples) > 100, "too few object_map messages"
    truth, ids, max_count, complete, worst, clutter = _stats(samples)

    print(f"\n{len(samples)} maps over {RUN_SEC:.0f} s after {WARMUP_SEC:.0f} s warmup")
    print(
        f"{'class':>8} {'true':>4} {'ids':>4} {'max':>4} {'complete':>9} "
        f"{'worst err':>9} {'clutter':>7}"
    )
    for key in sorted(truth):
        print(
            f"{key[0]:>4}/{key[1]:<3} {len(truth[key]):>4} {len(ids[key]):>4} "
            f"{max_count[key]:>4} {complete[key] / len(samples):>8.0%} "
            f"{worst[key]:>8.2f}m {clutter[key]:>7}"
        )

    problems = []
    for key, positions in truth.items():
        n = len(positions)
        if key[0] == LandmarkType.SLALOM_PIPE:
            continue  # only pipes within 7 m of the vehicle are kept
        if max_count[key] > n:
            problems.append(f"{key}: {max_count[key]} tracks at once, {n} true")
        if clutter[key]:
            problems.append(
                f"{key}: {clutter[key]} track samples > "
                f"{MAX_POSITION_ERROR_M} m from the truth"
            )
        if complete[key] < 0.95 * len(samples):
            problems.append(
                f"{key}: all {n} present in only "
                f"{complete[key] / len(samples):.0%} of the maps"
            )
        if key[0] in FOREVER and len(ids[key]) > n:
            problems.append(f"{key}: ids changed ({sorted(ids[key])})")
    assert not problems, "\n".join(problems)
