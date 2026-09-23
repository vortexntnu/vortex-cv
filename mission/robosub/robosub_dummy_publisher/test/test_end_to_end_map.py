"""End to end: dummy perception -> landmark_server -> object_map.

Starts the dummy publisher (detections as perception gives them: positions without
orientation, gate panels, torpedo icons, bin roles) and landmark_server, and
checks that the map derives the gate yaw, the torpedo openings and the bin
roles. Skipped when landmark_server is not installed.
"""

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
from robosub_dummy_publisher.course_layout import (  # noqa: E402
    _TORPEDO_CIRCLE_OFFSETS,
    _TORPEDO_ROLE_BY_VERSION,
    TASKS,
)
from vortex_msgs.msg import (  # noqa: E402
    LandmarkSubtype,
    LandmarkTrackArray,
    LandmarkType,
)

NAMESPACE = "nautilus"
ODOM_FRAME = "nautilus/odom"
VEHICLE = (8.0, 0.0, 2.0)  # between the gate (x = 4) and the torpedo board


def _yaw(track):
    q = track.landmark.pose.pose.orientation
    import math

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


@pytest.fixture(scope="module")
def object_map():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    landmark_config = os.path.join(
        LANDMARK_SHARE, "config", "landmark_server_config.yaml"
    )
    drone_config = os.path.join(AUV_SHARE, "config", "robots", "nautilus.yaml")
    dummy_config = os.path.join(
        DUMMY_SHARE, "config", "robosub_dummy_publisher_params.yaml"
    )
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
                landmark_config,
                "--params-file",
                drone_config,
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
                dummy_config,
            ],
            env=env,
            start_new_session=True,
        ),
    ]

    rclpy.init()
    node = rclpy.create_node("end_to_end_map_test")
    latest = {}
    node.create_subscription(
        LandmarkTrackArray,
        f"/{NAMESPACE}/landmark_server/object_map",
        lambda m: latest.update(msg=m),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )
    odom_pub = node.create_publisher(
        Odometry, f"/{NAMESPACE}/odom", qos_profile_sensor_data
    )

    def publish_odom():
        odom = Odometry()
        odom.header.frame_id = ODOM_FRAME
        odom.header.stamp = node.get_clock().now().to_msg()
        (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
        ) = VEHICLE
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

    node.create_timer(0.1, publish_odom)

    end = time.monotonic() + 25.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    yield latest.get("msg")

    node.destroy_node()
    rclpy.shutdown()
    for p in procs:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)


def _find(object_map, type_, subtype):
    return [
        t
        for t in object_map.landmark_tracks
        if t.landmark.type.value == type_ and t.landmark.subtype.value == subtype
    ]


def test_map_was_published(object_map):
    assert object_map is not None
    assert object_map.landmark_tracks


def test_gate_yaw_from_panels(object_map):
    import math

    (gate,) = _find(object_map, LandmarkType.GATE, LandmarkSubtype.GATE_WHOLE)
    assert gate.has_orientation
    # The vehicle (x = 8) is behind the gate (x = 4): the front points to +x.
    assert abs(_yaw(gate)) < math.radians(3.0)
    # Pulled to the panel midpoint (base pose of the gate task).
    assert gate.landmark.pose.pose.position.x == pytest.approx(4.0, abs=0.1)


def test_torpedo_openings_from_icons(object_map):
    import math

    # The board faces the vehicle (x = 8): front points to -x.
    (board,) = _find(
        object_map, LandmarkType.TORPEDO_BOARD, LandmarkSubtype.TORPEDO_BOARD_WHOLE
    )
    assert board.has_orientation
    assert abs(abs(_yaw(board)) - math.pi) < math.radians(3.0)

    base = TASKS["torpedo_board"].base_pose
    roles = _TORPEDO_ROLE_BY_VERSION["Task4_ver1.png"]
    # Without stonefish_sim's manifest the dummy falls back to version 1; with
    # it the version follows the seed, so accept either mapping.
    seen = 0
    for hole in _TORPEDO_CIRCLE_OFFSETS.values():
        expected = tuple(b + h for b, h in zip(base, hole, strict=True))
        candidates = [
            t
            for t in object_map.landmark_tracks
            if t.landmark.type.value == LandmarkType.TORPEDO_BOARD
            and t.derived
            and t.landmark.subtype.value
            in (
                LandmarkSubtype.TORPEDO_TARGET_LARGE_SEARCH_RESCUE,
                LandmarkSubtype.TORPEDO_TARGET_LARGE_SURVEY_REPAIR,
                LandmarkSubtype.TORPEDO_TARGET_SMALL_SEARCH_RESCUE,
                LandmarkSubtype.TORPEDO_TARGET_SMALL_SURVEY_REPAIR,
            )
        ]
        for t in candidates:
            p = t.landmark.pose.pose.position
            if (p.x - expected[0]) ** 2 + (p.y - expected[1]) ** 2 + (
                p.z - expected[2]
            ) ** 2 < 0.05**2:
                seen += 1
                break
    assert seen == 4, "each of the four openings should be within 5 cm"
    del roles


def test_bin_roles_from_down_camera(object_map):
    unclassified = _find(object_map, LandmarkType.BIN, LandmarkSubtype.BIN_UNCLASSIFIED)
    role_bins = [
        t
        for t in object_map.landmark_tracks
        if t.landmark.type.value == LandmarkType.BIN
        and t.landmark.subtype.value
        in (LandmarkSubtype.BIN_SEARCH_RESCUE, LandmarkSubtype.BIN_SURVEY_REPAIR)
    ]
    assert len(role_bins) == 4
    # Each bin appears once: the role bin, not the roleless duplicate.
    assert unclassified == []


def test_slalom_pipes_are_in_the_map(object_map):
    white = _find(
        object_map, LandmarkType.SLALOM_PIPE, LandmarkSubtype.SLALOM_PIPE_WHITE
    )
    red = _find(object_map, LandmarkType.SLALOM_PIPE, LandmarkSubtype.SLALOM_PIPE_RED)
    assert len(white) == 6
    assert len(red) == 3
    # No orientation for pipes (position only).
    assert not any(t.has_orientation for t in white + red)
