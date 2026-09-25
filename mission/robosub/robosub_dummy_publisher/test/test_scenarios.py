"""Scenario runs of landmark_targets against the full chain.

dummy perception -> landmark_server -> scenario node -> waypoint_manager ->
reference_filter_dp_quat -> a kinematic fake vehicle that follows the
reference. Skipped when a package is missing.
"""

import math
import os
import random
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError,
    get_package_share_directory,
)

try:
    LANDMARK_SHARE = get_package_share_directory("landmark_server")
    AUV_SHARE = get_package_share_directory("auv_setup")
    DUMMY_SHARE = get_package_share_directory("robosub_dummy_publisher")
    RF_SHARE = get_package_share_directory("reference_filter_dp_quat")
    get_package_share_directory("waypoint_manager")
    get_package_share_directory("landmark_targets")
except PackageNotFoundError:
    pytest.skip("workspace packages not installed", allow_module_level=True)

from geometry_msgs.msg import (  # noqa: E402
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
)
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from robosub_dummy_publisher.course_layout import TASKS  # noqa: E402
from vortex_msgs.msg import ReferenceFilterQuat  # noqa: E402

NAMESPACE = "nautilus"
ODOM_FRAME = "nautilus/odom"
START = (0.0, 0.0, 2.0)


def _popen(cmd, env):
    return subprocess.Popen(cmd, env=env, start_new_session=True)


def _stop(procs):
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)


def run_scenario(
    tasks, start, scenario, extra_params=(), timeout=150.0, warmup=14.0, seed="7"
):
    """Run a scenario; return (exit code, final vehicle pose [x y z qw qx qy qz])."""
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    drone_config = os.path.join(AUV_SHARE, "config", "robots", "nautilus.yaml")
    ns = ["--ros-args", "-r", f"__ns:=/{NAMESPACE}"]

    daemons = [
        _popen(
            [
                "ros2",
                "run",
                "landmark_server",
                "landmark_server_node",
                *ns,
                "--params-file",
                os.path.join(LANDMARK_SHARE, "config", "landmark_server_config.yaml"),
                "--params-file",
                os.path.join(LANDMARK_SHARE, "config", "sim.yaml"),
                "--params-file",
                drone_config,
            ],
            env,
        ),
        _popen(
            [
                "ros2",
                "run",
                "robosub_dummy_publisher",
                "robosub_dummy_publisher_node",
                *ns,
                "--params-file",
                os.path.join(
                    DUMMY_SHARE, "config", "robosub_dummy_publisher_params.yaml"
                ),
                "-p",
                f"tasks:=[{','.join(tasks)}]",
                "-p",
                f"seed:='{seed}'",
            ],
            env,
        ),
        _popen(
            [
                "ros2",
                "run",
                "reference_filter_dp_quat",
                "reference_filter_dp_quat_node",
                *ns,
                "--params-file",
                os.path.join(RF_SHARE, "config", "reference_filter_params.yaml"),
                "--params-file",
                drone_config,
                "-p",
                "altitude_control_enabled:=false",
                "-p",
                "omega:=[1.0,1.0,1.0,1.0,1.0,1.0]",
                "-p",
                "zeta:=[1.0,1.0,1.0,1.0,1.0,1.0]",
            ],
            env,
        ),
        _popen(
            [
                "ros2",
                "run",
                "waypoint_manager",
                "waypoint_manager_node",
                *ns,
                "--params-file",
                drone_config,
            ],
            env,
        ),
    ]

    rclpy.init()
    node = rclpy.create_node("fake_vehicle")
    pose_pub = node.create_publisher(
        PoseWithCovarianceStamped, f"/{NAMESPACE}/pose", qos_profile_sensor_data
    )
    twist_pub = node.create_publisher(
        TwistWithCovarianceStamped, f"/{NAMESPACE}/twist", qos_profile_sensor_data
    )
    odom_pub = node.create_publisher(
        Odometry, f"/{NAMESPACE}/odom", qos_profile_sensor_data
    )

    state = {"pose": [start[0], start[1], start[2], 1.0, 0.0, 0.0, 0.0]}

    def on_reference(msg):
        # A perfect vehicle: it is where the reference says.
        state["pose"] = [msg.x, msg.y, msg.z, msg.qw, msg.qx, msg.qy, msg.qz]

    node.create_subscription(
        ReferenceFilterQuat,
        f"/{NAMESPACE}/guidance/dp_quat",
        on_reference,
        qos_profile_sensor_data,
    )

    def publish_state():
        x, y, z, qw, qx, qy, qz = state["pose"]
        stamp = node.get_clock().now().to_msg()
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = ODOM_FRAME
        pose.header.stamp = stamp
        (
            pose.pose.pose.position.x,
            pose.pose.pose.position.y,
            pose.pose.pose.position.z,
        ) = x, y, z
        (
            pose.pose.pose.orientation.w,
            pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y,
            pose.pose.pose.orientation.z,
        ) = qw, qx, qy, qz
        pose_pub.publish(pose)
        twist = TwistWithCovarianceStamped()
        twist.header.frame_id = ODOM_FRAME
        twist.header.stamp = stamp
        twist_pub.publish(twist)
        odom = Odometry()
        odom.header.frame_id = ODOM_FRAME
        odom.header.stamp = stamp
        odom.pose.pose = pose.pose.pose
        odom_pub.publish(odom)

    node.create_timer(0.05, publish_state)

    scenario_proc = None
    exit_code = None
    try:
        # Let the map fill first, then start the scenario node.
        end = time.monotonic() + warmup
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        scenario_proc = _popen(
            [
                "ros2",
                "run",
                "landmark_targets",
                "landmark_targets_scenario_node",
                *ns,
                "-p",
                f"scenario:={scenario}",
                *extra_params,
            ],
            env,
        )
        end = time.monotonic() + timeout
        while time.monotonic() < end and scenario_proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
        exit_code = scenario_proc.poll()
        settle = time.monotonic() + 1.0
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.05)
        final = list(state["pose"])
    finally:
        node.destroy_node()
        rclpy.shutdown()
        _stop(daemons + ([scenario_proc] if scenario_proc is not None else []))
    return exit_code, final


def _yaw(pose):
    return math.atan2(
        2 * (pose[3] * pose[6] + pose[4] * pose[5]),
        1 - 2 * (pose[5] ** 2 + pose[6] ** 2),
    )


def test_gate_scenario_reaches_the_target_and_drives_through():
    # The vehicle starts 4 m before the gate (x = 4): the scenario approaches
    # 2.5 m in front of the opening of the chosen role (x = 1.5) and drives
    # 5 m through it, clear of the middle post at y ~ 0.
    from robosub_dummy_publisher.course_layout import TASKS, draw_role_picks
    from vortex_msgs.msg import LandmarkSubtype

    seed = "7"
    gate = TASKS["gate"]
    picks, _ = draw_role_picks(seed)
    (panel,) = [
        lm
        for lm in gate.build_landmarks(picks)
        if lm.landmark_subtype == LandmarkSubtype.GATE_SURVEY_REPAIR
    ]
    opening_y = gate.base_pose[1] + panel.offset[1]
    exit_code, final = run_scenario(["gate"], (0.0, 0.0, 2.0), "gate", seed=seed)
    assert exit_code == 0, f"scenario did not succeed (exit code {exit_code})"
    assert final[0] == pytest.approx(6.5, abs=0.5)
    assert final[1] == pytest.approx(opening_y, abs=0.3)
    assert abs(final[1]) > 0.4
    assert abs(_yaw(final)) < math.radians(15.0)


def test_torpedo_scenario_aims_at_the_large_opening_of_the_chosen_role():
    from robosub_dummy_publisher.course_layout import (
        _TORPEDO_CIRCLE_OFFSETS,
        _TORPEDO_ROLE_BY_VERSION,
        draw_role_picks,
    )

    seed = "7"
    picks, _ = draw_role_picks(seed)
    roles = _TORPEDO_ROLE_BY_VERSION.get(
        picks.get("torpedo_board"), _TORPEDO_ROLE_BY_VERSION["Task4_ver1.png"]
    )
    # The scenario picks the large survey/repair opening.
    from vortex_msgs.msg import LandmarkSubtype

    (hole_name,) = [
        n
        for n, sub in roles.items()
        if sub == LandmarkSubtype.TORPEDO_TARGET_LARGE_SURVEY_REPAIR
    ]
    base = TASKS["torpedo_board"].base_pose
    hole = tuple(
        b + o for b, o in zip(base, _TORPEDO_CIRCLE_OFFSETS[hole_name], strict=True)
    )

    exit_code, final = run_scenario(
        ["torpedo_board"], (base[0] - 6.0, hole[1], hole[2]), "torpedo", seed=seed
    )
    assert exit_code == 0, f"scenario did not succeed (exit code {exit_code})"
    # 1.5 m in front of the opening, looking at the board (heading +x).
    assert final[0] == pytest.approx(hole[0] - 1.5, abs=0.5)
    assert final[1] == pytest.approx(hole[1], abs=0.4)
    assert final[2] == pytest.approx(hole[2], abs=0.4)
    assert abs(_yaw(final)) < math.radians(20.0)


def test_bin_scenario_goes_above_the_bin_of_the_chosen_role():
    from robosub_dummy_publisher.course_layout import draw_role_picks
    from vortex_msgs.msg import LandmarkSubtype

    seed = "7"
    picks, _ = draw_role_picks(seed)
    bins = {lm.label: lm for lm in TASKS["bin"].landmarks(picks) if lm.camera == "down"}
    survey = [
        lm
        for lm in bins.values()
        if lm.landmark_subtype == LandmarkSubtype.BIN_SURVEY_REPAIR
    ]
    assert survey, "the seed draws no survey/repair bin"
    base = TASKS["bin"].base_pose
    start = (base[0] - 4.0, base[1], 2.0)
    # With two bins of the same role the scenario takes the nearest one.
    positions = [
        tuple(b + o for b, o in zip(base, lm.offset, strict=True)) for lm in survey
    ]
    target = min(positions, key=lambda p: math.dist(p, start))

    exit_code, final = run_scenario(["bin"], start, "bin", seed=seed)
    assert exit_code == 0, f"scenario did not succeed (exit code {exit_code})"
    assert final[0] == pytest.approx(target[0], abs=0.5)
    assert final[1] == pytest.approx(target[1], abs=0.5)
    assert final[2] == pytest.approx(target[2] - 1.0, abs=0.4)  # 1 m above
