"""The field-of-view filter: what the cameras could see from the vehicle pose."""

import math

import pytest

rclpy = pytest.importorskip("rclpy")
from nav_msgs.msg import Odometry  # noqa: E402
from robosub_dummy_publisher.course_layout import TASKS  # noqa: E402
from robosub_dummy_publisher.robosub_dummy_publisher_node import (  # noqa: E402
    RobosubDummyPublisherNode,
)


@pytest.fixture(scope="module")
def node():
    rclpy.init(args=["--ros-args", "-p", "use_field_of_view:=true"])
    n = RobosubDummyPublisherNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _pose(node, x, y, z, yaw=0.0):
    odom = Odometry()
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = z
    odom.pose.pose.orientation.z = math.sin(yaw / 2)
    odom.pose.pose.orientation.w = math.cos(yaw / 2)
    node._on_odom(odom)


def _landmarks(task, camera="front"):
    base = TASKS[task].base_pose
    return [
        (lm, tuple(b + o for b, o in zip(base, lm.offset, strict=True)))
        for lm in TASKS[task].landmarks({})
        if lm.camera == camera
    ]


def test_nothing_is_visible_before_odometry(node):
    node._vehicle = None
    lm, pos = _landmarks("gate")[0]
    assert not node._visible(lm, *pos)


def test_gate_is_seen_in_front_but_not_from_behind(node):
    lm, pos = next(x for x in _landmarks("gate") if x[0].label == "gate")
    _pose(node, 0.0, 0.0, 2.0, yaw=0.0)  # facing +x, gate at x = 4
    assert node._visible(lm, *pos)
    _pose(node, 8.0, 0.0, 2.0, yaw=0.0)  # gate is behind
    assert not node._visible(lm, *pos)
    _pose(node, 8.0, 0.0, 2.0, yaw=math.pi)  # turned around
    assert node._visible(lm, *pos)


def test_range_and_bearing_limits(node):
    lm, pos = next(x for x in _landmarks("gate") if x[0].label == "gate")
    _pose(node, pos[0] - 9.0, pos[1], pos[2], yaw=0.0)  # 9 m > 8 m range
    assert not node._visible(lm, *pos)
    _pose(node, pos[0] - 5.0, pos[1], pos[2], yaw=math.radians(60))  # 60 deg > 45
    assert not node._visible(lm, *pos)
    _pose(node, pos[0] - 5.0, pos[1], pos[2], yaw=math.radians(30))
    assert node._visible(lm, *pos)


def test_down_camera_sees_only_what_is_below(node):
    lm, pos = _landmarks("bin", camera="down")[0]
    _pose(node, pos[0], pos[1], pos[2] - 1.0)  # 1 m above the bin
    assert node._visible(lm, *pos)
    _pose(node, pos[0] + 3.0, pos[1], pos[2] - 1.0)  # 3 m to the side
    assert not node._visible(lm, *pos)
    _pose(node, pos[0], pos[1], pos[2] + 1.0)  # below the bin
    assert not node._visible(lm, *pos)
