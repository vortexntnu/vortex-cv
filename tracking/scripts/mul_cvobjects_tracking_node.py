#!/usr/bin/python3

import rospy
import rospkg
import numpy as np
import yaml
from tf.transformations import quaternion_from_euler
from track_manager_multiple_tracks import MultiTargetTrackManager, PDAF2MN


from geometry_msgs.msg import (
    PoseArray,
    PoseWithCovariance,
    Pose,
    Point,
    Quaternion,
    TwistWithCovariance,
    Twist,
    Vector3,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from cv_msgs.msg import OdometryArray

"""
Estimate position and velocity for each object realtive to the vessel (given measurements for the objects position).
"""


class Tracker:
    """
    Nodes created: MultiTargetTracker
    Subscribes to: lidar_clusters of type PoseArray. Will only read pose.x, pose.y and header.stamp.sec.
    Publishes to: mul_tracked_cv_objects of type nav_msgs/odometry array. Will write to header.stamp, msg.pose.pose.position.x,
            msg.pose.pose.position.y, pose.pose.orientation around z-axis, twist.twist.linear.x, twist.twist.linear.y.

    """

    def __init__(self):

        rospy.init_node("MultiTargetTracker")
        rospy.Subscriber("/lidar/clusters", PoseArray, self.cb)
        self.pub = rospy.Publisher(
            "/tracking/mul_tracked_cv_objects", OdometryArray, queue_size=10
        )

        self.seq = 0

        r = rospkg.RosPack()
        path = r.get_path("tracking")
        rospy.loginfo(path)
        with open(
            path + "/config/config_traking_sys.yaml",
            "r",
        ) as stream:
            config_loaded = yaml.safe_load(stream)

        self.track_manager = MultiTargetTrackManager(config_loaded)

        self.time_step = 0.1
        self.prev_time = 0
        self.observations = None

    def cb(self, msg):

        self.unpack_pose_array_msg(msg)
        self.track_manager.step_once(self.observations, self.time_step)
        if len(self.track_manager.confirmed_tracks) > 0:
            rospy.loginfo('%d objects are being tracked', len(self.track_manager.confirmed_tracks))
            rospy.loginfo('%d potatial new tracks', len(self.track_manager.tentative_tracks))
            self.publish()

    # def publish(self):
    #     for track in self.track_manager.confirmed_tracks:
    #         odometry_msg = self.pack_odometry_msg(track)
    #         self.pub.publish(odometry_msg)

    def publish(self):

        
        odometry_msgs = []
        for track in self.track_manager.confirmed_tracks:
            odometry_msg = self.pack_odometry_msg(track)
            odometry_msgs.append(odometry_msg)

        rospy.loginfo('before odom array')

        odometry_array_msg = OdometryArray()

        rospy.loginfo('after odom array')
        odometry_array_msg.header = Header()
        odometry_array_msg.header.stamp = rospy.get_rostime()
        odometry_array_msg.header.seq = self.seq
        self.seq += 1

        # Add odometry messages to the array message
        odometry_array_msg.odometry_array = odometry_msgs

        self.pub.publish(odometry_array_msg)

    def unpack_pose_array_msg(self, msg):

        self.time_step = msg.header.stamp.secs - self.prev_time
        self.prev_time = msg.header.stamp.secs

        observations_list = []
        for pose in msg.poses:
            observations_list.append(np.array([-pose.position.x, -pose.position.y]))

        self.observations = np.array(observations_list)

    def pack_odometry_msg(self, track: PDAF2MN):
        msg = Odometry()
        msg.pose = PoseWithCovariance()
        msg.pose.pose = Pose()
        msg.pose.pose.position = Point()
        msg.pose.pose.orientation = Quaternion()
        msg.twist = TwistWithCovariance()
        msg.twist.twist = Twist()
        msg.twist.twist.linear = Vector3()
        msg.header = Header()

        # - - - - header
        msg.header.stamp = rospy.get_rostime()
        msg.header.seq = self.seq
        self.seq += 1
        msg.header.frame_id = 'os_lidar'

        # - - - -  position
        x = track.pdaf.posterior_state_estimate.mean.reshape(
            4,
        )
        msg.pose.pose.position.x = x[0]
        msg.pose.pose.position.y = x[1]

        # - - - -  orientation
        theta = np.arctan2(x[3], x[2])  # rotation about z-axis
        q = quaternion_from_euler(0, 0, theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # - - - -  velocity
        msg.twist.twist.linear.x = x[2]
        msg.twist.twist.linear.y = x[3]

        return msg


if __name__ == "__main__":
    try:
        tracker = Tracker()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass