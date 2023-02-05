#!/usr/bin/python3

import rospy
import numpy as np
import yaml
from track_manager import TRACK_MANAGER, TRACK_STATUS

from geometry_msgs.msg import PoseArray, PoseWithCovariance, Pose, Point, Quaternion, TwistWithCovariance, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

"""
Estimate position and velocity for each boat realtive to the vessel (given measurements for the boats position).
"""


class Tracker:
    """
    Nodes created: Tracker
    Subscribes to: lidar_clusters of type PoseArray. Will only read pose.x, pose.y and header.stamp.sec.
    Publishes to: tracked_cv_object of type nav_msgs/odometry. Will write to header.stamp, msg.pose.pose.position.x, 
            msg.pose.pose.position.y, pose.pose.orientation around z-axis, twist.twist.linear.x, twist.twist.linear.y.

    """

    def __init__(self):

        rospy.init_node("Tracker")
        rospy.Subscriber("lidar_clusters", PoseArray, self.cb)
        self.pub = rospy.Publisher("tracked_cv_object", Odometry, queue_size=10)

        #OBS: temp solution
        with open(
            "/home/hannahcl/Documents/vortex/ros_ws/src/vortex-cv/traking/scripts/config_traking_sys.yaml",
            "r",
        ) as stream:
            config_loaded = yaml.safe_load(stream)

        self.track_manager = TRACK_MANAGER(config_loaded)

        self.time_step = 0.1
        self.prev_time = 0
        self.observations = None

    def cb(self, msg):

        self.unpack_pose_array_msg(msg)
        self.track_manager.cb(self.observations, self.time_step)
        if (
            self.track_manager.main_track.track_status == TRACK_STATUS.confirmed
            or self.track_manager.main_track.track_status
            == TRACK_STATUS.tentative_delete
        ):
            self.publish()

    def publish(self):
        odometry_msg = self.pack_odometry_msg()
        self.pub.publish(odometry_msg)

    def unpack_pose_array_msg(self, msg):

        self.time_step = msg.header.stamp.secs - self.prev_time
        self.prev_time = msg.header.stamp.secs

        observations_list = []
        for pose in msg.poses:
            observations_list.append(np.array([pose.position.x, pose.position.y]))

        self.observations = np.array(observations_list)

    def pack_odometry_msg(self):
        msg = Odometry()
        msg.pose = PoseWithCovariance()
        msg.pose.pose= Pose()
        msg.pose.pose.position = Point()
        msg.twist = TwistWithCovariance()
        msg.twist.twist = Twist()
        msg.twist.twist.linear = Vector3()
        msg.header = Header()


        # - - - - header
        msg.header.stamp = rospy.get_rostime()
        #msg.header.seq = ?
        #msg.header.frame_id = ?

        #- - - -  position
        x = self.track_manager.main_track.pdaf.state_post.reshape(4,)

        msg.pose.pose.position.x = x[0]
        msg.pose.pose.position.y = x[1]

        P_post = self.track_manager.main_track.pdaf.P_post
        # msg.pose.covariance[0][0] = P_post[0][0] #variance in x
        # msg.pose.covariance[1][1] = P_post[1][1] #variance in y

        #- - - -  orientation
        theta = np.arctan2(x[3], x[2]) #rotation about z-axis
        msg.pose.pose.orientation = Quaternion(0,0,theta, 0)
        #variance of rotation about z-axis ???

        #- - - -  velocity
        msg.twist.twist.linear.x = x[2]
        msg.twist.twist.linear.y = x[3]
        # msg.twist.covariance[0][0] = P_post[2][2]
        # msg.twist.covariance[1][1] = P_post[3][3]

        return msg



if __name__ == "__main__":
    try:
        tracker = Tracker()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
