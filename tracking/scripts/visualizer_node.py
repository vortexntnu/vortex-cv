#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdometryToPath:
    def __init__(self):
        rospy.init_node('visualizer_node')
        self.path_publisher = rospy.Publisher('nav_msgs/Path', Path, queue_size=10)
        rospy.Subscriber('/tracking/tracked_cv_object', Odometry, self.odometry_callback)
        #rospy.Subscriber('/tracking/mul_tracked_cv_objects', Odometry, self.odometry_callback)
        self.path = Path()
        self.path.header.frame_id = 'os_lidar' 
        rospy.loginfo('init ok')

    def odometry_callback(self, odometry):

        pose_stamped = PoseStamped()
        pose_stamped.pose = odometry.pose.pose
        pose_stamped.header = odometry.header
        self.path.poses.append(pose_stamped)
        self.path_publisher.publish(self.path)


if __name__ == '__main__':
    try:
        odometry_to_path = OdometryToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
