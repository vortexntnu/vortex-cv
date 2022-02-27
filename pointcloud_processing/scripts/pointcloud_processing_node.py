#!/usr/bin/env python

import rospy

# Import msg types
from cv_msgs.msg import PointArray
from geometry_msgs.msg import PointStamped, PoseStamped
from vortex_msgs.msg import ObjectPosition
from sensor_msgs.msg import PointCloud2

# Import classes
from pointcloud_mapping import PointCloudMapping

class PointcloudProcessingNode():
    """
    Handles tasks related to pointcloud processing
    """
    cameraframe_x = 1280    # Param to set the expected width of cameraframe in pixels
    cameraframe_y = 720     # Param to set the expected height of cameraframe in pixels
    use_reduced_pc = False  # Param to change wether or not to use reduced pointcloud data

    def __init__(self):
        rospy.init_node('pointcloud_processing_node')
        # Decide which pointcloud to use, this onsly works on topics described below
        if self.use_reduced_pc:
            self.pointcloud_reducedSub = rospy.Subscriber('/pointcloud_downsize/output', PointCloud2, self.pointcloud_camera_cb)
        else:
            self.pointcloudSub = rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_camera_cb)

        self.feat_detSub = rospy.Subscriber('/feature_detection/object_points', PointArray, self.feat_det_cb)
        

        # Defining classes
        self.pointcloud_mapper = PointCloudMapping()

    def feat_det_cb(self, msg):
        """
        Callback if a message from feature detection node is recieved. Will read through a PointArray message and add each point to a new list.
        It will use this pointlist to get orientation- and positiondata for the detected object, and send this to a remote filter.

        Args:
            msg: The message recieved from feature_detection_node. It should be a PointArray message.
        """
        headerdata = msg.header
        objectID = msg.Class
        
        # Generates an empty list and adds all the point from msg to it
        point_list = []
        for point in msg.point_array:
            point_list.append((point.y, point.x)) # TODO: change back from y, x to x, y as it is supposed to be the latter

        # Calls function to find object centre and orientation
        orientationdata, positiondata = self.pointcloud_mapper.object_orientation_from_point_list(point_list, self.pointcloud_data)
        self.send_position_orientation_data(headerdata, positiondata, orientationdata, objectID)

    def pointcloud_camera_cb(self, msg_data):
        """
        Callback for pointcloud message. Checks to see if message is correct type
        and stores it as a class variable which is updated everytime a new message is recieved.

        Args:
            msg_data: pointcloud-data message
        """
        assert isinstance(msg_data, PointCloud2) # This may be the wrong place to put this
        self.pointcloud_data = msg_data

        # Test
        # point_list = [(100,122),(101,130),(99,128),(102,125)]
        # orientationdata, positiondata = self.pointcloud_mapper.object_orientation_from_point_list(point_list, self.pointcloud_data)
        # self.send_position_orientation_data(self.pointcloud_data.header, positiondata, orientationdata, "objectID")



    def send_position_orientation_data(self, headerdata, positiondata, orientationdata, name):
        """
        Call to send position and orientation data for other nodes

        Args:
            headerdata: header you want to send with (frame)
            positiondata: [x, y, z] floats
            orientationdata: [x, y, z, w] floats
            name: name of detected object. String
        """
        if orientationdata:
            self.send_pose_message(headerdata, positiondata, orientationdata, name)
            self.send_ObjectPosition_message(headerdata, positiondata, orientationdata, name)



    def send_pointStamped_message(self, headerdata, position, name):
        """
        Publishes a PointStamped as a topic under /pointcloud_processing/object_point

        Args:
            headerdata: Headerdata to be used as a header will not be created in this function
            position: A position xyz in the form [x, y, z] where xyz are floats
            name: name to be given to the point published, must not contain special characters.

        Returns:
            Topic:
                /pointcloud_processing/object_point/name where name is your input
        """
        # For testing
        pointPub = rospy.Publisher('/pointcloud_processing/object_point/' + name, PointStamped, queue_size= 1)
        new_point = PointStamped()
        new_point.header = headerdata
        new_point.header.stamp = rospy.get_rostime()
        new_point.point.x = position[0]
        new_point.point.y = position[1]
        new_point.point.z = position[2]
        pointPub.publish(new_point)

    def send_pose_message(self, headerdata, position_data, quaternion_data, name):
        """
        Publishes a PoseStamped as a topic under /pointcloud_processing/object_pose

        Args:
            headerdata: Headerdata to be used as a header will not be created in this function
            position_data: A position xyz in the form [x, y, z] where xyz are floats
            quaternion_data: A quaternion wxyz in the form [w, x, y, z]
            name: name to be given to the point published, must not contain special characters.

        Returns:
            Topic:
                /pointcloud_processing/object_pose/name where name is your input
        """
        posePub = rospy.Publisher('/pointcloud_processing/object_pose_rviz/' + name, PoseStamped, queue_size= 1)
        p_msg = PoseStamped()
        # Format header
        p_msg.header = headerdata
        p_msg.header.stamp = rospy.get_rostime()

        # Build pose
        p_msg.pose.position.x = position_data[0]
        p_msg.pose.position.y = position_data[1]
        p_msg.pose.position.z = position_data[2]
        p_msg.pose.orientation.x = 1
        p_msg.pose.orientation.y = quaternion_data[2]
        p_msg.pose.orientation.z = 1
        p_msg.pose.orientation.w = 1
        posePub.publish(p_msg)

    def send_ObjectPosition_message(self, headerdata, position_data, quaternion_data, name):
        """
        Publishes a PoseStamped as a topic under /pointcloud_processing/object_pose

        Args:
            headerdata: Headerdata to be used as a header will not be created in this function
            position_data: A position xyz in the form [x, y, z] where xyz are floats
            quaternion_data: A quaternion wxyz in the form [w, x, y, z]
            name: string name to be given to the point published, must not contain special characters.

        Returns:
            Topic:
                /pointcloud_processing/object_pose/name where name is your input
        """
        objposePub = rospy.Publisher('/pointcloud_processing/object_pose/' + name, ObjectPosition, queue_size= 1)
        p_msg = ObjectPosition()
        p_msg.objectID = name

        # Build pose
        p_msg.objectPose.pose.position.x = position_data[0]
        p_msg.objectPose.pose.position.y = position_data[1]
        p_msg.objectPose.pose.position.z = position_data[2]
        p_msg.objectPose.pose.orientation.x = 1
        p_msg.objectPose.pose.orientation.y = quaternion_data[2]
        p_msg.objectPose.pose.orientation.z = 1
        p_msg.objectPose.pose.orientation.w = 1
        objposePub.publish(p_msg)

if __name__ == '__main__':
    node = PointcloudProcessingNode()

    while not rospy.is_shutdown():
        rospy.spin()

