#!/usr/bin/env python

import rospy
# import numpy as np
# import ros_numpy

# Import msg types
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from cv_msgs.msg import BBox, BBoxes, Point2, PointArray
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from vortex_msgs.msg import ObjectPosition
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

# Import classes
from position_estimator import PositionEstimator
from coord_pos import CoordPosition
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
        self.bboxSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.darknet_cb)
        # self.CVbboxSub = rospy.Subscriber('/gate_detection/BoundingBox', BoundingBox, self.feature_bbox_cb)
        # self.lim_pointcloudPub = rospy.Publisher('/pointcloud_processing/pointcloud_limited_to_bbox',PointCloud2, queue_size=1) 
        self.feat_detSub = rospy.Subscriber('/feature_detection/object_points', PointArray, self.feat_det_cb)
        
        # Decide which pointcloud to use, this onsly works on topics described below
        if self.use_reduced_pc:
            self.pointcloud_reducedSub = rospy.Subscriber('/pointcloud_downsize/output', PointCloud2, self.pointcloud_camera_cb)
        else:
            self.pointcloudSub = rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_camera_cb)
        
        # TODO: Needs reevaluation
        self.estimatorPub = rospy.Publisher('/pointcloud_processing/size_estimates', BBoxes, queue_size= 1) # TODO: Needs reevaluation
        
        # Calling classes from other files
        self.position_estimator = PositionEstimator()
        self.coord_positioner = CoordPosition()
        self.pointcloud_mapper = PointCloudMapping()

    def feat_det_cb(self, msg):
        """
        TODO: fillit
        """
        headerdata = msg.header
        objectID = msg.Class
        
        # Generates an empty list and adds all the point from msg to it
        point_list = []
        for point in msg.point_array:
            point_list.append((point.y, point.x)) # TODO: change back from y, x to x, y as it is supposed to be the latter

        # Calls function to find object centre and orientation
        assert isinstance(self.pointcloud_data, PointCloud2) # TODO: Figure out whether assertion can be done before calling a function or in function
        orientationdata, positiondata = self.pointcloud_mapper.object_orientation_from_point_list(point_list, self.pointcloud_data)
        self.send_position_orientation_data(headerdata, positiondata, orientationdata, objectID)

    def pointcloud_camera_cb(self, msg_data):
        """
        Stores message from a PointCloud2 message into a class variable

        Args:
            msg_data: callback message from subscription

        Returns:
            class variable: self.pointcloud_data
        """
        self.pointcloud_data = msg_data

        # Test
        # point_list = [(100,122),(101,130),(99,128),(102,125)]
        # assert isinstance(self.pointcloud_data, PointCloud2) # TODO: Figure out whether assertion can be done before calling a function or in function
        # orientationdata, positiondata = self.pointcloud_mapper.object_orientation_from_point_list(point_list, self.pointcloud_data)
        # self.send_position_orientation_data(self.pointcloud_data.header, positiondata, orientationdata, "objectID")

    # def feature_bbox_cb(self, data):
    #     """
    #     Args:
    #         data: bbox msg data
    #     """
    #     bounding_box = data
    #     self.republish_pointcloud_from_bbox(bounding_box)

    # def republish_pointcloud_from_bbox(self, bounding_box):
    #     """
    #     TODO: This needs explanation
    #     """
    #     # get pointcloud and bounding box data
    #     newest_msg = self.pointcloud_data       
    #     data_from_zed = ros_numpy.numpify(newest_msg)

    #     # get limits from bounding box
    #     x_min_limit = bounding_box.xmin
    #     x_max_limit = bounding_box.xmax
    #     y_min_limit = bounding_box.ymin
    #     y_max_limit = bounding_box.ymax

    #     # limiting pointcloud to bounding_box_size
    #     data_from_zed_old = np.array_split(data_from_zed, [x_min_limit], axis=0)[1]
    #     data_from_zed_old = np.array_split(data_from_zed_old, [x_max_limit-x_min_limit], axis=0)[0]
    #     data_from_zed_old = np.array_split(data_from_zed_old, [y_min_limit], axis=1)[1]
    #     data_from_zed_old = np.array_split(data_from_zed_old, [y_max_limit-y_min_limit], axis=1)[0]
        
    #     pcd_height, pcd_width = np.shape(data_from_zed_old)
    #     msg = ros_numpy.msgify(PointCloud2, data_from_zed_old)

    #     msg.header = newest_msg.header
    #     msg.height = pcd_height
    #     msg.width = pcd_width

    #     self.lim_pointcloudPub.publish(msg)

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

    def darknet_cb(self, data):
        """
        Gets the data from the subscribed message BoundingBoxes and publishes the size estimates of a detected object, and the position of the object.

        Args:
            data: The message that has been recieved.

        Returns:
            Published topics:
                estimatorPub: Array of detected objects as the estimated size of these. Topic also includes angles to the objects from the camera frame.
        """
        # Allocates msg data to local variables in order to process abs size
        ArrayBoundingBoxes = BBoxes()
        ArrayBoundingBoxes.header = data.header
        ArrayBoundingBoxes.image_header = data.image_header

        # Iterate through all the detected objects and estimate sizes
        for bbox in data.bounding_boxes:

            # Unintuitively position is logged as top to bottom. We fix it so it is from bot to top
            temp_ymin = bbox.ymin
            bbox.ymin = self.cameraframe_y - bbox.ymax # TODO: needs to be updated to automatically read ymax of camera
            bbox.ymax = self.cameraframe_y - temp_ymin

            or_data, pos_data = self.pointcloud_mapper.object_orientation_from_xy_area([bbox.xmin,bbox.xmax,bbox.ymin,bbox.ymax])
            self.send_pointStamped_message(self.pointcloud_data.header, pos_data, "test")

            # Store depth measurement of boundingbox
            depth_mtr = bbox.z

            # Get the size estimation from the size estimator class
            object_estimator_data = self.position_estimator.main(bbox)
            redefined_angle_x = object_estimator_data[2]
            redefined_angle_y = object_estimator_data[3]

            # Build the new bounding box message
            CurrentBoundingBox = BBox()
            CurrentBoundingBox.Class = bbox.Class
            CurrentBoundingBox.probability = bbox.probability
            CurrentBoundingBox.width = 0
            CurrentBoundingBox.height = 0
            CurrentBoundingBox.z = depth_mtr
            CurrentBoundingBox.centre_angle_x = redefined_angle_x
            CurrentBoundingBox.centre_angle_y = redefined_angle_y

            # Get the position of the object relative to the camera
            position = self.coord_positioner.main(redefined_angle_x, redefined_angle_y, depth_mtr)

            # Append the new message to bounding boxes array
            ArrayBoundingBoxes.bounding_boxes.append(CurrentBoundingBox)
            
        self.estimatorPub.publish(ArrayBoundingBoxes)

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

