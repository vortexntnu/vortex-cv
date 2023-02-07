#!/usr/bin/env python

import rospy

import numpy as np
import ros_numpy

# Import msg types
from cv_msgs.msg import BBoxes, BBox
from sensor_msgs.msg import PointCloud2

# Import classes
from coord_pos import CoordPosition
from position_estimator import PositionEstimator

class BoundingBoxProcessingNode():
    """
    Handles tasks related to pointcloud processing
    """
    cameraframe_x = 1280    # Param to set the expected width of cameraframe in pixels
    cameraframe_y = 720     # Param to set the expected height of cameraframe in pixels

    def __init__(self):
        rospy.init_node('boundingbox_processing_node')
        self.pointcloudSub = rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_camera_cb)
        self.bboxSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BBoxes, self.darknet_cb)

        # subscriber and publisher for limiting pointcloud to bounding box
        self.CVbboxSub = rospy.Subscriber('/gate_detection/BoundingBox', BBoxes, self.feature_bbox_cb)
        self.lim_pointcloudPub = rospy.Publisher('/pointcloud_processing/pointcloud_limited_to_bbox',PointCloud2, queue_size=1) 
        
        # TODO: Needs reevaluation
        self.estimatorPub = rospy.Publisher('/pointcloud_processing/size_estimates', BBoxes, queue_size= 1) # TODO: Needs reevaluation

        # Calling classes from other files
        self.position_estimator = PositionEstimator()
        self.coord_positioner = CoordPosition()

    def pointcloud_camera_cb(self, msg_data):
        """
        Stores message from a PointCloud2 message into a class variable

        Args:
            msg_data: callback message from subscription

        Returns:
            class variable: self.pointcloud_data
        """
        self.pointcloud_data = msg_data

    def feature_bbox_cb(self, data):
        """
        Args:
            data: bbox msg data
        """
        bounding_box = data
        self.republish_pointcloud_from_bbox(bounding_box)

    def republish_pointcloud_from_bbox(self, bounding_box):
        """
        Limits pointcloud data to size of detected bounding box and republishs this

        Args: 
            bounding_box: boundingbox message from subscription 

        Returns: 
            Published topics:
                lim_pointcloudPub: Limited pointcloud data to the size of the bounding boxes
        """
        # get pointcloud and bounding box data
        newest_msg = self.pointcloud_data   

        # converts pointcloud data into numpy array
        data_from_zed = ros_numpy.numpify(newest_msg)

        # get limits from bounding box
        x_min_limit = bounding_box.xmin
        x_max_limit = bounding_box.xmax
        y_min_limit = bounding_box.ymin
        y_max_limit = bounding_box.ymax

        # limiting pointcloud to bounding_box_size
        data_from_zed_old = np.array_split(data_from_zed, [x_min_limit], axis=0)[1]
        data_from_zed_old = np.array_split(data_from_zed_old, [x_max_limit-x_min_limit], axis=0)[0]
        data_from_zed_old = np.array_split(data_from_zed_old, [y_min_limit], axis=1)[1]
        data_from_zed_old = np.array_split(data_from_zed_old, [y_max_limit-y_min_limit], axis=1)[0]
        
        # converts data back into pointcloud message 
        pcd_height, pcd_width = np.shape(data_from_zed_old)
        msg = ros_numpy.msgify(PointCloud2, data_from_zed_old)
        msg.header = newest_msg.header
        msg.height = pcd_height
        msg.width = pcd_width

        # republish limited pointcloud
        self.lim_pointcloudPub.publish(msg)

    def darknet_cb(self, data):
        """
        Gets the data from the subscribed message BBoxes and publishes the size estimates of a detected object, and the position of the object.

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


if __name__ == '__main__':
    node = BoundingBoxProcessingNode()

    while not rospy.is_shutdown():
        rospy.spin()
