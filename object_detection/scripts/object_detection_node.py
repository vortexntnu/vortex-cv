#!/usr/bin/env python

import rospy
# import ros_numpy
import numpy as np

# Import msg types
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from cv_msgs.msg import BBox, BBoxes
from geometry_msgs.msg import PointStamped, PoseStamped
from vortex_msgs.msg import ObjectPosition
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

# Import classes
from position_estimator import PositionEstimator
from coord_pos import CoordPosition
from pointcloud_mapping import PointCloudMapping

class ObjectDetectionNode():
    """
    Handles tasks related to object detection
    """
    def __init__(self):
        rospy.init_node('object_detection_node')
        self.bboxSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bboxSub_cb)
        # self.CVbboxSub = rospy.Subscriber('/gate_detection/BoundingBox', BoundingBox, self.feature_bbox_cb)
        self.lim_pointcloudPub = rospy.Publisher('/object_detection/pointcloud_limited_to_bbox',PointCloud2, queue_size=1) 
        
        self.pointcloudSub = rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_cb)
        self.pointcloud_reducedSub = rospy.Subscriber('/object_detection/output', PointCloud2, self.pointcloud_downsampled_cb)
        
        self.estimatorPub = rospy.Publisher('/object_detection/size_estimates', BBoxes, queue_size= 1) # Needs reevaluation
        
        self.position_estimator = PositionEstimator()
        self.coord_positioner = CoordPosition()
        self.pointcloud_mapper = PointCloudMapping()
    
    def feature_bbox_cb(self, data):
        """
        Args:
            data: bbox msg data
        """
        bounding_box = data
        self.republish_pointcloud_from_bbox(bounding_box)

    def pointcloud_cb(self, msg_data):
        """
        Stores message from a PointCloud2 message into a class variable

        Args:
            msg_data: callback message from subscription

        Returns:
            class variable: self.pointcloud_data
        """
        self.pointcloud_data = msg_data
        # self.object_orientation_from_poincloud(msg_data, 0.3)
    
    def pointcloud_downsampled_cb(self, msg_data):
        self.pointcloud_data_redused = msg_data
        # self.object_orientation_from_poincloud(msg_data, 0.3)

    def republish_pointcloud_from_bbox(self, bounding_box):
        """
        """
        # # get pointcloud and bounding box data
        # newest_msg = self.pointcloud_data       
        # data_from_zed = ros_numpy.numpify(newest_msg)

        # # get limits from bounding box
        # x_min_limit = bounding_box.xmin
        # x_max_limit = bounding_box.xmax
        # y_min_limit = bounding_box.ymin
        # y_max_limit = bounding_box.ymax

        # # limiting pointcloud to bounding_box_size
        # data_from_zed_old = np.array_split(data_from_zed, [x_min_limit], axis=0)[1]
        # data_from_zed_old = np.array_split(data_from_zed_old, [x_max_limit-x_min_limit], axis=0)[0]
        # data_from_zed_old = np.array_split(data_from_zed_old, [y_min_limit], axis=1)[1]
        # data_from_zed_old = np.array_split(data_from_zed_old, [y_max_limit-y_min_limit], axis=1)[0]
        
        # pcd_height, pcd_width = np.shape(data_from_zed_old)
        # msg = ros_numpy.msgify(PointCloud2, data_from_zed_old)

        # msg.header = newest_msg.header
        # msg.height = pcd_height
        # msg.width = pcd_width

        # self.lim_pointcloudPub.publish(msg)
        pass

    def object_orientation_from_point_list(self, point_list, name):
        """
        Uses known points to find object and its orientation
        
        Args:
            point_list: list of points as tuples [(x,y),(x,y)]
            name: name of object as string
        """
        assert isinstance(self.pointcloud_data, PointCloud2)
        generated_list = []
        for point in point_list:
            pt_gen = point_cloud2.read_points(self.pointcloud_data, skip_nans=True, uvs=[[point[0],point[1]]])
            generated_list.append(pt_gen) # needs testing
        
        pose_data, middle_point = self.pointcloud_mapper.points_to_plane(generated_list)
        self.send_pose_message(self.pointcloud_data.header, middle_point, pose_data, name)
        self.send_ObjectPosition_message(self.pointcloud_data.header, middle_point, pose_data, name)

    def object_orientation_from_poincloud(self, pointcloud_data, threshold):
        """
        Uses pointcloud data to find object and its orientation in regards to pointcloud_data frame
        
        Args:
            pointcloud_data: The pointcloud data to analyze
            threshold: maximum distance of expected object dimensions as float cm.mm. Ex: if height is bigger than width input height
        """
        assert isinstance(pointcloud_data, PointCloud2)
        generated_pointcloud_list = []
        closest_point = 200.00
        pt_gen = point_cloud2.read_points(pointcloud_data, skip_nans=True)
        
        for pt in pt_gen:
            tmp_list = list(pt)
            tmp_list = tmp_list[:3]
            if (abs(pt[2]) < closest_point) and (abs(pt[2]) > 0.2):
                closest_point = abs(pt[2])
            generated_pointcloud_list.append(tmp_list)

        object_point_list = []
        for point in generated_pointcloud_list:
            if abs(point[2]) <= (threshold + closest_point):
                object_point_list.append(point)
        
        fit, middle_point = self.pointcloud_mapper.points_to_plane(object_point_list)
        quaternion_data = fit

        self.send_pose_message(pointcloud_data.header, middle_point, quaternion_data, "middle_pose")
    
    def send_pose_message(self, headerdata, position_data, quaternion_data, name):
        """
        Publishes a PoseStamped as a topic under /object_detection/object_pose

        Args:
            headerdata: Headerdata to be used as a header will not be created in this function
            position_data: A position xyz in the form [x, y, z] where xyz are floats
            quaternion_data: A quaternion wxyz in the form [w, x, y, z]
            name: name to be given to the point published, must not contain special characters.

        Returns:
            Topic:
                /object_detection/object_pose/name where name is your input
        """
        posePub = rospy.Publisher('/object_detection/object_pose_rviz/' + name, PoseStamped, queue_size= 1)
        p_msg = PoseStamped()
        # Format header
        p_msg.header = headerdata
        p_msg.header.stamp = rospy.get_rostime()

        # Build pose
        p_msg.pose.position.x = position_data[0]
        p_msg.pose.position.y = position_data[1]
        p_msg.pose.position.z = position_data[2]
        p_msg.pose.orientation.x = 0 # quaternion_data[1]
        p_msg.pose.orientation.y = 0 # quaternion_data[1]
        p_msg.pose.orientation.z = 1 # quaternion_data[0]
        if len(quaternion_data) == 3:
            p_msg.pose.orientation.w = quaternion_data[1]
        else:
            p_msg.pose.orientation.w = 1
        posePub.publish(p_msg)

    def send_ObjectPosition_message(self, headerdata, position_data, quaternion_data, name):
        """
        Publishes a PoseStamped as a topic under /object_detection/object_pose

        Args:
            headerdata: Headerdata to be used as a header will not be created in this function
            position_data: A position xyz in the form [x, y, z] where xyz are floats
            quaternion_data: A quaternion wxyz in the form [w, x, y, z]
            name: string name to be given to the point published, must not contain special characters.

        Returns:
            Topic:
                /object_detection/object_pose/name where name is your input
        """
        objposePub = rospy.Publisher('/object_detection/object_pose/' + name, ObjectPosition, queue_size= 1)
        p_msg = ObjectPosition()

        p_msg.objectID = name

        # Build pose
        p_msg.objectPose.pose.position.x = position_data[0]
        p_msg.objectPose.pose.position.y = position_data[1]
        p_msg.objectPose.pose.position.z = position_data[2]
        p_msg.objectPose.pose.orientation.w = quaternion_data[0]
        p_msg.objectPose.pose.orientation.x = quaternion_data[1]
        p_msg.objectPose.pose.orientation.y = quaternion_data[2]
        if len(quaternion_data) == 3:
            p_msg.objectPose.pose.orientation.z = 1
        else:
            p_msg.objectPose.pose.orientation.z = quaternion_data[3]
        objposePub.publish(p_msg)

    def get_pointcloud_position_of_xy_point(self, x_pixel, y_pixel):
        """
        Reads the point cloud data from a given x, y coordinate

        Args:
            x_pixel: position in x direction of point you want clouddata from
            y_pixel: position in y direction of point you want clouddata from

        Returns:
            Point cloud data for a point in the camera frame as list [x, y, z]
        """
        # Generates a readable version of the point cloud data
        is_pointcloud = isinstance(self.pointcloud_data, PointCloud2)
        if is_pointcloud:
            # Reads the point cloud data at given uvs: u = x cord, v = y cord
            pt_gen = point_cloud2.read_points(self.pointcloud_data, skip_nans=False, uvs=[[x_pixel, y_pixel]])
            for pt in pt_gen:
                self.pointcloud_x = pt[0]
                self.pointcloud_y = pt[1]
                self.pointcloud_z = pt[2]

        x, y, z = self.pointcloud_x, self.pointcloud_y, self.pointcloud_z
        return [x, y, z]

    def object_orientation_from_xy_area(self, area_with_limits):
        """
        Reads the point cloud data from a given area

        Args:
            area_with_limits: list of data [xmin, xmax, ymin, ymax]

        Returns:
            orientation
        """
        # Generates a readable version of the point cloud data
        assert isinstance(self.pointcloud_data, PointCloud2)

        xmin = area_with_limits[0]
        xmax = area_with_limits[1]
        ymin = area_with_limits[2]
        ymax = area_with_limits[3]

        # loops through the area data and adds points to a list
        obj_list = []
        for x in range(xmin  -1, xmax -1):
            for y in range(ymin - 1, ymax - 1):
                pt_gen = point_cloud2.read_points(self.pointcloud_data, skip_nans=True, uvs=[[x,y]])
                for pt in pt_gen:
                    obj_list.append(pt)

        name = "gate"

        pose_data, middle_point = self.pointcloud_mapper.points_to_plane(obj_list)
        self.send_pose_message(self.pointcloud_data.header, middle_point, pose_data, name)
        self.send_ObjectPosition_message(self.pointcloud_data.header, middle_point, pose_data, name)

    def bboxSub_cb(self, data):
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
            bbox.ymin = 376 - bbox.ymax # needs to be updated to automatically read ymax of camera
            bbox.ymax = 376 - temp_ymin

            # self.object_orientation_from_xy_area([bbox.xmin, bbox.xmax, bbox.ymin, bbox.ymax])
            botleft = self.get_pointcloud_position_of_xy_point(bbox.xmin + 50,bbox.ymin + 50)
            topleft = self.get_pointcloud_position_of_xy_point(bbox.xmin + 50,bbox.ymax - 50)
            botright = self.get_pointcloud_position_of_xy_point(bbox.xmax - 50,bbox.ymin + 50)
            topright = self.get_pointcloud_position_of_xy_point(bbox.xmax - 50,bbox.ymax - 50)
            

            self.send_pointStamped_message(ArrayBoundingBoxes.header, botleft, "botleft")
            self.send_pointStamped_message(ArrayBoundingBoxes.header, topleft, "topleft")
            self.send_pointStamped_message(ArrayBoundingBoxes.header, botright, "botright")
            self.send_pointStamped_message(ArrayBoundingBoxes.header, topright, "topright")

            pose_data, middle_point = self.pointcloud_mapper.points_to_plane([botright,botleft,topright,topleft])
            self.send_pose_message(self.pointcloud_data.header, middle_point, pose_data, "name")
            self.send_ObjectPosition_message(self.pointcloud_data.header, middle_point, pose_data, "name")

            rospy.loginfo(pose_data)

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
        Publishes a PointStamped as a topic under /object_detection/object_point

        Args:
            headerdata: Headerdata to be used as a header will not be created in this function
            position: A position xyz in the form [x, y, z] where xyz are floats
            name: name to be given to the point published, must not contain special characters.

        Returns:
            Topic:
                /object_detection/object_point/name where name is your input
        """
        # For testing
        pointPub = rospy.Publisher('/object_detection/object_point/' + name, PointStamped, queue_size= 1)
        new_point = PointStamped()
        new_point.header = headerdata
        new_point.header.stamp = rospy.get_rostime()
        new_point.point.x = position[0]
        new_point.point.y = position[1]
        new_point.point.z = position[2]
        pointPub.publish(new_point)


if __name__ == '__main__':
    node = ObjectDetectionNode()

    while not rospy.is_shutdown():
        rospy.spin()

