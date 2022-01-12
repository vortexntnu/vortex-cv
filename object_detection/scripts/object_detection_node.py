#!/usr/bin/env python

import rospy

# Import msg types
from darknet_ros_msgs.msg import BoundingBoxes
from cv_msgs.msg import BBox, BBoxes
from geometry_msgs.msg import PointStamped
from rospy.core import rospyinfo
from vortex_msgs.msg import ObjectPosition
from vortex_msgs.msg import PointStampedArray
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

# Import classes
from position_estimator import PositionEstimator
from coord_pos import CoordPosition

class ObjectDetectionNode():
    """Handles tasks related to object detection
    """
    pc_data = 0

    def __init__(self):
        rospy.init_node('object_detection_node')
        self.bboxSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bboxSub_cb_single_lense)
        self.pcSub = rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_cb)
        self.pcPub = rospy.Publisher('/object_detection/bbox_pointcloud', PointCloud2, queue_size= 1)
        self.estimatorPub = rospy.Publisher('/object_detection/size_estimates', BBoxes, queue_size= 1)
        self.landmarkPub = rospy.Publisher('/object_positions_in',ObjectPosition, queue_size= 1)
        self.position_estimator = PositionEstimator()
        self.coord_positioner = CoordPosition()
    
    def point_cloud():
        pass

    def pointcloud_cb(self, data):
        self.pc_data = data
        # assert isinstance(data, PointCloud2)
        # pt_gen = point_cloud2.read_points(data, skip_nans=False, uvs=[1, 2])
        # for pt in pt_gen:
        #     rospy.loginfo(pt)
        return

    def send_pointcloud(self, bbox_data):
        """"""
        x_from = bbox_data.xmin
        x_to = bbox_data.xmax
        y_from = bbox_data.xmin
        y_to = bbox_data.ymax

        # assert isinstance(self.pc_data, PointCloud2)
        # pt_gen = point_cloud2.read_points(self.pc_data, False, [y_from, x_from])
        # for pt in pt_gen:
        #     rospy.loginfo(pt)


        if self.pc_data != 0:
            arrayPosition = x_from*self.pc_data.point_step + y_from*self.pc_data.row_step

            arrayPosX = arrayPosition + self.pc_data.fields[0].offset  # X has an offset of 0
            arrayPosY = arrayPosition + self.pc_data.fields[1].offset  # Y has an offset of 4
            arrayPosZ = arrayPosition + self.pc_data.fields[2].offset  # Z has an offset of 8

            x = self.pc_data.data[arrayPosX]
            y = self.pc_data.data[arrayPosY]
            z = self.pc_data.data[arrayPosZ]
            #x = PointField.deserialize_numpy(x)
            

            self.rviz_point_single_lense(self.pc_data.header, [x,y,z], "pc_test")



    def bboxSub_cb_single_lense(self, data):
        """
        
        """
        # Allocates msg data to local variables in order to process abs size
        ArrayBoundingBoxes = BBoxes()
        ArrayBoundingBoxes.header = data.header
        ArrayBoundingBoxes.image_header = data.image_header

        # Iterate through all the detected objects and estimate sizes
        for bbox in data.bounding_boxes:
            # Sends bbox data to pointcloud func in order to respublish pointcloud data only within bbox
            self.send_pointcloud(bbox)

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

            # Create rviz point
            key_string = str(bbox.Class)
            self.rviz_point_single_lense(data.header, position, key_string)

            
        self.estimatorPub.publish(ArrayBoundingBoxes)

    def rviz_point_single_lense(self, headerdata, position, name):
        # For testing
        for i in position:
            if i != float:
                i = 0.0
        pointPub = rospy.Publisher('/object_detection/object_point/' + name, PointStamped, queue_size= 1)
        new_point = PointStamped()
        new_point.header = headerdata
        new_point.header.stamp = rospy.get_rostime()
        new_point.point.x = position[0]
        new_point.point.y = position[1]
        new_point.point.z = position[2]
        pointPub.publish(new_point)
        # rospy.loginfo(new_point)

    def bboxSub_cb_twin_lense(self, data):
        """
        Gets the data from the subscribed message BoundingBoxes and publishes the size estimates of a detected object, and the position of the object.

        Args:
            data: The message that has been recieved.

        Returns:
            Published topics:
                estimatorPub: Array of detected objects as the estimated size of these. Topic also includes angles to the objects from the camera frame.
                pointPub: A topic is created for each object that is detected. Publishes the point of the object in a coordinate frame.
        """
        # Allocates msg data to local variables in order to process abs size
        ArrayBoundingBoxes = BBoxes()
        ArrayBoundingBoxes.header = data.header
        ArrayBoundingBoxes.image_header = data.image_header

        # Dictionary for saving object to see if the same object is detected twice
        saved_objects = {}

        # Iterate through all the detected objects and estimate sizes
        for bbox in data.bounding_boxes:

            # Store depth measurement of boundingbox
            depth_mtr = bbox.z

            # Get the size estimation from the size estimator class
            position_estimator_data = self.position_estimator.main(bbox)
            length_x_mtr = position_estimator_data[0]
            length_y_mtr = position_estimator_data[1]
            redefined_angle_x = position_estimator_data[2]
            redefined_angle_y = position_estimator_data[3]

            # Build the new bounding box message
            CurrentBoundingBox = BBox()
            CurrentBoundingBox.Class = bbox.Class
            CurrentBoundingBox.probability = bbox.probability
            CurrentBoundingBox.width = length_x_mtr
            CurrentBoundingBox.height = length_y_mtr
            CurrentBoundingBox.z = depth_mtr
            CurrentBoundingBox.centre_angle_x = redefined_angle_x
            CurrentBoundingBox.centre_angle_y = redefined_angle_y

            # Get the position of the object relative to the camera
            position = self.coord_positioner.main(redefined_angle_x, redefined_angle_y, depth_mtr)

            # If the same object is detected in each lense then a point in the middle is calculated
            key_string = str(bbox.Class)
            if (key_string in saved_objects):
                value = saved_objects.get(key_string)
                new_x = (value[0]+position[0])*0.5
                new_y = (value[1]+position[1])*0.5
                new_z = (value[2]+position[2])*0.5
                self.landmarks([new_x,new_y,new_z], key_string)
                saved_objects[str(bbox.Class)] = [new_x, new_y, new_z]
            else:
                saved_objects[str(bbox.Class)] = [position[0], position[1], position[2]]

            self.rviz_point(data, saved_objects, key_string)

            # Append the new message to bounding boxes array
            ArrayBoundingBoxes.bounding_boxes.append(CurrentBoundingBox)

        self.estimatorPub.publish(ArrayBoundingBoxes)

    def rviz_point(self, data, dictionary, key_string):
        """Delete this when no longer needed"""
        # Create a publisher for each detected object. The name of the topic will be the name of the object from class in bounding box message.
        # # Build the message to place detected obejct in relation to camera 

        # pointPub = rospy.Publisher('/object_detection/object_points', PointStampedArray, queue_size= 1)
        # points = PointStampedArray()
        # points.header = data.header
        # points.image_header = data.image_header

        # for i in dictionary:
        #     corrected_point = dictionary.get(key_string)
        #     new_point = PointStamped()
        #     new_point.header = data.header
        #     new_point.header.stamp = rospy.get_rostime()
        #     new_point.point.x = corrected_point[0]
        #     new_point.point.y = corrected_point[1]
        #     new_point.point.z = corrected_point[2]
        #     points.positions.append(new_point)

        # pointPub.publish(points)

        pointPub = rospy.Publisher('/object_detection/object_point/' + key_string, PointStamped, queue_size= 1)
        new_point = PointStamped()
        new_point.header = data.header
        new_point.header.stamp = rospy.get_rostime()
        # new_point.objectID = key_string
        corrected_point = dictionary.get(key_string)
        new_point.point.x = corrected_point[0]
        new_point.point.y = corrected_point[1]
        new_point.point.z = corrected_point[2]
        pointPub.publish(new_point)

    def landmarks(self, position, object_id):
        """
        Builds and sends the ObjectPosition message
        
        Args:
            position: the position in the format of [x, y, z] of the detected object
            object_id: String with name of the object that is detected 

        Returns:
            Published topics:
                landmarkPub: The detected objects position as a landmark.
        """
        landmark = ObjectPosition()
        landmark.objectID = object_id
        landmark.position.x = position[0]
        landmark.position.y = position[1]
        landmark.position.z = position[2]
        self.landmarkPub.publish(landmark)

if __name__ == '__main__':
    node = ObjectDetectionNode()

    while not rospy.is_shutdown():
        rospy.spin()

