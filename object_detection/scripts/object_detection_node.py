#!/usr/bin/env python

import rospy

# Import msg types
from darknet_ros_msgs.msg import BoundingBoxes
from cv_msgs.msg import BBox, BBoxes
from geometry_msgs.msg import PointStamped

# Import classes
from size_estimator import SizeEstimator
from coord_pos import CoordPosition

class ObjectDetectionNode():
    """Handles tasks related to object detection
    """

    def __init__(self):
        rospy.init_node('object_detection_node')
        self.estimatorSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.estimatorSub_callback)
        self.estimatorPub = rospy.Publisher('/object_detection/size_estimates', BBoxes, queue_size= 1)
        self.size_estimator = SizeEstimator()
        self.coord_positioner = CoordPosition()

    def estimatorSub_callback(self, data):
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
            size_estimator_data = self.size_estimator.main(bbox)
            length_x_mtr = size_estimator_data[0]
            length_y_mtr = size_estimator_data[1]
            redefined_angle_x = size_estimator_data[2]
            redefined_angle_y = size_estimator_data[3]

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
            vector = self.coord_positioner.main(redefined_angle_x, redefined_angle_y, depth_mtr)

            # If the same object is detected in each lense then a point in the middle is calculated
            key_string = str(bbox.Class)
            if (key_string in saved_objects):
                value = saved_objects.get(key_string)
                new_x = (value[0]+vector[0])*0.5
                new_y = (value[1]+vector[1])*0.5
                new_z = (value[2]+vector[2])*0.5
                saved_objects[str(bbox.Class)] = [new_x, new_y, new_z]
            else:
                saved_objects[str(bbox.Class)] = [vector[0], vector[1], vector[2]]
            
            # rospy.loginfo(saved_objects)

            # Append the new message to bounding boxes array
            ArrayBoundingBoxes.bounding_boxes.append(CurrentBoundingBox)
            
            # Create a publisher for each detected object. The name of the topic will be the name of the object from class in bounding box message. 
            pointPub = rospy.Publisher('/object_detection/object_point/' + str(bbox.Class), PointStamped, queue_size= 1)

            # Build the message to place detected obejct in relation to camera
            new_point = PointStamped()
            new_point.header = data.header
            new_point.header.stamp = rospy.get_rostime()
            corrected_point = saved_objects.get(key_string)
            new_point.point.x = corrected_point[0]
            new_point.point.y = corrected_point[1]
            new_point.point.z = corrected_point[2]

            # Publish message for each detected object --> rethink this, is it smart to have this in the loop, or should a list of point be sendt instead.
            pointPub.publish(new_point)

        self.estimatorPub.publish(ArrayBoundingBoxes)

if __name__ == '__main__':
    node = ObjectDetectionNode()

    while not rospy.is_shutdown():
        rospy.spin()

