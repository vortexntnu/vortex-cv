class PointcloudProcessingNode():
    """

    Handles tasks related to object detection
    """
    def __init__(self):
        
        # self.pcSub = rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_cb)
        # self.pointcloudRedSub = rospy.Subscriber('/object_detection/output', PointCloud2, self.pointcloud_downsampled_cb)
        # self.landmarkPub = rospy.Publisher('/object_detection/object_positions_in',ObjectPosition, queue_size= 1)
        pass

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

            self.rviz_point_old(data, saved_objects, key_string)

            # Append the new message to bounding boxes array
            ArrayBoundingBoxes.bounding_boxes.append(CurrentBoundingBox)

        self.estimatorPub.publish(ArrayBoundingBoxes)

    def rviz_point_old(self, data, dictionary, key_string):
        """Delete this when no longer needed"""

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