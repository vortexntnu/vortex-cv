#!/usr/bin/env python

import rospy

# msg types
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

# classes
from confidence_mapping import ConfidenceMapping


class PreprocessingNode():
    """
    Class to handle operations related to the preprocessing node. \n
    This includes:
        - Confidence map --> masked confidence map
        - depth_registered --> confident depth_registered
        - image_rect_color_filtered --> confident image_rect_color_filtered
        - cloud_registered --> confident cloud_registered
    """

    def __init__(self):
        rospy.init_node('preprocessing_node')

        self.bridge = CvBridge()
        self.confMap = ConfidenceMapping()

        # Confidence map
        rospy.Subscriber(
            '/zed2/zed_node/confidence/confidence_map',
            Image,
            self.confidence_cb)
        self.maskedMapImagePub = rospy.Publisher(
            '/cv/preprocessing/confidence_map_masked', Image, queue_size=1)

        # Depth Registered
        rospy.Subscriber(
            '/zed2/zed_node/depth/depth_registered',
            Image,
            self.depth_registered_cb)
        self.confident_depthPub = rospy.Publisher(
            '/cv/preprocessing/depth_registered', Image, queue_size=1)

        # Rectified color image
        rospy.Subscriber(
            '/zed2/zed_node/rgb/image_rect_color',
            Image,
            self.image_rect_color_cb)
        self.confident_rectImagePub = rospy.Publisher(
            'cv/preprocessing/image_rect_color_filtered', Image, queue_size=1)

        # Pointcloud
        rospy.Subscriber(
            '/zed2/zed_node/point_cloud/cloud_registered',
            PointCloud2,
            self.pointcloud_cb)
        self.confident_pointcloudPub = rospy.Publisher(
            'cv/preprocessing/cloud_registered', PointCloud2, queue_size=1)

    def confidence_cb(self, msg):
        """
        Gets a confidence map from camera through subscription
        and creates a mask where confidence above a threshold is set to 1 and below is set to 0.
        The masked confidence map is the same size as the original but includes only 0 and 1 as unique values.

        A topic with the masked data is published. The result is an Image where only the pixels that are over the threshold are included

        Args:
            msg: confidence map message from camera. Type: Image message.
        """
        # Bridge image data from Image to cv_image data
        cv_image = self.bridge_to_cv(msg)

        # Make the masked map and store it in a class variable
        self.maskedMap, masked_as_cv_image = self.confMap.create_mask(
            cv_image, 3)

        # Bridge image data from cv_image to Image data
        ros_image = self.bridge_to_image(masked_as_cv_image)
        self.maskedMapImagePub.publish(ros_image)

    def pointcloud_cb(self, msg):
        """
        ***Callback***\n
        Pyblishes a confident representation of the message in the topic using a confidence map.

        Args:
            msg: the message in the topic callback
        """
        try:
            masked_map = self.maskedMap
        except AttributeError:
            return

        confident_pointcloud = self.confMap.add_mask_to_pointcloud(
            masked_map, msg)

        confident_pointcloud.header = msg.header
        confident_pointcloud.height = msg.height
        confident_pointcloud.width = msg.width
        self.confident_pointcloudPub.publish(confident_pointcloud)

    def depth_registered_cb(self, msg):
        """
        ***Callback***\n
        Pyblishes a confident representation of the message in the topic using a confidence map. !!!!!!!!!!!!!! Publishes instead pyblishes

        Args:
            msg: the message in the topic callback
        """
        cv_image = self.bridge_to_cv(msg)

        try:
            masked_map = self.maskedMap
        except AttributeError:
            return

        confident_depth = self.confMap.add_mask_to_cv_image(
            masked_map, cv_image)
        ros_image = self.bridge_to_image(confident_depth)
        self.confident_depthPub.publish(ros_image)

    def image_rect_color_cb(self, msg):
        """
        ***Callback***\n
        Pyblishes a confident representation of the message in the topic using a confidence map.

        Args:
            msg: the message in the topic callback
        """
        cv_image = self.bridge_to_cv(msg)

        try:
            masked_map = self.maskedMap
        except AttributeError:
            return

        confident_depth = self.confMap.add_mask_to_cv_image(
            masked_map, cv_image)
        ros_image = self.bridge_to_image(confident_depth, "bgra8")
        self.confident_rectImagePub.publish(ros_image)

    def bridge_to_cv(self, image_msg, encoding="passthrough"):
        """
        This function returns a cv image from a ros image

        Args:
            image_msg: the image to convert to cv
            encoding: type of encoding to be used

        Returns:
            image_transformed: ros image converted to cv_image
        """
        # Bridge image data from Image to cv_image data
        image_transformed = None
        try:
            image_transformed = self.bridge.imgmsg_to_cv2(image_msg, encoding)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        return image_transformed

    def bridge_to_image(self, cv_image_msg, encoding="passthrough"):
        """
        This function returns a ros image from a cv image

        Args:
            cv_image_msg: the cv_image to convert to ros image
            encoding: type of encoding to be used

        Returns:
            image_transformed: cv_image converted to ros image
        """
        # Bridge image data from CV image to Image data
        image_transformed = None
        try:
            image_transformed = self.bridge.cv2_to_imgmsg(
                cv_image_msg, encoding)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        return image_transformed


if __name__ == '__main__':
    node = PreprocessingNode()

    while not rospy.is_shutdown():
        rospy.spin()
