#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from time import sleep

from rospkg import RosPack

def publish_image(image_file):
    # Initialize ROS node
    rospy.init_node('image_publisher', anonymous=True)

    # Initialize publisher with topic name and message type
    image_pub = rospy.Publisher('/image', Image, queue_size=1)

    # Initialize OpenCV bridge
    bridge = CvBridge()

    # Read image from file
    # plt.imshow(img)
    i = 1
    while not rospy.is_shutdown():
        # Convert image to ROS message and publish
        image_file = image_file.replace(str(i), str(i+1))
        rospy.loginfo(image_file)
        i += 1
        img = cv2.imread(image_file)
        image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        # Spin node to keep it alive
        sleep(10)


if __name__ == '__main__':
    # Provide image file path as argument
    rp = RosPack()
    path = rp.get_path("valve_edc")
    image_file = path + "/data/valve_office/MicrosoftTeams-image(1).jpeg"
    publish_image(image_file)
