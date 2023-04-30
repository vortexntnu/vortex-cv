#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from time import sleep

def publish_image(image_file):
    # Initialize ROS node
    rospy.init_node('image_publisher', anonymous=True)

    # Initialize publisher with topic name and message type
    image_pub = rospy.Publisher('/image', Image, queue_size=1)
    
    # Initialize OpenCV bridge
    bridge = CvBridge()
    
    # Read image from file
    img = cv2.imread(image_file)
    plt.imshow(img)
    while not rospy.is_shutdown():
        # Convert image to ROS message and publish
        image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        # Spin node to keep it alive
        sleep(5)

if __name__ == '__main__':
    # Provide image file path as argument
    image_file = "/home/vortex/projects/beng_beluga_ws/src/vortex-cv/valve_edc/data/valve_office/MicrosoftTeams-image(13).jpeg"
    publish_image(image_file)
