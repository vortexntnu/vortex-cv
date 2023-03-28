#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge

class ImagePublisher():
    def __init__(self):
        
        rospy.init_node('image_publisher_node')
        self.imPub = rospy.Publisher("/cv/image_preprocessing/CLAHE/udfc", Image, queue_size=1)

        self.firstImPub = rospy.Publisher("/cv/image_preprocessing/CLAHE_single/udfc", Image, queue_size=1)
        
        self.bridge = CvBridge()
        self.img = cv.imread("./data/path_bendy_full.png")
        self.ros_img = self.bridge.cv2_to_imgmsg(self.img, encoding='bgr8')
        
        self.firstImPub.publish(self.ros_img)

    def publish(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.imPub.publish(self.ros_img)
            rate.sleep()

if __name__ == '__main__':
    try:
        image_publish = ImagePublisher()
        image_publish.publish()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass