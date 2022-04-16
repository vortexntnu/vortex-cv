

## TODO: imports taken from feature detection, remove what is unused

import rospy
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty, String
from cv_msgs.msg import Point2, PointArray
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
from timeit import default_timer as timer
import traceback

import scripts.feature_detection

class PathFollowingNode():
    """
    Node for comperhensive solution to the path following task in RoboSub 2022 on Perception side
    The Node only operates when fsm is in path_search/converge/execute state, and governs the transition between these
    
    3 main problems need to be solved by the node, and they are associated to the possible states for path:

        path_search:        During path search the node uses ZED camera feed to detect the path. After path is detected, the coordinates are published to Autonomous
                            and we request switch to converge.

        path_converge:      During path converge the goal is to center the path in the FOV of the underwater downwards facing camera (in the future refered to as the UDFC)
                            The path is classified in UDFC frame and it's areal centroid is mapped to odom and given as xy reference for the DP controller. The assumption being
                            that this scheme will progressively get the path more and more into FOV, and finally in the centre of UDFC frame. After the path is sufficiently
                            centered we move on to execute.

        path_execute:       During execute the drone keeps station over the path while Perception solves the estimation problem of the path to the next task. Here we can use
                            the fact that we are given the oportunity for multiple measurements and samples to cast the task as a batch problem and extract as much information
                            as possible before we declare a solution. The search for the next task starts when a waypoint is given to Autonomous after we have estimated path 
                            direction.
    """

    def __init__(self):
        rospy.init_node('pointcloud_processing_node')
        
        # Parameters for the algorithm:
        self.hsv_params = ["bruh is equal to 1"] # TODO: BenG help bruh not be equal to one. How does one HSV decide :(
        
        
        self.mission_topic_sub = rospy.Subscriber("/fsm/state", String, self.update_mission)
        self.feat_detSub = rospy.Subscriber('/feature_detection/object_points', PointArray, self.feat_det_cb)
        
        # List of states this algorithm runs in
        self.possible_states = ["path_search", "path_converge", "path_execute"]

        
        # TODO: get image sizes for both ZED and UDFC cameras, these will be used to make the fd objects
        # First initialization of image shape
        first_image_msg = rospy.wait_for_message(image_topic, Image)
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(first_image_msg, "passthrough")
            self.udfc_image_shape = self.cv_image.shape
        except CvBridgeError, e:
            self.image_shape = (720, 1280, 4)


        # Defining classes
        self.udfc_imgfeature_preprocessing = scripts.feature_detection.ImageFeatureProcessing()

    def path_following_zed_cb(self, zed_msg):
        """
        We only use data from the ZED if we are searching for the path

        Find the biggest contour in the image and return its range and bearing
        """
        if self.current_state != "path_search":
            return None

    def path_following_udfc_cb(self, udfc_msg):

        """
        The first time we classify the path in UDFC makes us move to converge state

        Simplest Way to do this: Find biggest contour and if it is above some threshold for area declare the path there

        """

        if self.current_state not in self.possible_states:
            return None

        # Apply HSV to image

        # Get contours and hierarchies

        # Pick the biggest one etc
        

        



if __name__ == '__main__':
    try:
        feature_detection_node = PathFollowingNode(image_topic='/zed2/zed_node/rgb/image_rect_color')
        # rospy.spin()
        feature_detection_node.spin()

    except rospy.ROSInterruptException:
        pass