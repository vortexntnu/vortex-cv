#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <vortex_msgs/LandmarkPose.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
// #include <opencv2/imgproc.hpp>

#include <vector>

#include "aruco_handler.hpp"

class ArucoDetectionNode {

public:
    ArucoDetectionNode();
    /**
     * The callback function for the op_sub-subscriber.
     * @param img_source is the message received on the ROS-topic, containing an object ID and the position of the object.
     * The object ID and position is stored in the objectPositions-map
     * The message received is further published on the object_positions_out-topic.
     */
    void callback(const sensor_msgs::ImageConstPtr& img_source);


    /**
     * ros::spinOnce() is called at 10Hz
     */
    void execute();

    void publishCVImg(const cv::Mat& img);
    void publishPose(const geometry_msgs::Pose& pose);


protected:
    ros::NodeHandle node;
    ros::Rate loop_rate;
    ros::Subscriber op_sub;
    ros::Publisher op_image_pub;
    ros::Publisher op_pose_pub;

    ArucoHandler arucoHandler;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::Board> board;
};

