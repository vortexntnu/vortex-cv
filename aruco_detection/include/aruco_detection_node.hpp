#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/imgproc.hpp>
#include <vector>
#include <vortex_msgs/ObjectPosition.h>
// #include <vortex_msgs/LandmarkPose.h>
#include <aruco_handler.hpp>


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

    void publishCVImg(const cv::Mat& img, ros::Time timestamp);
    void publishPose(const geometry_msgs::Pose& pose, ros::Time timestamp);


protected:
    // ROS stuff
    ros::NodeHandle node;
    ros::Rate loop_rate;
    ros::Subscriber opSub;
    ros::Publisher opImagePub;
    ros::Publisher opPosePub;
    ros::Publisher opPosePubTf;
    ros::Publisher opPosePubTfLandmark;

    // ArUco stuff
    ArucoHandler arucoHandler;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::Board> board;

    // TF stuff
    geometry_msgs::TransformStamped odom_udfc_transform;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;
    tf2_ros::TransformBroadcaster tfBroadcaster;
};

