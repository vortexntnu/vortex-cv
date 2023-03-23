#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include <vector>
#include <Eigen/Dense>
#include "eigen_typedefs.h"

#include <vortex_msgs/ObjectPosition.h>
// #include <vortex_msgs/LandmarkPose.h>
#include <aruco_handler.hpp>


class ArucoFilteringNode {

public:
    ArucoFilteringNode();
    /**
     * The callback function for the op_sub-subscriber.
     * @param img_source is the message received on the ROS-topic, containing an object ID and the position of the object.
     * The object ID and position is stored in the objectPositions-map
     * The message received is further published on the object_positions_out-topic.
     */
    void aruco_cb(const geometry_msgs::PoseStamped& msg_in);
    void dvl_alt_cb(const std_msgs::Float32& msg_in),


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
    ros::Subscriber arucoPoseSub;
    ros::Subscriber dvlAltSub;
    ros::Publisher opPosePubTf;
    ros::Publisher opPosePubTfLandmark;
    const float rate = 10;

    Eigen::Vector3d m_aruco_position;
    float m_dvl_alt;
};

