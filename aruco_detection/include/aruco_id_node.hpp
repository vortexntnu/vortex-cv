#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <vector>


class ArucoIdNode {

public:
    ArucoIdNode();
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


protected:
    // ROS stuff
    ros::NodeHandle node;
    ros::Rate loop_rate;
    ros::Subscriber opSub;
    ros::Publisher opImagePub;

    // ArUco stuff
    cv::Ptr<cv::aruco::Dictionary> dictionary;

};

