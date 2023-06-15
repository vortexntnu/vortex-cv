#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int64.h>

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <file_handler.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

class ArucoIdNode {

public:
	ArucoIdNode();
	/**
	 * The callback function for the op_sub-subscriber.
	 */
	void callback(const sensor_msgs::ImageConstPtr &img_source);

	/**
	 * ros::spinOnce() is called at 10Hz
	 */
	void execute();

	void publishCVImg(const cv::Mat &img, ros::Time timestamp);
	void publishId(int id);

protected:
	// ROS stuff
	ros::NodeHandle node;
	ros::Rate loop_rate;
	ros::Subscriber opImageSub;
	ros::Publisher opImagePub;
	ros::Publisher opIdPub;

	// ArUco stuff
	cv::Ptr<cv::aruco::Dictionary> dictionary;

	//
	std::vector<int> storedIds;
};
