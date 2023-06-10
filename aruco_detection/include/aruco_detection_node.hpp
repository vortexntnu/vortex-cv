#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h> // Read and publish images

#include <geometry_msgs/TransformStamped.h>      // For frame transforms
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For frame transforms
#include <tf2_ros/buffer.h>                      // For frame transforms
#include <tf2_ros/transform_broadcaster.h>       // For frame transforms
#include <tf2_ros/transform_listener.h>          // For frame transforms

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/imgproc.hpp>
#include <vector>
#include <vortex_msgs/ObjectPosition.h>
// #include <vortex_msgs/LandmarkPose.h>
#include <aruco_handler.hpp>
#include <image_processing.hpp>
#include <filter_params_rqt.hpp>

class ArucoDetectionNode {

public:
	ArucoDetectionNode();
	/**
	 * The callback function for the op_sub-subscriber.
	 */
	void callback(const sensor_msgs::ImageConstPtr &img_source);

	/**
	 * ros::spinOnce() is called at 10Hz
	 */
	void execute();

	void publishCVImg(const cv::Mat &img, ros::Time timestamp);
	void publishPose(const geometry_msgs::Pose &pose, ros::Time timestamp);

protected:
	// ROS stuff
	ros::NodeHandle node;
	ros::Rate loop_rate;
	ros::Subscriber opImageSub;
	ros::Publisher opImagePub;
	ros::Publisher opPosePubUDFC;
	ros::Publisher opPosePubODOM;
	ros::Publisher opObjPubUDFC;
	ros::Publisher opObjPubODOM;
	// ros::Publisher opPosePubTfLandmark; //Moved to vision_kf

	// ArUco stuff
	ArucoHandler arucoHandler;
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::Board> board;

	// TF stuff
	geometry_msgs::TransformStamped odom_udfc_transform;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	tf2_ros::TransformBroadcaster tfBroadcaster;

	// rqt stuff
	FilterParams_rqt filterParams;
};
