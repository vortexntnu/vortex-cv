#include "aruco_detection_node.hpp"

ArucoDetectionNode::ArucoDetectionNode() : loop_rate{10}, tfListener{tfBuffer}, arucoHandler{}
{
	double fx, fy, cx, cy, k1, k2, p1, p2, k3;
	while (!node.getParam("fx", fx))
	{
		ROS_WARN_STREAM("DOCKING_NODE: Can't read camera parameters");
		ros::Duration(3.0).sleep();
	}
	node.getParam("fx", fx);
	node.getParam("fy", fy);
	node.getParam("cx", cx);
	node.getParam("cy", cy);
	node.getParam("k1", k1);
	node.getParam("k2", k2);
	node.getParam("p1", p1);
	node.getParam("p2", p2);
	node.getParam("k3", k3);
	
	ROS_INFO_STREAM("Camera parameters" << fx << fy << cx << cy << k1 << k2 << p1 << p2 << k3);
	cv::Mat cameraMatrix           = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);

	arucoHandler.cameraMatrix           = cameraMatrix;
	arucoHandler.distortionCoefficients = distortionCoefficients;

	opImageSub = node.subscribe("filtered_image", 10, &ArucoDetectionNode::callback, this);

	opImagePub    = node.advertise<sensor_msgs::Image>("aruco_image", 100);
	opPosePubUDFC = node.advertise<geometry_msgs::PoseStamped>("aruco_udfc_pose", 100);
	opPosePubODOM = node.advertise<geometry_msgs::PoseStamped>("aruco_odom_pose", 100);
	opObjPubUDFC  = node.advertise<vortex_msgs::ObjectPosition>("aruco_udfc_obj", 100);
	opObjPubODOM  = node.advertise<vortex_msgs::ObjectPosition>("aruco_odom_obj", 100);

	dictionary = new cv::aruco::Dictionary;
	// dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100); // Vortex Docking plate dictionary
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL); // TAC dictionary


	double markerSize, xDist, yDist;
	std::vector<int> ids;

	while (!node.getParam("markerSize", markerSize))
	{
		ROS_WARN_STREAM("DOCKING_NODE: Can't read aruco board config");
		ros::Duration(3.0).sleep();
	}
	node.getParam("markerSize", markerSize);
	node.getParam("xDist", xDist);
	node.getParam("yDist", yDist);
	node.getParam("ids", ids);
	ROS_INFO_STREAM("IDS: " << ids.at(0));
	board = arucoHandler.createRectangularBoard(markerSize, xDist, yDist, dictionary, ids); // TAC dimensions

	////////////////////////////
	//// Init Transforms ///////
	////////////////////////////
	std::string parentFrame = "odom";
	std::string childFrame  = "udfc_aruco_link";

	// Wait for a transform to be available
	while (!tfBuffer.canTransform(parentFrame, childFrame, ros::Time(0))) {
		geometry_msgs::TransformStamped transform;
		try {
			transform = tfBuffer.lookupTransform(parentFrame, childFrame, ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN_STREAM("DOCKING_NODE:" << ex.what());

			ros::Duration(3.0).sleep();
			continue;
		}
	}
	ROS_INFO_STREAM("DOCKING_NODE: Transform between " << parentFrame << " and " << childFrame << " found.");
}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr &img_source)
{
	// Input
	const cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(img_source, sensor_msgs::image_encodings::BGR8);

	cv::Mat img = cvImage->image;
	if (img.empty()) {
		ROS_INFO_STREAM("DOCKING_NODE: Empty image");
		return;
	}

	// Detect and publish pose
	geometry_msgs::Pose pose;
	cv::Mat modifiedImg;

	size_t markersDetected = arucoHandler.detectBoardPose(img, modifiedImg, board, pose);

	if (markersDetected > 1)
		publishPose(pose, cvImage->header.stamp);

	publishCVImg(modifiedImg, cvImage->header.stamp);
}

void ArucoDetectionNode::publishCVImg(const cv::Mat &img, ros::Time timestamp = ros::Time::now())
{
	static size_t counter{0};
	cv_bridge::CvImage imgBridge;
	sensor_msgs::Image imgMsg;

	std_msgs::Header header;
	header.seq   = counter++;
	header.stamp = timestamp;
	imgBridge    = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
	imgBridge.toImageMsg(imgMsg);
	opImagePub.publish(imgMsg);
}

void ArucoDetectionNode::publishPose(const geometry_msgs::Pose &pose, ros::Time timestamp = ros::Time::now())
{
	vortex_msgs::ObjectPosition vortexPoseMsg;
	vortexPoseMsg.objectID          = "docking_point";
	vortexPoseMsg.isDetected        = true;
	vortexPoseMsg.estimateConverged = true;
	vortexPoseMsg.estimateFucked    = false;

	static size_t counter{0};
	geometry_msgs::PoseStamped poseMsg;
	poseMsg.header.frame_id = "udfc_aruco_link";
	poseMsg.header.seq      = counter++;
	poseMsg.header.stamp    = timestamp;
	poseMsg.pose            = pose;

	vortexPoseMsg.objectPose = poseMsg;
	opObjPubUDFC.publish(vortexPoseMsg);
	opPosePubUDFC.publish(vortexPoseMsg.objectPose);

	// Transform udfc pose to world frame

	try {
		odom_udfc_transform = tfBuffer.lookupTransform("odom", "udfc_aruco_link", timestamp);
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN_STREAM(ex.what());
	}

	geometry_msgs::Pose poseTF;
	tf2::doTransform(pose, poseTF, odom_udfc_transform);

	vortexPoseMsg.objectPose.header.frame_id = "odom";
	vortexPoseMsg.objectPose.pose            = poseTF;
	opObjPubODOM.publish(vortexPoseMsg);
	opPosePubODOM.publish(vortexPoseMsg.objectPose);
}

void ArucoDetectionNode::execute()
{
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ArucoDetectionNode");

	ArucoDetectionNode arucoNode;
	arucoNode.execute();
}