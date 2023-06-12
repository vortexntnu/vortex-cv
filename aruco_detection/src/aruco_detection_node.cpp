#include "aruco_detection_node.hpp"

ArucoDetectionNode::ArucoDetectionNode() : loop_rate{10}, tfListener{tfBuffer}, arucoHandler{}
{
	// udfc
	double fx = 492.0642427973538, fy = 491.6400793457974, cx = 428.50825789526795, cy = 253.9114545212349;
	double k1 = -0.10031376438769382, k2 = 0.06480104711529402, p1 = 0.000571040851725, p2 = 0.000273292707729, k3 = -0.01242539054818;

	// zed2i left HD 720p
	// double fx = 531.75, fy = 532.04, cx = 632.77, cy = 356.759;
	// double k1 = -0.04568, k2 = 0.0180176, p1 = 0.000246693, p2 = -8.1439e-05,
	// k3 = -0.00783292;

	// zed2i left VGA 640p
	// double fx=265.875, fy=266.02, cx=331.885, cy=185.8795;
	// double k1=-0.04568, k2=0.0180176, p1=0.000246693, p2=-8.1439e-05,
	// k3=-0.00783292;

	// unit camera matrix
	// double fx=1, fy=1, cx=0, cy=0;
	// double k1=-0, k2=0, p1=0, p2=0, k3=0;

	cv::Mat cameraMatrix           = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);

	arucoHandler.cameraMatrix           = cameraMatrix;
	arucoHandler.distortionCoefficients = distortionCoefficients;

	opImageSub = node.subscribe("udfc_filtered", 10, &ArucoDetectionNode::callback, this);

	opImagePub    = node.advertise<sensor_msgs::Image>("aruco_image", 100);
	opPosePubUDFC = node.advertise<geometry_msgs::PoseStamped>("aruco_udfc_pose", 100);
	opPosePubODOM = node.advertise<geometry_msgs::PoseStamped>("aruco_odom_pose", 100);
	opObjPubUDFC  = node.advertise<vortex_msgs::ObjectPosition>("aruco_udfc_obj", 100);
	opObjPubODOM  = node.advertise<vortex_msgs::ObjectPosition>("aruco_odom_obj", 100);
	// opPosePubTfLandmark = node.advertise<vortex_msgs::ObjectPosition>("object_positions_in", 100); // this publisher has been moved moved to vision_kf

	dictionary = new cv::aruco::Dictionary;
	// dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100); // Vortex Docking plate dictionary
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL); // TAC dictionary

	// board = arucoHandler.createRectangularBoard(.09, .18, .135, dictionary, {28, 7, 96, 19}); // A4 paper
	board = arucoHandler.createRectangularBoard(.150, .430, .830, dictionary, {28, 7, 96, 19}); // TAC dimensions
	// board = arucoHandler.createRectangularBoard(.167, .462, .862, dictionary, {28, 7, 96, 19}); // Vortex Docking plate dimensions

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
			ROS_WARN_STREAM(ex.what());

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
	if (img.empty()) 
	{
		ROS_INFO_STREAM("DOCKING_NODE: Empty image");
		return;
	}
	// Sharpen image
	// cv::Mat filteredImg;
	
	// filter_from_rqt(img, filteredImg, filterParams.configs);

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

		// // IMAGE INPUT
		// cv::Mat img =
		// cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/TACboard3.jpg",
		// cv::IMREAD_COLOR); if( img.empty() )  // Check for invalid input
		// {
		//     ROS_WARN("Could not open or find the image");
		// }

		// // WEBCAM INPUT
		// static cv::Mat img;

		// // cv::namedWindow("Display window");
		// static cv::VideoCapture cap(0);
		// if (!cap.isOpened()) {
		//     ROS_INFO("cannot open camera");
		// }
		// cap >> img;

		// geometry_msgs::Pose pose;
		// int markersDetected = arucoHandler.detectBoardPose(img, board, pose);
		// publishCVImg(img, ros::Time::now());
		// if (markersDetected > 0)
		// {
		//     publishPose(pose, ros::Time::now());
		// }

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