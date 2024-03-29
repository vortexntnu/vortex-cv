#include "aruco_handler.hpp"

ArucoHandler::ArucoHandler() : detectorParams{cv::aruco::DetectorParameters::create()}
{
	double fx = 1, fy = 1, cx = 0, cy = 0;
	double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
	cameraMatrix                           = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	distortionCoefficients                 = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
	detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

int ArucoHandler::detectMarkerPoses(cv::Mat &img, const cv::Ptr<cv::aruco::Dictionary> dictionary, std::vector<geometry_msgs::Pose> &poses,
                                    std::vector<int> &ids, double markerLength)
{
	ROS_WARN("THIS FUNCTION MAY FAIL! (is currently unused)");
	std::vector<std::vector<cv::Point2f>> corners, rejected;

	cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams,
	                         rejected); //, cameraMatrix, distortionCoefficients);
	if (ids.size() == 0)
		return 0;

	std::vector<cv::Vec3d> rvecs, tvecs;

	cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distortionCoefficients, rvecs, tvecs);

	for (size_t i{0}; i < ids.size(); i++) {
		poses.push_back(tvec_rvec2pose(rvecs.at(i), tvecs.at(i)));
	}
	return ids.size();
}

cv::Matx41d ArucoHandler::axisAngle2Quaternion(const cv::Matx31d &aa)
{
	double angle = cv::norm(aa);
	cv::Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
	double angle_2 = angle / 2;
	// qx, qy, qz, qw
	cv::Matx41d q(axis(0) * sin(angle_2), axis(1) * sin(angle_2), axis(2) * sin(angle_2), cos(angle_2));
	return q;
}

geometry_msgs::Pose ArucoHandler::tvec_rvec2pose(const cv::Vec3d &rvec, const cv::Vec3d &tvec)
{
	cv::Matx41d quaternion = axisAngle2Quaternion(rvec);
	geometry_msgs::Pose pose;
	pose.position.x = tvec[0];
	pose.position.y = tvec[1];
	pose.position.z = tvec[2];

	pose.orientation.x = quaternion(0);
	pose.orientation.y = quaternion(1);
	pose.orientation.z = quaternion(2);
	pose.orientation.w = quaternion(3);

	return pose;
}

cv::Ptr<cv::aruco::Board> ArucoHandler::createRectangularBoard(float markerSize, float xDist, float yDist, const cv::Ptr<cv::aruco::Dictionary> &dictionary,
                                                               const std::vector<int> &ids)
{
	const float markerHalf{markerSize / 2}, xHalf{xDist / 2}, yHalf{yDist / 2};

	// Define center of each marker
	std::vector<cv::Point3f> markerCenters;
	markerCenters.push_back({-xHalf - markerHalf, yHalf + markerHalf, 0});
	markerCenters.push_back({xHalf + markerHalf, yHalf + markerHalf, 0});
	markerCenters.push_back({xHalf + markerHalf, -yHalf - markerHalf, 0});
	markerCenters.push_back({-xHalf - markerHalf, -yHalf - markerHalf, 0});

	// Place marker at each marker center
	std::vector<std::vector<cv::Point3f>> markerPoints;
	for (size_t i{0}; i < markerCenters.size(); i++) {
		std::vector<cv::Point3f> marker;
		float xOffset{markerCenters.at(i).x};
		float yOffset{markerCenters.at(i).y};

		// Marker corners need to be added CLOCKWISE from top left corner
		marker.push_back({xOffset - markerHalf, yOffset + markerHalf, 0});
		marker.push_back({xOffset + markerHalf, yOffset + markerHalf, 0});
		marker.push_back({xOffset + markerHalf, yOffset - markerHalf, 0});
		marker.push_back({xOffset - markerHalf, yOffset - markerHalf, 0});
		markerPoints.push_back(marker);
	}
	cv::Ptr<cv::aruco::Board> board = new cv::aruco::Board;
	board                           = cv::aruco::Board::create(markerPoints, dictionary, ids);
	return board;

	/*

	top left corner of marker
	|
	X---O               X---O
	|id0|---- xDist ----|id1|
	O---O               O---O
	  |                   |
	  |                   |
	  |                   |
	  |                   |
	yDist       O       yDist
	  |         |         |
	  |      origin       |
	  |                   |
	  |                   |
	X---O               X---O
	|id3|---- xDist ----|id2|
	O---O               O---O
	|   |
	markerSize

	*/
}

size_t ArucoHandler::detectBoardPose(const cv::Mat &originalImg, cv::Mat &modifiedImg, const cv::Ptr<cv::aruco::Board> &board, geometry_msgs::Pose &pose)
{

	if (originalImg.empty())
		ROS_WARN("No image");

	std::vector<std::vector<cv::Point2f>> corners, rejected;
	std::vector<int> ids, recoveredIds;
	originalImg.copyTo(modifiedImg);

	cv::aruco::detectMarkers(originalImg, board->dictionary, corners, ids, detectorParams, rejected);
	if (ids.size() == 0)
		return 0;
	// Draw Markers
	cv::aruco::drawDetectedMarkers(modifiedImg, corners, ids);
	cv::aruco::drawDetectedMarkers(modifiedImg, rejected, cv::noArray(), cv::Scalar(0, 0, 255));

	// Estimate pose
	cv::Vec3d rvec, tvec;
	int numUsedMarkers = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix, distortionCoefficients, rvec,
	                                                  tvec); // replace with cv::solvePnP if OpenCV is updated to v. 4.5.5 or
	                                                         // above. It is more accurate
	cv::aruco::refineDetectedMarkers(originalImg, board, corners, ids, rejected, cameraMatrix, distortionCoefficients, 10, 3, true, recoveredIds, detectorParams);
	numUsedMarkers += recoveredIds.size();
	size_t detectedMarkerThreshhold = 1;
	if (numUsedMarkers < 1)
		return 0; // Don't estimate if you only see less markers than the threshhold
	pose = tvec_rvec2pose(rvec, tvec);

	float length = cv::norm(board->objPoints[0][0] - board->objPoints[0][1]); // Visual length of the drawn axis
	cv::aruco::drawAxis(modifiedImg, cameraMatrix, distortionCoefficients, rvec, tvec, length);

	return numUsedMarkers;
}
