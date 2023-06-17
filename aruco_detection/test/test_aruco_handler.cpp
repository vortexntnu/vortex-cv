#include <aruco_handler.hpp>

void testPoses()
{
	cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
	dictionary                                = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/test/"
	                         "pictures/aruco_image_test.jpg",
	                         cv::IMREAD_COLOR);

	if (img.empty()) // Check for invalid input
	{
		ROS_INFO("Could not open or find the image");
	}

	ArucoHandler arucoHandler{};
	std::vector<geometry_msgs::Pose> poses;
	std::vector<int> ids;
	double markerLength{5};
	std::vector<std::vector<cv::Point2f>> corners, rejected;


	int markernum = arucoHandler.detectMarkerPoses(img, dictionary, poses, ids, markerLength);
	ROS_INFO_STREAM("poses: " << poses.at(0) << poses.at(1));
}

void testBoardCreator()
{
	cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;

	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
	ArucoHandler arucoHandler{};
	// unit camera matrix
	double fx = 1, fy = 1, cx = 0, cy = 0;
	double k1 = -0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;

	cv::Mat cameraMatrix           = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);

	arucoHandler.cameraMatrix           = cameraMatrix;
	arucoHandler.distortionCoefficients = distortionCoefficients;
	cv::Ptr<cv::aruco::Board> board     = arucoHandler.createRectangularBoard(200, 800 - 400, 1200 - 400, dictionary, {28, 7, 96, 19});

	cv::Mat boardImg;
	cv::aruco::drawPlanarBoard(board, {800, 1200}, boardImg, 10);
	cv::imwrite("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/TACboard.jpg", boardImg);
}

void testBoardPose()
{
	cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
	dictionary                                = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

	cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/TACboard2.jpg", cv::IMREAD_COLOR);
	if (img.empty()) // Check for invalid input
	{
		ROS_INFO("Could not open or find the image");
	}

	ArucoHandler arucoHandler{};
	geometry_msgs::Pose pose;
	cv::Ptr<cv::aruco::Board> board = arucoHandler.createRectangularBoard(200, 800 - 400, 1200 - 400, dictionary, {28, 7, 96, 19});

	cv::Mat imgCopy;
	img.copyTo(imgCopy);
	int markernum = arucoHandler.detectBoardPose(img, imgCopy, board, pose);
	ROS_INFO_STREAM("BoardPose: " << pose);
}

int main()
{
	// testBoardCreator();
}