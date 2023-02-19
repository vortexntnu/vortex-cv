#include "aruco_handler.hpp"


void testPoses() 
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/aruco_image_test.jpg", cv::IMREAD_COLOR);

    if( img.empty() )  // Check for invalid input
    {
        ROS_INFO("Could not open or find the image");
    }
    
    ArucoHandler markerHandler{};
    std::vector<geometry_msgs::Pose> poses;
    std::vector<int> ids;
    double markerLength{5};
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    // cv::aruco::detectMarkers(img, dictionary, corners, ids);
    // markerHandler.detectMarkers(img, corners, ids, rejected);
    int markernum = markerHandler.detectMarkerPoses(img, dictionary, poses, ids, markerLength);
    // ROS_INFO_STREAM("num ids: " << ids.size() );
    // ROS_INFO_STREAM("num ids: " << markernum);
    ROS_INFO_STREAM("poses: " << poses.at(0) << poses.at(1));
}


void testBoardCreator() 
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    ArucoHandler arucoHandler{};

    cv::Ptr<cv::aruco::Board> board = arucoHandler.createRectangularBoard(200, 800-400, 1200-400, dictionary, {28,7,96,19});

    cv::Mat boardImg;
    cv::aruco::drawPlanarBoard(board, {800,1200}, boardImg, 10);
    cv::imwrite("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/TACboard.jpg", boardImg);
}

void testBoardPose() 
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

    cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/TACboard2.jpg", cv::IMREAD_COLOR);
    if( img.empty() )  // Check for invalid input
    {
        ROS_INFO("Could not open or find the image");
    }
    
    ArucoHandler aruco{};
    geometry_msgs::Pose pose;
    cv::Ptr<cv::aruco::Board> board = aruco.createRectangularBoard(200, 800-400, 1200-400, dictionary, {28,7,96,19});

    int markernum = aruco.detectBoardPose(img, board, pose);
    ROS_INFO_STREAM("BoardPose: " << pose);
}

int main() 
{
    testBoardPose();
}