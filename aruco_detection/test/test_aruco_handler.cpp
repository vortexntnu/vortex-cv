#include "aruco_handler.hpp"

int main() 
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    ROS_INFO_STREAM(dictionary.empty());

    cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/aruco_image_test.jpg", cv::IMREAD_COLOR);

    if( img.empty() )  // Check for invalid input
    {
        ROS_INFO("Could not open or find the image");
    }
    
    ArucoHandler markerHandler{dictionary};
    std::vector<geometry_msgs::Pose> poses;
    std::vector<int> ids;
    double markerLength{5};
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    ROS_INFO("abs");

    // cv::aruco::detectMarkers(img, dictionary, corners, ids);
    // markerHandler.detectMarkers(img, corners, ids, rejected);
    int markernum = markerHandler.markerPoses(img, poses, ids, markerLength);
    // ROS_INFO_STREAM("num ids: " << ids.size() );
    ROS_INFO_STREAM("num ids: " << markernum);

}