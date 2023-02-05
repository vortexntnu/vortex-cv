#include "aruco_detection_node.h"


ArucoDetectionNode::ArucoDetectionNode() 
: loop_rate(10) 
, dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250))
, detectorParameters(cv::aruco::DetectorParameters::create())
, markerLength(0.05)
, cameraMatrix(cv::Mat::eye(3, 3, CV_32F))
, distCoeffs(cv::Mat::zeros(3, 1, CV_32F))
{
    op_sub = node.subscribe("/udfc/wrapper/camera_rect",10, &ArucoDetectionNode::callback, this);
    op_pub = node.advertise<aruco_msgs::MarkerArray>("arUco_marker_positions_out",10);
}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr& img_source){

    // Convert ROS-image to CV-image
    ////////////////////////////////

    const cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(img_source, "");

    // Detect markers //////////////
    ////////////////////////////////

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::detectMarkers(cvImage->image, dictionary, markerCorners, markerIds, detectorParameters, rejectedCandidates);

    // Estimate poses //////////////
    ////////////////////////////////
    aruco_msgs::MarkerArray markerMsg;
    estimateMarkerPoses(markerIds, markerCorners, markerMsg);

    op_pub.publish(markerMsg);            
}

void ArucoDetectionNode::execute(){
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ArucoDetectionNode::estimateMarkerPoses(
    std::vector<int> markerIds, 
    std::vector<std::vector<cv::Point2f>> markerCorners, 
    aruco_msgs::MarkerArray& markerMsg) {

    // Estimate poses //////////////
    ////////////////////////////////
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);


    // Put result in a ROS message /
    ////////////////////////////////    

    std_msgs::Header msgHeader;
    static size_t seq_id{0};
    
    msgHeader.frame_id = "UDFC_frame";
    msgHeader.seq      = seq_id++;
    msgHeader.stamp    = ros::Time::now();

    markerMsg.header = msgHeader;

    for (size_t i; i<markerIds.size(); i++) {
        aruco_msgs::Marker marker;
        marker.header                = msgHeader;
        marker.id                    = markerIds.at(i);
        marker.pose.pose.orientation = rvecs.at(i);
        marker.pose.pose.position    = tvecs.at(i);
        markerMsg.markers.push_back(marker);
    }
}

void ArucoDetectionNode::estimateBoardPose(
    std::vector<int> markerIds, 
    std::vector<std::vector<cv::Point2f>> markerCorners, 
    geometry_msgs::Pose boardPose) {

    // if at least one marker is detected
    cv::Vec3d rvec, tvec;
    int valid{0};
    if (markerIds.size() > 0) {
        valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, &board, cameraMatrix, distCoeffs, rvec, tvec);
    }
    if (!valid) return;
    double angle = cv::norm(rvec);
    double nu    = 

    boardPose.orientation = geometry_msgs::Point(tvec);
    boardPose.position    = tvec;
}






int main(int argc, char **argv){
    // Needs config:
    //      Camera calibration
    //      Marker dictionary
    //      Marker length
    //      Board
    //      Frame
    //      Camera subscribe topic
    ros::init(argc,argv,"ArucoDetectionNode");
    ArucoDetectionNode arucoNode;
    arucoNode.execute();
}