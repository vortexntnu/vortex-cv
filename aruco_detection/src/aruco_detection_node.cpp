#include "aruco_detection_node.h"

#include <aruco_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>


ArucoDetectionNode::ArucoDetectionNode() : loop_rate(10) {
    op_sub = node.subscribe("/zed2/zed_node/rgb/image_rect_color",10, &ArucoDetectionNode::callback, this);

    op_pub = node.advertise<aruco_msgs::MarkerArray>("arUco_marker_positions_out",10);

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    parameters = cv::aruco::DetectorParameters::create();
}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr& img_source){

    // Convert ROS-image to CV-image
    ////////////////////////////////

    const cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(img_source,"");


    // Detect markers //////////////
    ////////////////////////////////

    cv::Mat inputImage;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


    // op_pub.publish(objPos);            
}

void ArucoDetectionNode::execute(){
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char **argv){
    ros::init(argc,argv,"ArucoDetectionNode");
    ArucoDetectionNode arucoNode;
    arucoNode.execute();
}