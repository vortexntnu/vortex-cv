#include "aruco_detection_node.hpp"




ArucoDetectionNode::ArucoDetectionNode() 
: loop_rate(10) 
{
    op_sub = node.subscribe("/udfc/wrapper/camera_rect",10, &ArucoDetectionNode::callback, this);
    op_pub = node.advertise<sensor_msgs::ImageConstPtr>("arUco_marker_positions_out",10);

    // cv::Mat image = cv::imread("./mark_id_09.jpg", cv::IMREAD_COLOR);
    // cv::namedWindow("arucoMarker", cv::WINDOW_AUTOSIZE);
}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr& img_source){

}

void ArucoDetectionNode::execute(){
    while (ros::ok()) {

        // cv::namedWindow("arucoMarker", cv::WINDOW_AUTOSIZE);


        cv::Mat img = cv::imread("./mark_id_09.jpg", cv::IMREAD_COLOR);
        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg; // >> message to be sent

        std_msgs::Header header; // empty header
        static size_t counter{0};
        header.seq = counter++; // user defined counter
        header.stamp = ros::Time::now(); // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        op_pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

        ros::spinOnce();
        loop_rate.sleep();
    }
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