#include "aruco_detection_node.hpp"




ArucoDetectionNode::ArucoDetectionNode() 
: loop_rate(10) 
{
    op_sub = node.subscribe("/udfc/wrapper/camera_rect",10, &ArucoDetectionNode::callback, this);
    op_image_pub = node.advertise<sensor_msgs::Image>("arUco_image_out",10);

    // cv::Mat image = cv::imread("./mark_id_09.jpg", cv::IMREAD_COLOR);
    // cv::namedWindow("arucoMarker", cv::WINDOW_AUTOSIZE);
}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr& img_source){

}

void ArucoDetectionNode::execute(){
    while (ros::ok()) {

        // cv::namedWindow("arucoMarker", cv::WINDOW_AUTOSIZE);

        
        cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/src/mark_id_09.jpg", cv::IMREAD_COLOR);
        if( img.empty() )  // Check for invalid input
        {
            ROS_INFO("Could not open or find the image");

        }

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Mat imgCopy;

        cv::aruco::detectMarkers(img, &dictionary, corners, ids);
        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(imgCopy, corners, ids);
            ROS_INFO("detected a marker of id: %i", ids.at(0));
        }
        else{
            ROS_INFO("No marker detected");
        }


        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg; // >> message to be sent

        std_msgs::Header header; // empty header
        static size_t counter{0};
        header.seq = counter++; // user defined counter
        header.stamp = ros::Time::now(); // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        op_image_pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);







        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char **argv){

    // cv::Mat markerImage;
    // const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    // cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
    // cv::imwrite("/vortex_ws/src/vortex-cv/aruco_detection/src/")

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