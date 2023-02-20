#include "aruco_detection_node.hpp"


ArucoDetectionNode::ArucoDetectionNode() 
: loop_rate{10}
, arucoHandler{}
{
    op_sub = node.subscribe("/udfc/wrapper/camera_rect",10, &ArucoDetectionNode::callback, this);
    op_image_pub = node.advertise<sensor_msgs::Image>("aruco_image_out",10);
    op_pose_pub  = node.advertise<geometry_msgs::Pose>("aruco_poses_out",10);
    
    dictionary = new cv::aruco::Dictionary;
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    board = arucoHandler.createRectangularBoard(200, 400, 600, dictionary, {28,7,96,19});
}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr& img_source){

}

void ArucoDetectionNode::execute()
{
    while (ros::ok()) {

        // cv::namedWindow("arucoMarker", cv::WINDOW_AUTOSIZE);
        
        cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/TACboard3.jpg", cv::IMREAD_COLOR);
        if( img.empty() )  // Check for invalid input
        {
            ROS_INFO("Could not open or find the image");
        }

        std::vector<int> ids;
        std::vector<geometry_msgs::Pose> poses;
        arucoHandler.detectMarkerPoses(img, dictionary, poses, ids, 5);

        for (geometry_msgs::Pose pose: poses) {
            op_pose_pub.publish(pose);
        }

        geometry_msgs::Pose pose;
        int markernum = arucoHandler.detectBoardPose(img, board, pose);
        // ROS_INFO_STREAM("BoardPose: " << pose);


        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg; // >> message to be sent

        std_msgs::Header header; // empty header
        static size_t counter{0};
        header.seq = counter++; // user defined counter
        header.stamp = ros::Time::now(); // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        op_image_pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);



        ros::spinOnce();
        loop_rate.sleep();
    }
}





int main(int argc, char **argv) {
    ros::init(argc,argv,"ArucoDetectionNode");

    ArucoHandler arucoHandler;

    ArucoDetectionNode arucoNode;
    arucoNode.execute();
}