#include "aruco_detection_node.hpp"

ArucoDetectionNode::ArucoDetectionNode() 
: loop_rate{10}
, arucoHandler{}
{
    double fx=531.75, fy=532.04, cx=632.77, cy=356.759;
    double k1=-0.04568, k2=0.0180176, p1=0.000246693, p2=-8.1439e-05, k3=-0.00783292;
    cv::Mat cameraMatrix           = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
    arucoHandler.cameraMatrix = cameraMatrix;
    arucoHandler.distortionCoefficients = distortionCoefficients;
    op_sub = node.subscribe("/zed2i/zed_node/left/image_rect_color",10, &ArucoDetectionNode::callback, this);
    op_image_pub = node.advertise<sensor_msgs::Image>("aruco_image_out",100);
    op_pose_pub  = node.advertise<geometry_msgs::PoseStamped>("aruco_poses_out",100);
    
    dictionary = new cv::aruco::Dictionary;
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    // board = arucoHandler.createRectangularBoard(9, 18, 13.5, dictionary, {28,7,96,19});
    board = arucoHandler.createRectangularBoard(.09, .18, .135, dictionary, {28,7,96,19});   //A4 paper
    // board = arucoHandler.createRectangularBoard(.2, .4, .6, dictionary, {28,7,96,19});       //Actual dimensions
}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr& img_source)
{
    const cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(img_source, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cvImage->image;

    geometry_msgs::Pose pose;
    size_t markersDetected = arucoHandler.detectBoardPose(img, board, pose);

    if (markersDetected > 0) publishPose(pose);

    publishCVImg(img);
}

void ArucoDetectionNode::publishCVImg(const cv::Mat& img)
{
    static size_t counter{0};
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; 

    std_msgs::Header header;
    header.seq = counter++; 
    header.stamp = ros::Time::now(); // Should the time now be used, or the time the image was taken be used?
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    img_bridge.toImageMsg(img_msg); 
    op_image_pub.publish(img_msg);
}

void ArucoDetectionNode::publishPose(const geometry_msgs::Pose& pose)
{
    static size_t counter{0};
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "zed2i_left_camera_frame";
    poseMsg.header.seq = counter++;
    poseMsg.header.stamp = ros::Time::now(); // Should the time now be used, or the time the image was taken be used?
    poseMsg.pose = pose;
    op_pose_pub.publish(poseMsg);
}

void ArucoDetectionNode::execute()
{
    while (ros::ok()) {

        // IMAGE INPUT
        // cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/TACboard3.jpg", cv::IMREAD_COLOR);
        // if( img.empty() )  // Check for invalid input
        // {
        //     ROS_INFO("Could not open or find the image");
        // }

        // WEBCAM INPUT
        static cv::Mat img;
        // cv::namedWindow("Display window");
        static cv::VideoCapture cap(0);
        if (!cap.isOpened()) {
            ROS_INFO("cannot open camera");
        }   
        cap >> img;

        std::vector<int> ids;
        std::vector<geometry_msgs::Pose> poses;
        arucoHandler.detectMarkerPoses(img, dictionary, poses, ids, 5);

        for (geometry_msgs::Pose pose: poses) {
            op_pose_pub.publish(pose);
        }

        geometry_msgs::Pose pose;
        int markersDetected = arucoHandler.detectBoardPose(img, board, pose);
        publishCVImg(img);
        if (markersDetected > 0) publishPose(pose);


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