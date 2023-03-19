#include "aruco_detection_node.hpp"

ArucoDetectionNode::ArucoDetectionNode() 
: loop_rate{10}
, listener{tfBuffer}
, arucoHandler{}
{
    // zed2i left
    // double fx=531.75, fy=532.04, cx=632.77, cy=356.759;
    // double k1=-0.04568, k2=0.0180176, p1=0.000246693, p2=-8.1439e-05, k3=-0.00783292;
    double fx=1, fy=1, cx=0, cy=0;
    double k1=-0, k2=0, p1=0, p2=0, k3=0;
    cv::Mat cameraMatrix           = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
    arucoHandler.cameraMatrix = cameraMatrix;
    arucoHandler.distortionCoefficients = distortionCoefficients;
    opSub = node.subscribe("/zed2i/zed_node/left_raw/image_raw_color",10, &ArucoDetectionNode::callback, this);
    opImagePub          = node.advertise<sensor_msgs  ::Image         >("aruco_image_out"   ,100);
    opPosePub           = node.advertise<geometry_msgs::PoseStamped   >("aruco_poses_out"   ,100);
    opPosePubTf         = node.advertise<geometry_msgs::PoseStamped   >("aruco_tf_poses_out",100);
    opPosePubTfLandmark = node.advertise<vortex_msgs  ::ObjectPosition>("object_position_in",100);
    
    dictionary = new cv::aruco::Dictionary;
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    board = arucoHandler.createRectangularBoard(.09, .18, .135, dictionary, {28,7,96,19});   //A4 paper
    // board = arucoHandler.createRectangularBoard(.2, .4, .6, dictionary, {28,7,96,19});       //Actual dimensions

    

    ////////////////////////////
    //// Init Transforms ///////
    ////////////////////////////
    std::string parentFrame = "odom"; 
    std::string childFrame  = "udfc_link";

    // Wait for a transform to be available
    while (!tfBuffer.canTransform(parentFrame, childFrame, ros::Time(0))) 
    {
        try 
        {
            ROS_INFO_STREAM("No transform between " << parentFrame << " and " << childFrame);
            ros::Duration(2.0).sleep();
        }
        catch(tf2::TransformException &ex) 
        {
            ROS_WARN_STREAM("TransformException: " << ex.what());
            ros::Duration(2.0).sleep();
        }
    }
    ROS_INFO_STREAM("Transform between " << parentFrame << " and " << childFrame << " found.");


}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr& img_source)
{
    const cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(img_source, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cvImage->image;

    geometry_msgs::Pose pose;
    size_t markersDetected = arucoHandler.detectBoardPose(img, board, pose);

    if (markersDetected > 0) publishPose(pose, cvImage->header.stamp);

    publishCVImg(img, cvImage->header.stamp);
}

void ArucoDetectionNode::publishCVImg(const cv::Mat& img, ros::Time timestamp)
{
    static size_t counter{0};
    cv_bridge::CvImage imgBridge;
    sensor_msgs::Image imgMsg; 

    std_msgs::Header header;
    header.seq = counter++; 
    header.stamp = timestamp; // Should the time now be used, or the time the image was taken be used?
    imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    imgBridge.toImageMsg(imgMsg); 
    opImagePub.publish(imgMsg);
}

void ArucoDetectionNode::publishPose(const geometry_msgs::Pose& pose, ros::Time timestamp)
{
    static size_t counter{0};
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "zed2i_left_camera_frame";
    poseMsg.header.seq = counter++;
    poseMsg.header.stamp = timestamp; // Should the time now be used, or the time the image was taken be used?
    poseMsg.pose = pose;
    opPosePub.publish(poseMsg);
    

    geometry_msgs::Pose poseTF;
    tf2::doTransform(pose, poseTF, odom_udfc_transform);
    geometry_msgs::PoseStamped poseTFMsg;
    poseTFMsg.header.frame_id = "odom";
    poseTFMsg.header.seq      = counter;
    poseTFMsg.header.stamp    = timestamp;
    poseTFMsg.pose            = poseTF;
    opPosePubTf.publish(poseMsg);

    vortex_msgs::ObjectPosition vortexPoseMsg;
    vortexPoseMsg.objectID          = "docking_point";
    vortexPoseMsg.objectPose        = poseTFMsg;
    vortexPoseMsg.isDetected        = true;
    vortexPoseMsg.estimateConverged = true;
    vortexPoseMsg.estimateFucked    = false;
    opPosePubTfLandmark.publish(vortexPoseMsg);
}

void ArucoDetectionNode::execute()
{
    while (ros::ok()) {

        // // IMAGE INPUT
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

        // std::vector<std::vector<cv::Point2f>> corners, rejected;
        // std::vector<int> ids;
        // cv::Ptr<cv::aruco::DetectorParameters> dtParams = cv::aruco::DetectorParameters::create();
        // cv::aruco::detectMarkers(img, board->dictionary, corners, ids, dtParams, rejected);//, cameraMatrix, distortionCoefficients);


        geometry_msgs::Pose pose;
        int markersDetected = arucoHandler.detectBoardPose(img, board, pose);
        publishCVImg(img, ros::Time::now());
        if (markersDetected > 0) 
        {
            publishPose(pose, ros::Time::now());
        }


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