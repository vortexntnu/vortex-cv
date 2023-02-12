#include "aruco_detection_node.hpp"

#include <Eigen/Geometry>


ArucoDetectionNode::ArucoDetectionNode() 
: loop_rate(10) 
{
    op_sub = node.subscribe("/udfc/wrapper/camera_rect",10, &ArucoDetectionNode::callback, this);
    op_image_pub = node.advertise<sensor_msgs::Image>("aruco_image_out",10);
    op_pose_pub  = node.advertise<geometry_msgs::Pose>("aruco_poses_out",10);

    // cv::Mat image = cv::imread("./mark_id_09.jpg", cv::IMREAD_COLOR);
    // cv::namedWindow("arucoMarker", cv::WINDOW_AUTOSIZE);
}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr& img_source){

}

void ArucoDetectionNode::execute(){
    while (ros::ok()) {

        // cv::namedWindow("arucoMarker", cv::WINDOW_AUTOSIZE);

        
        cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/src/aruco_image_test.jpg", cv::IMREAD_COLOR);
        if( img.empty() )  // Check for invalid input
        {
            ROS_INFO("Could not open or find the image");
        }

        // dictionary needs to be dynamically allocated or else we get a runtime error when it goes out of scope
        cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Mat imgCopy;

        cv::aruco::detectMarkers(img, dictionary, corners, ids);
        // if at least one marker detected
        if (ids.size() > 0)
        {
            // ROS_INFO("detected a marker of id: %i", ids.at(0));
            cv::aruco::drawDetectedMarkers(img, corners, ids);

            double fx=1, fy=1, cx=0, cy=0;
            double k1=0, k2=0, p1=0, p2=0, k3=0;
            
            cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
            cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
            std::vector< cv::Vec3d > rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distortionCoefficients, rvecs, tvecs);
                    // draw axis for each marker
            for(size_t i{0}; i<ids.size(); i++)
            {
                cv::aruco::drawAxis(img, cameraMatrix, distortionCoefficients, rvecs[i], tvecs[i], .1);

                
                // axis-angle to rotation matrix
                cv::Matx41d quaternion = aa2quaternion(rvecs[i]);
                geometry_msgs::Pose pose;
                pose.position.x = tvecs[i][0];
                pose.position.y = tvecs[i][1];
                pose.position.z = tvecs[i][2];

                pose.orientation.x = quaternion(0);
                pose.orientation.y = quaternion(1);
                pose.orientation.z = quaternion(2);
                pose.orientation.w = quaternion(3);

                op_pose_pub.publish(pose);
            }

        }










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

cv::Matx41d aa2quaternion(const cv::Matx31d& aa)
{
    double angle = cv::norm(aa);
    cv::Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
    double angle_2 = angle / 2;
    //qx, qy, qz, qw
    cv::Matx41d q(axis(0) * sin(angle_2), axis(1) * sin(angle_2), axis(2) * sin(angle_2), cos(angle_2));
    return q;
}




int main(int argc, char **argv){

    // cv::Mat markerImage;
    // cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
    // dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    // int id = 23;
    // cv::aruco::drawMarker(dictionary, id, 200, markerImage, 1);
    // cv::imwrite("/vortex_ws/src/vortex-cv/aruco_detection/src/markerID_23.jpg", markerImage);

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