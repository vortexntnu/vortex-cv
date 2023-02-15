#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "aruco_detection_node.hpp"
// #include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Geometry>

void drawCustomFrameAxes(cv::Mat img, cv::Mat cameraMatrix, cv::InputArray distCoeffs, cv::InputArray estimatedRvec,
                         cv::InputArray estimatedTvec, float axisLength, int thickness = 3,
                         cv::Point3f center = cv::Point3f(0.f, 0.f, 0.f), cv::Point3f axes = cv::Point3f(1.f, -1.f, -1.f)) 
{
    std::vector<cv::Point3f> ax = {center, 
                                   center + cv::Point3f(axes.x, 0.f, 0.f)*axisLength,
                                   center + cv::Point3f(0.f, axes.y, 0.f)*axisLength,
                                   center + cv::Point3f(0.f, 0.f, axes.z)*axisLength};
    
    cv::Scalar colors[3] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0)};
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(ax, estimatedRvec, estimatedTvec, cameraMatrix, distCoeffs, imagePoints);
    for (size_t i = 1ull; i < ax.size(); i++)
        cv::line(img, imagePoints[0], imagePoints[i], colors[i-1], thickness);
}

ArucoDetectionNode::ArucoDetectionNode() 
: loop_rate(10) 
{
    op_sub = node.subscribe("/udfc/wrapper/camera_rect",10, &ArucoDetectionNode::callback, this);
    op_image_pub = node.advertise<sensor_msgs::Image>("aruco_image_out",10);
    op_pose_pub  = node.advertise<geometry_msgs::Pose>("aruco_poses_out",10);

    // cv::Mat image = cv::imread("./mark_id_09.jpg", cv::IMREAD_COLOR);
    // cv::namedWindow("arucoMarker", cv::WINDOW_AUTOSIZE);
    cv::Mat a;
}

void ArucoDetectionNode::callback(const sensor_msgs::ImageConstPtr& img_source){

}

void ArucoDetectionNode::execute(){
    while (ros::ok()) {

        // cv::namedWindow("arucoMarker", cv::WINDOW_AUTOSIZE);

        
        cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/boardtest_obscured2.jpg", cv::IMREAD_COLOR);
        if( img.empty() )  // Check for invalid input
        {
            ROS_INFO("Could not open or find the image");
        }

        // dictionary needs to be dynamically allocated or else we get a runtime error when it goes out of scope
        // or something idk
        cv::Ptr<cv::aruco::Dictionary> dictionary = new cv::aruco::Dictionary;
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::Mat imgCopy;

        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);
        // if at least one marker detected
        if (ids.size() > 0)
        {
            // ROS_INFO("detected a marker of id: %i. Markers detected: %i", ids.at(0), ids.size());
            // ROS_INFO("ids:%i, %i, %i, %i", ids.at(0), ids.at(1), ids.at(2), ids.at(3));
            // cv::aruco::drawDetectedMarkers(img, corners, ids);

            double fx=1, fy=1, cx=0, cy=0;
            double k1=0, k2=0, p1=0, p2=0, k3=0;
            
            cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
            cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
            std::vector< cv::Vec3d > rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 1, cameraMatrix, distortionCoefficients, rvecs, tvecs);
            // cv::solvePnP()
                    // draw axis for each marker
            for(size_t i{0}; i<ids.size(); i++)
            {
                // cv::aruco::drawAxis(img, cameraMatrix, distortionCoefficients, rvecs[i], tvecs[i], .1);
                // drawCustomFrameAxes(img, cameraMatrix, distortionCoefficients, rvecs[i], tvecs[i], .1);
                
                // axis-angle to rotation matrix
                // cv::Matx41d quaternion = aa2quaternion(rvecs[i]);
                // geometry_msgs::Pose pose;
                // pose.position.x = tvecs[i][0];
                // pose.position.y = tvecs[i][1];
                // pose.position.z = tvecs[i][2];

                // pose.orientation.x = quaternion(0);
                // pose.orientation.y = quaternion(1);
                // pose.orientation.z = quaternion(2);
                // pose.orientation.w = quaternion(3);

                // op_pose_pub.publish(pose);
            }



            /// BOARD DETECTION
            cv::Vec3d rvec, tvec;

            cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(2,2,.5,1,dictionary, 0);
            int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix, distortionCoefficients, rvec, tvec);
            // ROS_INFO("VALID: %i", valid);
            if(valid > 0) {
                // cv::aruco::drawAxis(img, cameraMatrix, distortionCoefficients, rvec, tvec, 0.1);
                drawCustomFrameAxes(img, cameraMatrix, distortionCoefficients, rvec, tvec, .5, 3, cv::Point3f(.5+.5,.5+.5,0));
                // ROS_INFO("t: [%f,%f,%f]",tvec[0],tvec[1],tvec[2]);
                // ROS_INFO("r: [%f,%f,%f]",rvec[0],rvec[1],rvec[2]);
                cv::Matx41d quaternion = aa2quaternion(rvec);
                geometry_msgs::Pose pose;
                pose.position.x = tvec[0];
                pose.position.y = tvec[1];
                pose.position.z = tvec[2];

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
    // // int id = 23;
    // // cv::aruco::drawMarker(dictionary, id, 200, markerImage, 1);
    // // cv::imwrite("/vortex_ws/src/vortex-cv/aruco_detection/src/markerID_23.jpg", markerImage);
    // cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(2,2,.5,1,dictionary, 0);
    
    // cv::Mat boardImage;
    // board->draw(cv::Size(600, 500), boardImage, 10, 1);
    // cv::imwrite("/vortex_ws/src/vortex-cv/aruco_detection/test/pictures/arucoBoard2.jpg", boardImage);


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