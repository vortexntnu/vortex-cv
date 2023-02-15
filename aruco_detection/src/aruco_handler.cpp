#include "aruco_handler.hpp"

ArucoHandler::ArucoHandler(const cv::Ptr<cv::aruco::Dictionary>& dictionary)
: dictionary{dictionary}
, detectorParams{cv::aruco::DetectorParameters::create()}
{
    double fx=1, fy=1, cx=0, cy=0;
    double k1=0, k2=0, p1=0, p2=0, k3=0;
    cameraMatrix           = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

ArucoHandler::ArucoHandler(const cv::Ptr<cv::aruco::Dictionary> &dictionary, cv::Mat cameraMatrix, cv::Mat distortionCoefficients)
: dictionary{dictionary}
, cameraMatrix{cameraMatrix}
, distortionCoefficients{distortionCoefficients}
, detectorParams{cv::aruco::DetectorParameters::create()}
{
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

void ArucoHandler::detectMarkers(cv::InputArray img, cv::OutputArrayOfArrays corners, cv::OutputArray ids, cv::OutputArrayOfArrays rejected = cv::noArray())
{
    cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected, cameraMatrix, distortionCoefficients);   
}

int ArucoHandler::markerPoses(cv::InputArray img, std::vector<geometry_msgs::Pose> &poses, cv::OutputArray ids, double markerLength)
{
    ROS_INFO_STREAM(dictionary.empty());

    std::vector<std::vector<cv::Point2f>> corners, rejected;
    
    ROS_INFO_STREAM("num ids: " << ids.size() );
    ROS_INFO_STREAM(dictionary.empty());

    cv::aruco::detectMarkers(img, dictionary, corners, ids);//, detectorParams, rejected, cameraMatrix, distortionCoefficients);
    if (ids.depth() == 0) return 0;

    std::vector< cv::Vec3d > rvecs, tvecs;
    //REPLACE with cv::solvePnP if opencv is updated to v. 4.5.5 or above. It is more accurate
    //NB! t_vec points to center of marker in v. 4.5.4 and below. To top left corner in ~ 4.5.5 and above
    cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distortionCoefficients, rvecs, tvecs);
    
    ROS_INFO_STREAM("num ids: " << ids.size() );
    for (size_t i{0}; i<ids.depth(); i++)
    {
    ROS_INFO_STREAM("num ids: " << ids.size() );

        poses.push_back(tvec_rvec2pose(rvecs.at(i), tvecs.at(i)));
    ROS_INFO_STREAM("num ids: " << ids.size() );

    }
    return ids.depth();
}



cv::Matx41d ArucoHandler::axisAngle2Quaternion (const cv::Matx31d& aa)
{
    double angle = cv::norm(aa);
    cv::Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
    double angle_2 = angle / 2;
    //qx, qy, qz, qw
    cv::Matx41d q(axis(0) * sin(angle_2), axis(1) * sin(angle_2), axis(2) * sin(angle_2), cos(angle_2));
    return q;
}

geometry_msgs::Pose ArucoHandler::tvec_rvec2pose(cv::Vec3d rvec, cv::Vec3d tvec)
{
    cv::Matx41d quaternion = axisAngle2Quaternion(rvec);
    geometry_msgs::Pose pose;
    pose.position.x = tvec[0];
    pose.position.y = tvec[1];
    pose.position.z = tvec[2];

    pose.orientation.x = quaternion(0);
    pose.orientation.y = quaternion(1);
    pose.orientation.z = quaternion(2);
    pose.orientation.w = quaternion(3);
}

// void ArucoHandler::findCenter(cv::Mat img, 
//                               cv::InputArray rvec,
//                               cv::InputArray estimatedTvec, 
//                               float axisLength,
//                               cv::Point3f center = cv::Point3f(0.f, 0.f, 0.f), 
//                               cv::Point3f axes = cv::Point3f(1.f, -1.f, -1.f)) 
// {
//     std::vector<cv::Point3f> ax = {center, 
//                                    center + cv::Point3f(axes.x, 0.f, 0.f)*axisLength,
//                                    center + cv::Point3f(0.f, axes.y, 0.f)*axisLength,
//                                    center + cv::Point3f(0.f, 0.f, axes.z)*axisLength};
//     cv::transform()
// }