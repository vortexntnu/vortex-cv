#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <string>
#include <vector>
#include <Eigen/Geometry>


/**
 * Wrapper class for manipulating aruco markers. 
 * 
 *  Estimate pose of marker/board 
 *  Store paramters that are used for all detections
 * 
*/
class ArucoHandler {
public: 
    ArucoHandler(const cv::Ptr<cv::aruco::Dictionary> &dictionary, cv::Mat cameraMatrix, cv::Mat distortionCoefficients);
    ArucoHandler(const cv::Ptr<cv::aruco::Dictionary> &dictionary);
    
    /**
     * Detects arUco markers in image
     * Use markerPoses and boardPose instead
    */
    void detectMarkers(cv::InputArray img, cv::OutputArrayOfArrays corners, cv::OutputArray ids, cv::OutputArrayOfArrays rejected);
    
    /**
     * Looks for markers in image and returns poses of the markers detected
     * @param img   input: image
     * @param poses output: estimated poses of detected markers
     * @param ids   ouput: ids of detected markers
    */
    int markerPoses(cv::InputArray img, std::vector<geometry_msgs::Pose> &poses, cv::OutputArray ids, double markerLength);
    
    /**
     * Looks for ArUco boards in image and returns its pose if found
     * @param img input: image
     * @param pose output: estimated pose of board
    */
    void boardPose(cv::InputArray img, geometry_msgs::Pose pose);
    void drawAxis();
    
    /**
     * Creates a custom 4 marker board
    */
    cv::aruco::Board createCustomBoard();
    void drawMarker(int id, std::string filepath);
    void drawBoard(cv::Ptr<cv::aruco::Board> board, std::string filepath);
protected:
    void findCenter();
    /**
     * Converts a rotation in the axis-angle representation to a quaternion
     * @param aa rotation vector with rotation axis in the direction it points and angle "norm(aa)"
    */
    cv::Matx41d axisAngle2Quaternion (const cv::Matx31d& aa);
    /**
     * Converts OpenCV position- and rotation-representation to pose
     * @param rvec rotation vector
     * @param tvec translation vector
    */
    geometry_msgs::Pose tvec_rvec2pose(cv::Vec3d rvec, cv::Vec3d tvec);

    const cv::Ptr<cv::aruco::Dictionary> &dictionary;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
// markerSize


};