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
    ArucoHandler(cv::Mat cameraMatrix, cv::Mat distortionCoefficients);
    ArucoHandler();
    
    
    /**
     * Looks for markers in image and returns poses of the markers detected
     * @param img   input: image
     * @param poses output: estimated poses of detected markers
     * @param ids   ouput: ids of detected markers
    */
    int detectMarkerPoses(const cv::Mat& img, const cv::Ptr<cv::aruco::Dictionary> dictionary, std::vector<geometry_msgs::Pose> &poses, const std::vector<int> &ids, double markerLength);
    
    /**
     * Looks for ArUco boards in image and returns its pose if found
     * @param img input: image (Will be modified)
     * @param pose output: estimated pose of board
    */
    size_t detectBoardPose(cv::Mat& img, const cv::Ptr<cv::aruco::Board>& board, geometry_msgs::Pose& pose);

    void drawAxis();
    
    /**
     * Creates a rectangular aruco-board with a marker in each corner
     * @param markerSize size [m] of marker side in meters
     * @param xDist distance [m] between markers in x-direction
     * @param yDist distance [m] between markers in y-direction
     * @param dictionary the dictionary of markers employed for this board
     * @param ids ids of the 4 markers. Added clockwise from the top-left corner. All markers have the default orientation
    */
    cv::Ptr<cv::aruco::Board> createRectangularBoard(float markerSize, float xDist, float yDist, const cv::Ptr<cv::aruco::Dictionary>& dictionary, const std::vector<int>& ids);
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;
protected:
    void findCenter();
    /**
     * Converts a rotation in the axis-angle representation to a quaternion
     * @param aa rotation vector with rotation axis in the direction it points and angle "norm(aa)"
     * @returns quaternion of the rotation vector
    */
    cv::Matx41d axisAngle2Quaternion (const cv::Matx31d& aa);

    /**
     * Converts OpenCV position- and rotation-representation to pose
     * @param rvec rotation vector in axis-angle representation. This is what Open-cv uses
     * @param tvec translation vector
     * @returns geometry_msgs pose
    */
    geometry_msgs::Pose tvec_rvec2pose(const cv::Vec3d &rvec, const cv::Vec3d &tvec);


    // todo: make below members const

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

};