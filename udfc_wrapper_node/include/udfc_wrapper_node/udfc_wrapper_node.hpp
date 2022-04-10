#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Example parameters
const float fx=700.819;
const float fy=700.819;
const float cx=665.465;
const float cy=371.953;
const float k1=-0.6;
const float k2=1;


class UDFCWrapperNode
{
private:
    ros::Publisher ros_image_raw_publisher;
    ros::Publisher ros_image_rect_publisher;
    std_msgs::Header header;
    sensor_msgs::Image ros_image_raw;
    sensor_msgs::Image ros_image_rect;
    std::string image_raw_topic;
    std::string image_rect_topic;
    std::string camera_frame;
    cv_bridge::CvImage img_bridge;
    cv::Mat _cv_image;
    int _camera_id = 0;
    int counter_raw = 0;
    int counter_rect = 0;

    std::vector<double> calibrationParams{fx,fy,cx,cy,k1,k2};
    cv::Matx33f CameraMatrix;
    std::vector<double> distortionCoefficents;
   
    void getCVImage();
    void toImageRaw(cv::Mat cv_image);
    void toImageRect(cv::Mat cv_image);

    void getDistortionCoefficents();
    void getCameraMatrix();
    
public:
    UDFCWrapperNode(ros::NodeHandle nh);

    // Temp
};
