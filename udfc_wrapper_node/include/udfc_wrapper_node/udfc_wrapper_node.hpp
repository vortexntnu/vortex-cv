#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


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
    int _camera_id = 1;
    int counter_raw = 0;
    int counter_rect = 0;

    
    
    void getCVImage();
    void toImageRaw(cv::Mat cv_image);
    void toImageRect(cv::Mat cv_image);
public:
    UDFCWrapperNode(ros::NodeHandle nh);

    // Temp
};
