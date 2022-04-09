#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <gSLICr.h>
#include "engines/gSLICr_core_engine.h"


class SLICNode
{
private:
    ros::NodeHandle *m_nh;
    ros::Publisher slic_img_pub;
    ros::Subscriber image_sub;
    std::string slic_image_topic;

    gSLICr::objects::settings my_settings;
    gSLICr::engines::core_engine* gSLICr_engine;
    gSLICr::UChar4Image* in_img;
    gSLICr::UChar4Image* out_img;
    
    cv_bridge::CvImage img_bridge;
    cv::Mat _cv_image;
    
    void image_cb(const sensor_msgs::Image::ConstPtr& msg);

public:
    SLICNode(ros::NodeHandle *nh);
};


    