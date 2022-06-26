#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class ImagePreprocessingNode
{
    private:
        cv_bridge::CvImage img_bridge;
        cv::Mat cv_img;


    public:
        std::string image_topic;

        ros::Publisher CLAHEPub;
        ros::Subscriber ImgTopicSub;
        void image_callback(const sensor_msgs::Image::ConstPtr& msg);
        void cv_image_publisher(cv::Mat img);

}