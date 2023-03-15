#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "buoy_detection/buoy_detection.hpp"


// See this example from ros wiki for converting between ROS images and OpenCV images; http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages 

class BuoyDetectionNode
{
    private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it; 
    image_transport::Publisher img_pub_BRG8; 
    image_transport::Publisher img_pub_MONO8; 
    image_transport::Subscriber img_sub; 

    BouyDetection bd; 

    public:
    BuoyDetectionNode(): it(nh){

        img_sub = it.subscribe("image_in", 10, &BuoyDetectionNode::callback, this); 
        img_pub_BRG8 = it.advertise("brg_image_out", 10); 
        img_pub_MONO8 = it.advertise("mono_image_out", 10); 
    }

    void callback(const sensor_msgs::ImageConstPtr& img_source){


        cv_bridge::CvImageConstPtr raw_image_ptr;
        try
        {
        raw_image_ptr= cv_bridge::toCvShare(img_source, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

        cv::Mat thresholded_green_image = bd.threshold_channel(raw_image_ptr->image, 0); // 0 - blue, 1 - green, 2 - red
        publish_MONO8_cvImg(thresholded_green_image); 
        publish_BRG8_cvImg(raw_image_ptr->image); 

    }


    void publish_BRG8_cvImg(const cv::Mat& img_out){

        std_msgs::Header header; // empty header
        static size_t counter{0};
        header.seq = counter++; // user defined counter
        header.stamp = ros::Time::now(); // time

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_out).toImageMsg();
        img_pub_BRG8.publish(msg); 
    }

    void publish_MONO8_cvImg(const cv::Mat& img_out){

        std_msgs::Header header; // empty header
        static size_t counter{0};
        header.seq = counter++; // user defined counter
        header.stamp = ros::Time::now(); // time

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_out).toImageMsg();
        img_pub_MONO8.publish(msg); 
    }

}; 


int main(int argc, char **argv)
{   

    ros::init(argc, argv, "buoy_detection_node"); 
    BuoyDetectionNode wrapper;
    ros::spin();

    return 0;
}