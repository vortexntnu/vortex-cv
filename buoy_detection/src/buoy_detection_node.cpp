#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "buoy_detection/buoy_detection.hpp"

class BuoyDetectionNode
{
    private:
    image_transport::Publisher img_pub; 
    image_transport::Subscriber img_sub; 

    BouyDetection bd; 


    public:
    BuoyDetectionNode(ros::NodeHandle& nh){

        image_transport::ImageTransport it(nh);
        img_sub = it.subscribe("/images/raw", 10, &BuoyDetectionNode::callback); //dont need "this" ? 
        img_pub = it.advertise("/images/out", 10); //dont need type? 
    }

    void callback(sensor_msgs::ImageConstPtr& img_source){

        cv::Mat raw_image = cv_bridge::toCvShare(img_source, "bgr8")->image;

        cv::Mat threholded_img = bd.threshold(raw_image); 

        publish_cvImg(threholded_img); 

    }

    void publish_cvImg(const cv::Mat& img_out){

        std_msgs::Header header; // empty header
        static size_t counter{0};
        header.seq = counter++; // user defined counter
        header.stamp = ros::Time::now(); // time

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "brg8", img_out).toImageMsg();
        img_pub.publish(msg); 
    }

    void spin(){
        static cv::Mat raw_image = cv::imread("/vortex_ws/src/vortex-cv/buoy_detection/test/images.png"); 
    }
}; 


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "buoy_detection_node");
    ros::NodeHandle nh; 
    BuoyDetectionNode wrapper(nh);
   
    ros::spin();

    return 0;
}