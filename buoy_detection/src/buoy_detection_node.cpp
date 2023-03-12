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
    image_transport::Publisher img_pub; 
    image_transport::Subscriber img_sub; 

    BouyDetection bd; 


    public:
    BuoyDetectionNode(): it(nh){

        img_sub = it.subscribe("/images/raw", 10, &BuoyDetectionNode::callback, this); //dont need "this" ? 
        img_pub = it.advertise("/images/out", 10); //dont need type? 
    }

    void callback(const sensor_msgs::ImageConstPtr& img_source){

        // cv::Mat raw_image = cv_bridge::toCvShare(img_source, "bgr8")->image;

        cv_bridge::CvImageConstPtr raw_image_ptr;
        try
        {
        raw_image_ptr= cv_bridge::toCvShare(img_source, "bgr8");
        }
        catch (cv_bridge::Exception& e)
        {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

        //cv::Mat threholded_img = bd.threshold(raw_image_ptr->image); 
        publish_cvImg(raw_image_ptr->image); 

    }


    void publish_cvImg(const cv::Mat& img_out){

        std_msgs::Header header; // empty header
        static size_t counter{0};
        header.seq = counter++; // user defined counter
        header.stamp = ros::Time::now(); // time

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "brg8", img_out).toImageMsg();
        img_pub.publish(msg); 
    }

    // void spin(){
    //     static cv::Mat raw_image = cv::imread("/vortex_ws/src/vortex-cv/buoy_detection/test/images.png"); 
    // }
}; 


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "buoy_detection_node"); 
    BuoyDetectionNode wrapper();
    ros::spin();

    return 0;
}