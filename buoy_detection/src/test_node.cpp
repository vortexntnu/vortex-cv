#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "buoy_detection/buoy_detection.hpp"

class BuoyDetectionTestNode
{
    private:
    image_transport::Publisher img_pub; 
    image_transport::Subscriber img_sub; 
    
    public:
    BuoyDetectionTestNode(ros::NodeHandle *nh){
        image_transport::ImageTransport it(nh);

        img_sub = it.subscribe("/images/out", 10, &BuoyDetectionTestNode::callback); 
        img_pub = it.advertise("/images/raw", 10); 

        ros::NodeHandle

    }

    void callback(sensor_msgs::Image msg){

        // try
        // {
        //     cv::Mat raw_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            
        // }
        // catch (cv_bridge::Exception& e)
        // {
        //     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        // }
        
    }; 

    void publish_cvImg(const cv::Mat& img_out){

        std_msgs::Header header; // empty header
        static size_t counter{0};
        header.seq = counter++; // user defined counter
        header.stamp = ros::Time::now(); // time

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "brg8", img_out).toImageMsg();
        img_pub.publish(msg); 
    }

    void spin(){
        ros::Rate loop_rate(5); 
        while (nh.ok){

            static cv::Mat raw_image = cv::imread("/vortex_ws/src/vortex-cv/buoy_detection/test/images.png"); 
            publish_cvImg(raw_image); 

            ros::spinOnce(); 
            loop_rate.sleep(); 
        }; 
    }
}; 


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "buoy_detection_test_node");
    ros::NodeHandle nh; 
    BuoyDetectionTestNode wrapper(&nh);
    wrapper.spin(); 

    return 0;
}