#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "buoy_detection/buoy_detection.hpp"

class BuoyDetectionNode
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    BouyDetection detector; 

    

private:
    std::string image_topic{"/zed2/zed_node/rgb/image_rect_color"};
    std::string clahe_topic{"/image_preprocessing/clahe"}; //TODO change


public:
    BuoyDetectionNode()
        : it_(nh_)
        {
            image_sub_ = it_.subscribe(image_topic, 1, &BuoyDetectionNode::image_callback, this);
            detector.threshold(); 
        }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        // ImagePreprocessing image_preprocessing_(clahe_clip_limit, clahe_tile_grid_size);
        // std_msgs::Header header = msg->header;
        // cv::Mat cv_image_;
        // try
        // {
        //     cv_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        //     image_pub_.publish(cv_bridge::CvImage(header, "bgr8", clahe_image_).toImageMsg());
        // }
        // catch(const std::exception& e)
        // {
        //     ROS_ERROR("cv_bridge exception: %s", e.what());
        //     return;
        // }

    }

    // void image_publish(cv_bridge::CvImagePtr cv_ptr)
    // {
    //     image_pub_.publish(cv_ptr -> toImageMsg());
    // }
    
};

int main(int argc, char **argv)
{   
    std::cout<<"hei"; 
    ros::init(argc, argv, "buoy_detection_node");
    ros::NodeHandle nh;
    BuoyDetectionNode wrapper;
   
    ros::spin();

    return 0;
}