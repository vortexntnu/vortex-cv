#include "SLIC_node/SLIC_node.hpp"

#include <opencv2/opencv.hpp>
#include <gSLICr.h>
#include "engines/gSLICr_core_engine.h"


SLICNode::SLICNode(ros::NodeHandle *nh) : m_nh(nh)
{
    // Get Params
    slic_image_topic = "/image_preprocessing/SLIC_img";
    slic_img_pub = m_nh->advertise<sensor_msgs::Image>(slic_image_topic, 10);
    
    image_sub = m_nh->subscribe("/zed2i/zed_node/rgb/image_rect_color", 1, &SLICNode::image_cb, this);
    ROS_INFO("Hello %s", "World");

   	my_settings.img_size.x = 1080;
	my_settings.img_size.y = 720;
	my_settings.no_segs = 200;
	my_settings.spixel_size = 16;
	my_settings.coh_weight = 0.6f;
	my_settings.no_iters = 5;
	my_settings.color_space = gSLICr::XYZ; // gSLICr::CIELAB for Lab, or gSLICr::RGB for RGB
	my_settings.seg_method = gSLICr::GIVEN_SIZE; // or gSLICr::GIVEN_NUM for given number
	my_settings.do_enforce_connectivity = true; // whether or not run the enforce connectivity step
    
    // gSLICr_engine = new gSLICr::engines::core_engine(my_settings);
    // in_img = new gSLICr::UChar4Image(my_settings.img_size, true, true);
    // out_img = new gSLICr::UChar4Image(my_settings.img_size, true, true);

    // cv::Size s(my_settings.img_size.x, my_settings.img_size.y);
	// cv::Mat oldFrame, frame;
	// cv::Mat boundry_draw_frame; boundry_draw_frame.create(s, CV_8UC3);


}

void SLICNode::image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    slic_img_pub.publish(cv_ptr->toImageMsg());
}

// void SLICNode::slic_processing(cv::Mat cv_image)
// {
//     header.seq = counter_raw;
//     counter_raw += 1;
//     header.stamp = ros::Time::now();
//     header.frame_id = camera_frame;
//     img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_image);
//     img_bridge.toImageMsg(ros_image_raw);
//     ros_image_raw_publisher.publish(ros_image_raw);
// }

// void SLICNode::pub_slic_img(cv::Mat cv_image)
// {
//     header.seq = counter_raw;
//     counter_raw += 1;
//     header.stamp = ros::Time::now();
//     header.frame_id = camera_frame;
//     img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_image);
//     img_bridge.toImageMsg(ros_image_raw);
//     ros_image_raw_publisher.publish(ros_image_raw);
// }

// void SLICNode::spin()
// {
//     header.seq = counter_raw;
//     counter_raw += 1;
//     header.stamp = ros::Time::now();
//     header.frame_id = camera_frame;
//     img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_image);
//     img_bridge.toImageMsg(ros_image_raw);
//     ros_image_raw_publisher.publish(ros_image_raw);
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SLICNode");
    ROS_INFO("init done");
    ros::NodeHandle nh;
    ROS_INFO("nh done");
    SLICNode wrapper(&nh);
    ROS_INFO("wrapper done");
    ros::spin();
    ROS_INFO("spin done");

    return 0;
}