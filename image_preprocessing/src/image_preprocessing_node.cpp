#include <ros/ros.h>
#include <image_preprocessing_node.hpp>

#include <bits/stdc++.h>
#include <stdlib.h>

ImagePreprocessingNode::ImagePreprocessingNode(ros::NodeHandle nh)
{
    //image_topic  = "bruh how to get the image topic??"
    //string topic = "bruh how do I split the image topic";

    ImgTopicSub            = nh.subscribe(image_topic, image_callback, 1);

    CLAHEPub               = nh.advertise<sensor_msgs::Image>('/cv/image_preprocessing/CLAHE' + ns, 1)
    //single_CLAHEPub        = nh.advertise<sensor_msgs::Image>('/cv/image_preprocessing/CLAHE_single' + ns, 1)
    //GWPub                  = nh.advertise<sensor_msgs::Image>('/cv/image_preprocessing/GW' + ns, 1)
    
    image_preprocessing = ImagePreprocessing(2, 8)
    
    // First initialization of image shape
    first_image_msg = rospy.wait_for_message(image_topic, Image)
    self.cv_image = self.bridge.imgmsg_to_cv2(first_image_msg, "passthrough")
    self.image_shape = self.cv_image.shape
}

void ImagePreprocessingNode::image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


}

void ImagePreprocessingNode::cv_image_publisher(cv::Mat img)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_preprocessing_node");
    ros::NodeHandle nh;
    ImagePreprocessingNode(nh); // How to get this nh thingy into my function??
    ros::spin();

    return 0;
}