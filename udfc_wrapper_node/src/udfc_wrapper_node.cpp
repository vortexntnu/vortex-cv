#include "udfc_wrapper_node/udfc_wrapper_node.hpp"

UDFCWrapperNode::UDFCWrapperNode(ros::NodeHandle nh)
{
    image_raw_topic = "udfc/wrapper/camera_raw";
    image_rect_topic = "udfc/wrapper/camera_rect";
    ros_image_raw_publisher = nh.advertise<sensor_msgs::Image>(image_raw_topic,10);
    ros_image_rect_publisher = nh.advertise<sensor_msgs::Image>(image_rect_topic,10);
    camera_frame = "udfc";
    getCVImage();
}

void UDFCWrapperNode::getCVImage()
{
    // cv::namedWindow("Display window");
    cv::VideoCapture cap(_camera_id);
    if (!cap.isOpened())
    {
        std::cout << "cannot open camera"; // Needs to be changed to work with ROS
    }
    while (true){
        cap >> _cv_image;
        toImageRaw(_cv_image);
        // cv::imshow("Display window", _cv_image);
        // cv::waitKey(25);
    }
    cv::destroyAllWindows();
}

void UDFCWrapperNode::toImageRaw(cv::Mat cv_image)
{
    header.seq = counter_raw;
    counter_raw += 1;
    header.stamp = ros::Time::now();
    header.frame_id = camera_frame;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_image);
    img_bridge.toImageMsg(ros_image_raw);
    ros_image_raw_publisher.publish(ros_image_raw);
}

void UDFCWrapperNode::toImageRect(cv::Mat cv_image)
{
    header.seq = counter_rect;
    counter_rect += 1;
    header.stamp = ros::Time::now();
    header.frame_id = camera_frame;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_image);
    img_bridge.toImageMsg(ros_image_rect);
    ros_image_rect_publisher.publish(ros_image_rect);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udfc_wrapper_node");
    ros::NodeHandle nh;
    UDFCWrapperNode wrapper(nh);
    ros::spin();

    return 0;
}