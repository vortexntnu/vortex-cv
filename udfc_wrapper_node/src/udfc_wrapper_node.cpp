#include "udfc_wrapper_node/udfc_wrapper_node.hpp"


//DIstortian params for testing
using namespace std;
float fx{700.819};
float fy{700.819};
float cx{665.465};
float cy=cx/2;
float k1{-0.174318};
float k2{0.0261121};
cv::InputArray CamMat = ((fx,0,cx),(0,fy,cy),(0,0,1));
cv::InputArray dist = NULL;//((k1,k2,0),(k1,k2,0),(k1,k2,0));

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
        //cv::Mat dst = _cv_image.clone();
        //cv::undistort(_cv_image,dst,NULL,NULL);
       // cv::imshow("Display window", dst);
       // cv::waitKey(0);
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