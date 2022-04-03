#include "udfc_wrapper_node/udfc_wrapper_node.hpp"

UDFCWrapperNode::UDFCWrapperNode(ros::NodeHandle nh)
{
    cv::namedWindow("Display window");
    cv::VideoCapture cap(_camera_id);
    if (!cap.isOpened())
    {
        std::cout << "cannot open camera"; // Needs to be changed to work with ROS
    }
    while (true){
        cap >> cv_image;
        cv::imshow("Display window", cv_image);
        cv::waitKey(25);
    }
}

void UDFCWrapperNode::getCVImage()
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udfc_wrapper_node");
    ros::NodeHandle nh;
    UDFCWrapperNode wrapper(nh);
    ros::spin();

    return 0;
}