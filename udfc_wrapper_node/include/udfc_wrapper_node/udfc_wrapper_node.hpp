#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class UDFCWrapperNode
{
private:
    ros::Publisher ros_image;
    cv::Mat cv_image;
    int _camera_id = 1;

    // Temporary
    
    
    void getCVImage();
    void toRosImage();
public:
    UDFCWrapperNode(ros::NodeHandle nh);

    // Temp
};
