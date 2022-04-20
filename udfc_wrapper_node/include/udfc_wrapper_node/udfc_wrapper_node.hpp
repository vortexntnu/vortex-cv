#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Example parameters



class UDFCWrapperNode
{
private:
    ros::Publisher ros_image_raw_publisher;
    ros::Publisher ros_image_rect_publisher;
    
    std_msgs::Header header;
    sensor_msgs::Image ros_image_raw;
    sensor_msgs::Image ros_image_rect;
    
    std::string image_raw_topic;
    std::string image_rect_topic;
    std::string camera_frame;
    
    std::string image_pipeline_prefix;
    std::string image_pipeline;
    
    cv_bridge::CvImage img_bridge;
    cv::Mat _cv_image;

    cv::Size s;
    cv::Size s1;
    cv::Rect a{};
    cv::Rect *rectPointer = &a;
    bool initFlag = 0;
    
    int counter_raw = 0;
    int counter_rect = 0;
   

    cv::Matx33f CameraMatrix;
    std::vector<double> distortionCoefficents;
   
    void getCVImage();
    void toImageRaw(cv::Mat cv_image);
    void toImageRect(cv::Mat cv_image);

    void getDistortionCoefficents();
    void getCameraMatrix();
    
public:
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float p1;
    float p2;
    float k3;
    int _camera_id;
    int _pipeline_id;


    UDFCWrapperNode(ros::NodeHandle nh);
    std::vector<double> calibrationParams{fx,fy,cx,cy,k1,k2,p1,p2,k3};
    std::vector<std::string> paramNames{"fx","fy","cx","cy","k1","k2","p1","p2","k3"};


    // Temp
};
