
#include "udfc_wrapper_node/udfc_wrapper_node.hpp"
#include <opencv2/core/matx.hpp>





void UDFCWrapperNode::getCameraMatrix(){
// This function turns the calibration parameters into a camera matrix.

double fx{calibrationParams[0]};
double fy{calibrationParams[1]};
double cx{calibrationParams[2]};                      
double cy{calibrationParams[3]};

cv::Matx33f Matrix(fx, 0, cx,
          0, fy, cy,
          0, 0, 1);

CameraMatrix=Matrix;
}


void UDFCWrapperNode::getDistortionCoefficents(){
// This function turns the calibration parameters into a vector of distortion params.

    double k1{calibrationParams[4]};
    double k2{calibrationParams[5]};
    distortionCoefficents = {k1,k2,0,0};
}




UDFCWrapperNode::UDFCWrapperNode(ros::NodeHandle nh)
{   
    //Getting rosparams
    nh.getParam("camera_id", _camera_id); //Setting camera_id
    for(int i=0; i < paramNames.size();i++){  
        nh.getParam(paramNames[i], calibrationParams[i]);//Getting calibration params
       
    }
   
    
    getCameraMatrix(); //Gives us the camera matrix.
    getDistortionCoefficents(); // Gives us the Distortion coeff.

    image_raw_topic = "udfc/wrapper/camera_raw";
    image_rect_topic = "udfc/wrapper/camera_rect";
    ros_image_raw_publisher = nh.advertise<sensor_msgs::Image>(image_raw_topic,10); 
    ros_image_rect_publisher = nh.advertise<sensor_msgs::Image>(image_rect_topic,10);
    camera_frame = "udfc";
    getCVImage();
}


void UDFCWrapperNode::getCVImage()
{   
    cv::VideoCapture cap(_camera_id);
    if (!cap.isOpened())
    {
        std::cout << "cannot open camera"; // Needs to be changed to work with ROS
    }
    while (ros::ok()){ //Will stop running when Ctrl + c is pressed in terminal.
        cap >> _cv_image;
        toImageRaw(_cv_image);
        toImageRect(_cv_image);
    }
}

void UDFCWrapperNode::toImageRaw(cv::Mat cv_image)
{
    //The function takes in a cv image and converts it into a ros image and publishes it.
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
    //The function takes in a cv image, rectifies it, and converts it into a ros image and publishes it.
    cv::Mat dst = _cv_image.clone(); //Makes a copy of the cv-image.
    cv::undistort(_cv_image,dst,CameraMatrix,distortionCoefficents); //Undistorts the cloned image.
    header.seq = counter_rect;
    counter_rect += 1;
    header.stamp = ros::Time::now();
    header.frame_id = camera_frame;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, dst);
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