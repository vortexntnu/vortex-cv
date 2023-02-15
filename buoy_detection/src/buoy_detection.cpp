#include "buoy_detection/buoy_detection.hpp"

void BouyDetection::threshold(){

    cv::Mat image = imread("/vortex_ws/src/vortex-cv/buoy_detection/test/images.png", cv::IMREAD_GRAYSCALE); 
    if (image.empty()){
        ROS_INFO("Image is empty"); 
    }
    cv::Mat dst; 
    
    // Basic threhold example 
    cv::threshold(image,dst,200, 255, cv::THRESH_BINARY); 


    uint8_t *myData = image.data;
    int width = image.cols;
    int height = image.rows;
    ROS_INFO("w %i, h %i", width, height); 

    uint8_t *myNewData = dst.data;
    width = image.cols;
    height = image.rows;
    ROS_INFO("w %i, h %i", width, height); 

    int _stride = image.step;//in case cols != strides
    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
            uint8_t val = myNewData[ i * _stride + j];
            ROS_INFO("%i", val); 
            //do whatever you want with your value
        }
    }





}