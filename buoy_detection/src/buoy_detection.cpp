#include "Buoy_detection.hpp"

BouyDetection::threashold(){
     // Read image 
    cv::Mat src = imread("threshold.png", IMREAD_GRAYSCALE); 
    cv::Mat dst; 

    // Basic threhold example 
    cv::threshold(src,dst,0, 255, THRESH_BINARY); 
    cv::imwrite("opencv-threshold-example.jpg", dst); 
}