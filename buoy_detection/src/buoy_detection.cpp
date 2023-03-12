#include "buoy_detection/buoy_detection.hpp"

cv::Mat BouyDetection::threshold_channel(cv::Mat img, int channel){

    if (img.empty()){
        ROS_INFO("Image is empty"); 
    }
    cv::Mat mono_normal_image; 
    cv::Mat thresholded_mg; 
    
    // cv::Mat different_Channels[3];

    // cv::split(img, different_Channels);//splitting images into 3 different channels//  
    // cv::Mat mono_image = different_Channels[channel];//loading blue channels//
    
    normalize_channel(img, mono_normal_image, channel); 

    cv::threshold(mono_normal_image, thresholded_mg, 0, 255, CV_THRESH_BINARY); 

    return thresholded_mg; 

}; 

void BouyDetection::normalize_channel(const cv::Mat& src, cv::Mat& dst, int channel){

    dst.create(src.rows, src.cols, CV_8UC1); //belive that this is for 8 bits, unsigned, depth = 1. 

    for (int r=0; r++; r < src.rows){
        for (int c=0; c++; c < src.cols){
            cv::Vec3b pixel = src.at<cv::Vec3b>(r, c);
            dst.at<uchar>(r, c) = 100*pixel[channel]/(pixel[0] + pixel[1] + pixel[2]); 

            ROS_INFO("%d", pixel[channel]/(pixel[0] + pixel[1] + pixel[2])); 
        }
    }

}; 