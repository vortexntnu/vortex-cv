#include "buoy_detection/buoy_detection.hpp"

//TODO: 
// - use rosparam for tuning params
// - allaow for thresholding multiple channels
// - use clustering
// - publish to PCP

cv::Mat BouyDetection::threshold_channel(cv::Mat img, int channel){

    if (img.empty()){
        ROS_INFO("Image is empty"); 
    }
    cv::Mat mono_normal_image; 
    cv::Mat thresholded_mg; 
    
    normalize_channel(img, mono_normal_image, channel); 

    cv::threshold(mono_normal_image, thresholded_mg, 250, 255, CV_THRESH_BINARY); 

    return thresholded_mg; 

}; 

void BouyDetection::normalize_channel(const cv::Mat& src, cv::Mat& dst, int channel){


    if (src.empty()){
        ROS_INFO("Src is empty"); 
    }

    cv::Mat different_Channels[3];
    cv::split(src, different_Channels);//splitting images into 3 different channels// 

    cv::Mat intensity_f((different_Channels[0] + different_Channels[1] + different_Channels[2]));     // er det riktig å dele på 3? 

    cv::Mat normalized_f; 
    cv::divide(different_Channels[channel], intensity_f, normalized_f);

    normalized_f.convertTo(dst, CV_8UC1, 255.0); 

    if (dst.empty()){
        ROS_INFO("Dst is empty"); 
    }

}; 

