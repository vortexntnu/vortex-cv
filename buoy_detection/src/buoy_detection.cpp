#include "buoy_detection/buoy_detection.hpp"

cv::Mat BouyDetection::threshold_channel(cv::Mat img, int channel){

    if (img.empty()){
        ROS_INFO("Image is empty"); 
    }
    cv::Mat dst; 
    // cv::Mat different_Channels[3];

    // cv::split(img, different_Channels);//splitting images into 3 different channels//  
    // cv::Mat mono_image = different_Channels[channel];//loading blue channels//
    
    cv::Mat mono_normal_image = normalize_channel(img, channel); 

    cv::threshold(mono_normal_image, dst, 1, 255, CV_THRESH_BINARY); 

    return dst; 

}; 

cv::Mat BouyDetection::normalize_channel(cv::Mat img, int channel){

    cv::Mat dst;
    dst.create(img.rows, img.cols, CV_8UC1); //belive that this is for 8 bits, unsigned, depth = 1. 

    for (int r=0; r++; r < img.rows){
        for (int c=0; c++; c < img.cols){
            cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);
            dst.at<uchar>(r, c) = 100*pixel[channel]/(pixel[0] + pixel[1] + pixel[2]); 
        }
    }

    return dst; 

}; 