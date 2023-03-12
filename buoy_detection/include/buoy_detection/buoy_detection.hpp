#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <iostream>

class BouyDetection
{
    private:
        cv::Mat lab_image, dst; 

        void normalize_channel(const cv::Mat& src, cv::Mat& dst, int channel);  

    public:

        cv::Mat threshold_channel(cv::Mat img, int channel); 
        
};

