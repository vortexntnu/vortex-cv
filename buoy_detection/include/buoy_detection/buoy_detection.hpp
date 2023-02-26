#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <iostream>

class BouyDetection
{
    private:
        cv::Mat lab_image, dst; 

    public:

        cv::Mat threshold(cv::Mat img); 
        

};

