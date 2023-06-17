#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>

#include <image_filters/imgFilterConfig.h>

/**
 * Makes edges harder
 */
void sharpeningFilter(const cv::Mat &original, cv::Mat &modified);
/**
 * Makes edges harder in a smarter way
 */
void unsharpeningFilter(const cv::Mat &original, cv::Mat &modified, size_t blurSize = 1, double sharpness = 1.5);

/**
 * Expands the dark areas of the image
 */
void erodingFilter(const cv::Mat &original, cv::Mat &modified, size_t erosionSize = 1);

/**
 * Expands the bright areas of the image
 */
void dilatingFilter(const cv::Mat &original, cv::Mat &modified, size_t dilationSize = 1);

/**
 * White Balancing Filter
 */
void whiteBalanceFilter(const cv::Mat &original, cv::Mat &filtered, double contrastPercentage = 0.2);

/**
 * A filter that worked well-ish in the mc-lab conditions easter 2023
 * Uses a combination of dilation and unsharpening
 */
void ebusFilter(const cv::Mat &original, cv::Mat &filtered, size_t erosionSize = 2, size_t blurSize = 30, size_t maskWeight = 5);

void filter_from_rqt(const cv::Mat &original, cv::Mat &filtered, image_filters::imgFilterConfig &config);
