#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>
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
 * A filter that worked well-ish in the mc-lab conditions easter 2023
 * Uses a combination of dilation and unsharpening
 */
void mcLabFilter(const cv::Mat &original, cv::Mat &filtered);

/**
 * White Balancing Filter
 */
void whiteBalanceFilter(const cv::Mat &original, cv::Mat &filtered, double contrastPercentage = 0.2);