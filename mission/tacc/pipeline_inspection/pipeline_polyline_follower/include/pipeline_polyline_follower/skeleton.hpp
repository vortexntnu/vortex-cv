#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * Zhang-Suen thinning: reduces a binary (0/255) mask to a 1-pixel-wide
 * skeleton in-place.
 */
void zhang_suen_thin(cv::Mat& img);

/**
 * Trace the skeleton into an ordered pixel path.
 *
 * Finds endpoint pixels (exactly one 8-connected neighbor), then walks the
 * skeleton from one endpoint to the other.  If the skeleton has branches, the
 * longest simple path between any two endpoints is returned.  Returns an empty
 * vector when the skeleton has no foreground pixels.
 */
std::vector<cv::Point> trace_skeleton(const cv::Mat& skeleton);
