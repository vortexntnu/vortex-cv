#pragma once

#include <opencv2/core.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <vector>

namespace vortex::board_detection {

std::vector<geometry_msgs::msg::Point32> depthToPoints(
    const cv::Mat& depth_image,
    const cv::Mat& board_mask,
    double horizontal_fov,
    double vertical_fov
);

}