#include "lib/depth_to_points.hpp"

#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <vector>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace vortex::board_detection {

std::vector<geometry_msgs::msg::Point32> depthToPoints(
    const cv::Mat& depth_image,
    const cv::Mat& board_mask,
    double horizontal_fov,
    double vertical_fov
) {
    std::vector<geometry_msgs::msg::Point32> pointCloud;

    if (depth_image.size() != board_mask.size()) {
        throw std::runtime_error(
            "Depth image and board mask have different dimensions"
        );
    }

    if (depth_image.type() != CV_32FC1 ||
        board_mask.type() != CV_8UC1) {
        throw std::runtime_error("Invalid image format");
    }

    constexpr double pi = 3.14159265358979323846;

    if (!std::isfinite(horizontal_fov) ||
        !std::isfinite(vertical_fov) ||
        horizontal_fov <= 0.0 ||
        vertical_fov <= 0.0 ||
        horizontal_fov >= pi ||
        vertical_fov >= pi) {
        throw std::runtime_error("Invalid camera FOV");
    }

    const int width = depth_image.cols;
    const int height = depth_image.rows;

    pointCloud.reserve(width * height);

    const double fx = width / (2.0 * std::tan(horizontal_fov / 2.0));
    const double fy = height / (2.0 * std::tan(vertical_fov / 2.0));

    const double cx = (width - 1) / 2.0;
    const double cy = (height - 1) / 2.0;

    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {

            if (board_mask.at<uint8_t>(v, u) == 0)
                continue;

            const float depth = depth_image.at<float>(v, u);

            if (!std::isfinite(depth) || depth <= 0.0f)
                continue;

            geometry_msgs::msg::Point32 point;
            point.x = (u - cx) * depth / fx; // Right
            point.y = (v - cy) * depth / fy; // Down
            point.z = depth;                 // Forward

            pointCloud.push_back(point);
        }
    }

    return pointCloud;
}

}