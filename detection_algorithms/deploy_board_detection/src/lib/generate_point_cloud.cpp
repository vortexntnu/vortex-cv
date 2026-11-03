#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <vector>
#include <cmath>

std::vector<Eigen::Vector3f> depthToPoints(
    const cv::Mat& depth,     // CV_32FC1, depth in meters
    const cv::Mat& mask,      // CV_8UC1, 0 or 255
    float horizontal_fov      // radians
) {
    std::vector<Eigen::Vector3f> points;

    const int W = depth.cols;
    const int H = depth.rows;

    const float fx = W / (2.0f * std::tan(horizontal_fov / 2.0f));
    const float fy = fx;

    const float cx = (W - 1) / 2.0f;
    const float cy = (H - 1) / 2.0f;

    for (int v = 0; v < H; ++v) {
        for (int u = 0; u < W; ++u) {

            // Ignore pixels outside segmentation mask
            if (mask.at<uint8_t>(v, u) == 0)
                continue;

            float z = depth.at<float>(v, u);

            // Ignore invalid depth
            if (!std::isfinite(z) || z <= 0.0f)
                continue;

            float x = (u - cx) * z / fx;
            float y = (v - cy) * z / fy;

            points.emplace_back(x, y, z);
        }
    }

    return points;
}