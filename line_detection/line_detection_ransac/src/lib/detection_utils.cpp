#ifndef LINE_DETECTION_HOUGHP__LIB__DETECTION_UTILS_HPP_
#define LINE_DETECTION_HOUGHP__LIB__DETECTION_UTILS_HPP_

#include "line_detection_ransac/lib/detection_utils.hpp"
#include <spdlog/spdlog.h>

namespace vortex::line_detection {

void detect_boundaries(const BoundaryConfig& boundary_config,
                       const cv::Mat& input_image,
                       std::vector<cv::Point>& boundary_points) {
    constexpr float deg2rad = M_PI / 180.0f;
    const float min_angle = -boundary_config.angle / 2.0f * deg2rad;
    const float max_angle = boundary_config.angle / 2.0f * deg2rad;
    const int num_rays = boundary_config.num_rays;
    const int cols = input_image.cols;
    const int rows = input_image.rows;
    const int threshold = boundary_config.threshold;
    const float step = boundary_config.step;
    const int sample_side_length = boundary_config.sample_side_length;
    const bool edge_detection = boundary_config.edge_detection;

    if (sample_side_length % 2 == 0) {
        throw std::runtime_error("sample_side_length must be odd");
    }
    const int half_side_length = sample_side_length / 2;

    cv::Point origin(cols / 2, rows);

    for (int h = 0; h < num_rays; h++) {
        float angle = min_angle + h * (max_angle - min_angle) / (num_rays - 1);
        float last_avg = 0.0f;
        bool last_avg_initialized = false;

        for (float i = 0.0f; i < rows; i += step) {
            float xf = origin.x + i * std::sin(angle);
            float yf = origin.y - 2.0f - i * std::cos(angle);

            int x = static_cast<int>(std::round(xf));
            int y = static_cast<int>(std::round(yf));
            if (x < half_side_length || x >= cols - half_side_length ||
                y < half_side_length || y >= rows - half_side_length) {
                continue;
            }

            float avg_value = 0.0f;
            for (int j = -half_side_length; j <= half_side_length; j++) {
                for (int k = -half_side_length; k <= half_side_length; k++) {
                    avg_value += input_image.at<uint8_t>(y + k, x + j);
                }
            }
            avg_value /=
                (static_cast<float>(sample_side_length * sample_side_length));

            if (edge_detection && !last_avg_initialized) {
                last_avg = avg_value;
                last_avg_initialized = true;
                continue;
            }

            if (abs(avg_value - last_avg) > threshold) {
                const float dx = static_cast<float>(x - origin.x);
                const float dy = static_cast<float>(y - origin.y);
                if (std::sqrt(dx * dx + dy * dy) >=
                    boundary_config.min_dist_from_origin) {
                    boundary_points.emplace_back(x, y);
                }
                break;
            }
            if (edge_detection) {
                last_avg = avg_value;
            }
        }
    }
}

void detect_lines(const RansacConfig& ransac_config,
                  const std::vector<cv::Point>& boundary_points,
                  std::vector<cv::Vec4i>& cv_lines) {
    const int points_checked = ransac_config.points_checked;
    const float inlier_threshold = ransac_config.inlier_threshold;
    const int min_remaining_points = ransac_config.min_remaining_points;
    const int min_inliers = ransac_config.min_inliers;
    const int max_distance = ransac_config.max_distance;

    std::vector<cv::Point> boundary_points_copy = boundary_points;

    while (boundary_points_copy.size() >
           static_cast<size_t>(min_remaining_points)) {
        std::vector<int> best_count_idxs;
        int best_count = 0;

        for (size_t i = 0; i < boundary_points_copy.size(); i++) {
            for (size_t j = 1; j < static_cast<size_t>(points_checked); j++) {
                int count = 0;
                std::vector<int> inliers_idxs;

                if (i + j >= boundary_points_copy.size()) {
                    break;
                }

                if (boundary_points_copy[i].x ==
                    boundary_points_copy[i + j].x) {
                    for (size_t k = 0; k < boundary_points_copy.size(); k++) {
                        if (std::abs(boundary_points_copy[k].x -
                                     boundary_points_copy[i].x) <=
                            inlier_threshold) {
                            count++;
                            inliers_idxs.push_back(k);
                        }
                    }
                } else if (boundary_points_copy[i].y ==
                           boundary_points_copy[i + j].y) {
                    for (size_t k = 0; k < boundary_points_copy.size(); k++) {
                        if (std::abs(boundary_points_copy[k].y -
                                     boundary_points_copy[i].y) <=
                            inlier_threshold) {
                            count++;
                            inliers_idxs.push_back(k);
                        }
                    }
                } else {
                    float x1 = boundary_points_copy[i].x;
                    float y1 = boundary_points_copy[i].y;
                    float x2 = boundary_points_copy[i + j].x;
                    float y2 = boundary_points_copy[i + j].y;

                    float a = y2 - y1;
                    float b = x1 - x2;
                    float c = x2 * y1 - x1 * y2;

                    float norm = std::sqrt(a * a + b * b);

                    for (size_t k = 0; k < boundary_points_copy.size(); k++) {
                        if ((std::abs(a * boundary_points_copy[k].x +
                                      b * boundary_points_copy[k].y + c) /
                             norm) <= inlier_threshold) {
                            count++;
                            inliers_idxs.push_back(k);
                        }
                    }
                }

                if (count > best_count) {
                    best_count = count;
                    best_count_idxs = inliers_idxs;
                }
            }
        }

        if (best_count_idxs.size() < 2) {
            break;
        }

        float dist1 =
            std::sqrt(std::pow(boundary_points_copy[best_count_idxs[0]].x -
                                   boundary_points_copy[best_count_idxs[1]].x,
                               2) +
                      std::pow(boundary_points_copy[best_count_idxs[0]].y -
                                   boundary_points_copy[best_count_idxs[1]].y,
                               2));
        float dist2 = std::sqrt(
            std::pow(boundary_points_copy
                             [best_count_idxs[best_count_idxs.size() - 2]]
                                 .x -
                         boundary_points_copy
                             [best_count_idxs[best_count_idxs.size() - 1]]
                                 .x,
                     2) +
            std::pow(boundary_points_copy
                             [best_count_idxs[best_count_idxs.size() - 2]]
                                 .y -
                         boundary_points_copy
                             [best_count_idxs[best_count_idxs.size() - 1]]
                                 .y,
                     2));

        if (dist1 > max_distance) {
            best_count_idxs.erase(best_count_idxs.begin());
            best_count--;
        }
        if (dist2 > max_distance) {
            best_count_idxs.erase(best_count_idxs.end() - 1);
            best_count--;
        }

        if (best_count < min_inliers) {
            break;
        }

        cv_lines.push_back(cv::Vec4i(
            boundary_points_copy[best_count_idxs[0]].x,
            boundary_points_copy[best_count_idxs[0]].y,
            boundary_points_copy[best_count_idxs[best_count_idxs.size() - 1]].x,
            boundary_points_copy[best_count_idxs[best_count_idxs.size() - 1]]
                .y));

        for (int i = best_count_idxs.size() - 2; i > 0; i--) {
            boundary_points_copy.erase(boundary_points_copy.begin() +
                                       best_count_idxs[i]);
        }
    }
}

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_HOUGHP__LIB__DETECTION_UTILS_HPP_
