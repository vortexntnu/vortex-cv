#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <vortex/utils/types.hpp>
#include "line_detection_ransac/lib/line_detection_ransac.hpp"
#include "line_detection_ransac/lib/typedefs.hpp"
#include "line_detection_ransac/lib/utils.hpp"

namespace vortex::line_detection {

LineDetectorRansac::LineDetectorRansac(
    const BoundaryConfig& boundary_detection_config,
    const RansacConfig& ransac_config)
    : boundary_config_(boundary_detection_config),
      ransac_config_(ransac_config) {}

Result LineDetectorRansac::detect(const cv::Mat& input_image,
                                  DetectorMode mode) const {
    if (input_image.empty()) {
        throw std::runtime_error(
            "LineDetectorRansac::detect: input_image is empty");
    }

    cv::Mat gray8 = vortex::line_detection::to_gray8(input_image);
    std::vector<cv::Point> boundary_points;

    detect_boundaries(gray8, boundary_points);

    cv::Mat debug_image;
    if (mode == DetectorMode::debug) {
        cv::cvtColor(gray8, debug_image, cv::COLOR_GRAY2BGR);
        for (const auto& pt : boundary_points) {
            cv::circle(debug_image, pt, 3, cv::Scalar(0, 0, 255), -1);
        }
    }

    std::vector<cv::Vec4i> cv_lines;

    detect_lines(boundary_points, cv_lines);

    Result r;
    r.line_segments = to_line_segments(cv_lines);

    switch (mode) {
        case DetectorMode::standard: {
            r.output = NoOutput{};
            break;
        }
        case DetectorMode::visualize: {
            VisualizeOutput v;
            v.overlay_color = vortex::line_detection::make_overlay_color(
                input_image, gray8, cv_lines);
            r.output = std::move(v);
            break;
        }
        case DetectorMode::debug: {
            DebugOutput d;
            d.boundaries = debug_image;
            d.overlay_boundaries =
                vortex::line_detection::make_overlay_boundaries(debug_image,
                                                                cv_lines);
            d.overlay_color = vortex::line_detection::make_overlay_color(
                input_image, gray8, cv_lines);
            r.output = std::move(d);
            break;
        }
    }
    return r;
}

void LineDetectorRansac::detect_lines(
    const std::vector<cv::Point>& boundary_points,
    std::vector<cv::Vec4i>& cv_lines) const {
    const int points_checked = ransac_config_.points_checked;
    const float inlier_threshold = ransac_config_.inlier_threshold;
    const int min_remaining_points = ransac_config_.min_remaining_points;
    const int min_inliers = ransac_config_.min_inliers;

    std::vector<cv::Point> boundary_points_copy = boundary_points;
    std::vector<std::vector<cv::Point>> line_segments;

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

void LineDetectorRansac::detect_boundaries(
    const cv::Mat& input_image,
    std::vector<cv::Point>& boundary_points) const {
    constexpr float deg2rad = M_PI / 180.0f;
    const float min_angle = -boundary_config_.angle / 2.0f * deg2rad;
    const float max_angle = boundary_config_.angle / 2.0f * deg2rad;
    const int num_rays = boundary_config_.num_rays;
    const int cols = input_image.cols;
    const int rows = input_image.rows;
    const int threshold = boundary_config_.threshold;
    const float step = boundary_config_.step;
    const int sample_side_length = boundary_config_.sample_side_length;

    if (sample_side_length % 2 == 0) {
        throw std::runtime_error("sample_side_length must be odd");
    }
    const int half_side_length = sample_side_length / 2;

    cv::Point origin(cols / 2, rows - 1);

    for (int h = 0; h < num_rays; h++) {
        float angle = min_angle + h * (max_angle - min_angle) / (num_rays - 1);

        for (float i = 0.0f; i < rows; i += step) {
            float xf = origin.x + i * std::sin(angle);
            float yf = origin.y - 2.0f - i * std::cos(angle);

            int x = static_cast<int>(std::round(xf));
            int y = static_cast<int>(std::round(yf));

            if (x < 2 || x >= cols - 2 || y < 2 || y >= rows - 2) {
                break;
            }

            float avg_value = 0.0f;
            for (int j = -half_side_length; j <= half_side_length; j++) {
                for (int k = -half_side_length; k <= half_side_length; k++) {
                    avg_value += input_image.at<uint8_t>(y + k, x + j);
                }
            }
            avg_value /=
                (static_cast<float>(sample_side_length * sample_side_length));

            if (avg_value > threshold) {
                boundary_points.emplace_back(x, y);
                break;
            }
        }
    }
}

std::vector<vortex::utils::types::LineSegment2D>
LineDetectorRansac::to_line_segments(const std::vector<cv::Vec4i>& cv_lines) {
    std::vector<vortex::utils::types::LineSegment2D> line_segments;
    line_segments.reserve(cv_lines.size());
    for (const auto& cv_line : cv_lines) {
        line_segments.push_back(vortex::utils::types::LineSegment2D{
            {static_cast<double>(cv_line[0]), static_cast<double>(cv_line[1])},
            {static_cast<double>(cv_line[2]),
             static_cast<double>(cv_line[3])}});
    }
    return line_segments;
}

}  // namespace vortex::line_detection
