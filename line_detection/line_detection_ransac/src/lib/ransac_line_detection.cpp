#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <vortex/utils/types.hpp>
#include "line_detection_ransac/lib/detection_utils.hpp"
#include "line_detection_ransac/lib/line_detection_ransac.hpp"
#include "line_detection_ransac/lib/typedefs.hpp"
#include "line_detection_ransac/lib/utils.hpp"
#include <vortex/cv_utils/image_conversions.hpp>

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

    cv::Mat gray8 = vortex::cv_utils::to_gray8(input_image);
    std::vector<cv::Point> boundary_points;

    detect_boundaries(boundary_config_, gray8, boundary_points);

    cv::Mat debug_image;
    if (mode == DetectorMode::debug) {
        cv::cvtColor(gray8, debug_image, cv::COLOR_GRAY2BGR);
        for (const auto& pt : boundary_points) {
            cv::circle(debug_image, pt, 3, cv::Scalar(0, 0, 255), -1);
        }
    }

    std::vector<cv::Vec4i> cv_lines;

    detect_lines(ransac_config_, boundary_points, cv_lines);

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
