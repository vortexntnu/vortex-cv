#include "line_detection_sonar/lib/houghp_line_detection.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>
#include <vortex/utils/types.hpp>
#include "line_detection_sonar/lib/typedefs.hpp"
#include "line_detection_sonar/lib/utils.hpp"

namespace vortex::line_detection {

HoughPLineDetector::HoughPLineDetector(const EdgeDetectionConfig& edge_config,
                                       const LineDetectionConfig& line_config)
    : edge_config_(edge_config), line_config_(line_config) {}

Result HoughPLineDetector::detect(const cv::Mat& input_image, Mode mode) const {
    if (input_image.empty()) {
        throw std::runtime_error(
            "HoughPLineDetector::detect: input_image is empty");
    }
    cv::Mat gray8 = vortex::line_detection::to_gray8(input_image);
    cv::Mat edge_image;

    detect_edges(gray8, edge_image);

    std::vector<cv::Vec4i> cv_lines;
    detect_line_segments(edge_image, cv_lines);

    Result r;
    r.line_segments = to_line_segments(cv_lines);

    switch (mode) {
        case Mode::standard: {
            r.output = NoOutput{};
            break;
        }
        case Mode::visualize: {
            VisualizeOutput v;
            vortex::line_detection::make_overlay_color(
                input_image, gray8, cv_lines, v.overlay_color);
            r.output = std::move(v);
            break;
        }
        case Mode::debug: {
            DebugOutput d;
            d.canny = edge_image;
            vortex::line_detection::make_overlay_canny(edge_image, cv_lines,
                                                       d.overlay_canny);
            vortex::line_detection::make_overlay_color(
                input_image, gray8, cv_lines, d.overlay_color);
            r.output = std::move(d);
            break;
        }
    }
    return r;
}

void HoughPLineDetector::detect_edges(const cv::Mat& input_image,
                                      cv::Mat& edge_image) const {
    cv::Canny(input_image, edge_image, edge_config_.low_threshold,
              edge_config_.high_threshold, edge_config_.aperture_size,
              edge_config_.L2_gradient);
}

void HoughPLineDetector::detect_line_segments(
    const cv::Mat& edge_image,
    std::vector<cv::Vec4i>& cv_lines) const {
    cv::HoughLinesP(edge_image, cv_lines, line_config_.rho, line_config_.theta,
                    line_config_.threshold);
}

std::vector<vortex::utils::types::LineSegment2D> to_line_segments(
    const std::vector<cv::Vec4i>& cv_lines) {
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
