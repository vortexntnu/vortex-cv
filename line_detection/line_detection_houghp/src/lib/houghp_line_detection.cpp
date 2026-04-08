#include <opencv2/imgproc.hpp>
#include <vortex/cv_utils/image_conversions.hpp>
#include <vortex/utils/types.hpp>
#include "line_detection_houghp/lib/line_detection_houghp.hpp"
#include "line_detection_houghp/lib/typedefs.hpp"
#include "line_detection_houghp/lib/utils.hpp"

namespace vortex::line_detection {

LineDetectorHoughP::LineDetectorHoughP(const CannyConfig& edge_config,
                                       const HoughPConfig& line_config)
    : canny_config_(edge_config), houghp_config_(line_config) {}

Result LineDetectorHoughP::detect(const cv::Mat& input_image,
                                  DetectorMode mode) const {
    if (input_image.empty()) {
        throw std::runtime_error(
            "LineDetectorHoughP::detect: input_image is empty");
    }
    cv::Mat gray8 = vortex::cv_utils::to_gray8(input_image);
    cv::Mat edge_image;

    detect_edges(gray8, edge_image);

    std::vector<cv::Vec4i> cv_lines;
    detect_line_segments(edge_image, cv_lines);

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
            d.canny = edge_image;
            d.overlay_canny = vortex::line_detection::make_overlay_canny(
                edge_image, cv_lines);
            d.overlay_color = vortex::line_detection::make_overlay_color(
                input_image, gray8, cv_lines);
            r.output = std::move(d);
            break;
        }
    }
    return r;
}

void LineDetectorHoughP::detect_edges(const cv::Mat& input_image,
                                      cv::Mat& edge_image) const {
    cv::Canny(input_image, edge_image, canny_config_.low_threshold,
              canny_config_.high_threshold, canny_config_.aperture_size,
              canny_config_.L2_gradient);
}

void LineDetectorHoughP::detect_line_segments(
    const cv::Mat& edge_image,
    std::vector<cv::Vec4i>& cv_lines) const {
    cv::HoughLinesP(edge_image, cv_lines, houghp_config_.rho,
                    houghp_config_.theta, houghp_config_.threshold,
                    houghp_config_.min_line_length,
                    houghp_config_.max_line_gap);
}

std::vector<vortex::utils::types::LineSegment2D>
LineDetectorHoughP::to_line_segments(const std::vector<cv::Vec4i>& cv_lines) {
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
