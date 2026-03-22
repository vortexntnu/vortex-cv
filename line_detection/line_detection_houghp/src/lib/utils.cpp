#include "line_detection_houghp/lib/utils.hpp"
#include <vortex/cv_utils/image_conversions.hpp>

#include <opencv2/imgproc.hpp>
#include <vector>

namespace vortex::line_detection {

constexpr int k_line_thickness = 2;
constexpr int k_line_type = cv::LINE_AA;
const cv::Scalar k_line_color(0, 0, 255);  // BGR red

cv::Mat make_overlay_color(const cv::Mat& input_bgr,
                           const cv::Mat& gray8,
                           const std::vector<cv::Vec4i>& lines) {
    if (input_bgr.empty()) {
        throw std::runtime_error("make_overlay_color: input_bgr is empty");
    }
    cv::Mat out_bgr = vortex::cv_utils::to_bgr8(input_bgr, gray8);
    for (const auto& l : lines) {
        const cv::Point p0(l[0], l[1]);
        const cv::Point p1(l[2], l[3]);
        cv::line(out_bgr, p0, p1, k_line_color, k_line_thickness, k_line_type);
    }
    return out_bgr;
}

cv::Mat make_overlay_canny(const cv::Mat& edges,
                           const std::vector<cv::Vec4i>& lines) {
    if (edges.empty()) {
        throw std::runtime_error("make_overlay_canny: edges is empty");
    }
    if (edges.type() != CV_8UC1) {
        throw std::runtime_error("make_overlay_canny: edges must be CV_8UC1");
    }

    cv::Mat out_bgr;
    cv::cvtColor(edges, out_bgr, cv::COLOR_GRAY2BGR);

    for (const auto& l : lines) {
        const cv::Point p0(l[0], l[1]);
        const cv::Point p1(l[2], l[3]);
        cv::line(out_bgr, p0, p1, k_line_color, k_line_thickness, k_line_type);
    }
    return out_bgr;
}

}  // namespace vortex::line_detection
