#include "line_detection_ransac/lib/utils.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>
#include <vortex/cv_utils/image_conversions.hpp>

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

cv::Mat make_overlay_boundaries(const cv::Mat& input_bgr,
                                const std::vector<cv::Vec4i>& lines) {
    if (input_bgr.empty()) {
        throw std::runtime_error("make_overlay_boundaries: input_bgr is empty");
    }

    if (input_bgr.channels() != 3) {
        throw std::runtime_error(
            "make_overlay_boundaries: input must be 3-channel BGR");
    }

    // Clone input so we don't overwrite it
    cv::Mat out_bgr = input_bgr.clone();

    // Draw all lines
    for (const auto& l : lines) {
        cv::line(out_bgr, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                 cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    return out_bgr;
}

}  // namespace vortex::line_detection
