#ifndef LINE_DETECTION_HOUGHP__LIB__UTILS_HPP_
#define LINE_DETECTION_HOUGHP__LIB__UTILS_HPP_

#include <opencv2/imgproc.hpp>
#include <vector>

namespace vortex::line_detection {

/**
 * @brief Create a BGR visualization by overlaying detected lines on the input
 * color image.
 * @param input_bgr Input color image (typically CV_8UC3). If not already in
 * BGR format, it should be converted to BGR8 using @c to_bgr8 before
 * calling this function.
 * @param gray8 Grayscale version of the input image (CV_8UC1). This is used as
 * an intermediate step to ensure proper handling of different input formats and
 * depths.
 * @param lines Detected line segments, where each line is represented as a
 * cv::Vec4i (x1, y1, x2, y2).
 * @return A BGR image with line segments overlaid on the input image. This is
 * typically CV_8UC3.
 */
cv::Mat make_overlay_color(const cv::Mat& input_bgr,
                           const cv::Mat& gray8,
                           const std::vector<cv::Vec4i>& lines);

/**
 * @brief Create a BGR visualization by overlaying detected lines on the Canny
 * edge map.
 * @param edges Canny edge map (CV_8UC1).
 * @param lines Detected line segments, where each line is represented as a
 * cv::Vec4i (x1, y1, x2, y2).
 * @return A BGR image with line segments overlaid on the edge map. This is
 * typically CV_8UC3.
 */
cv::Mat make_overlay_canny(const cv::Mat& edges,
                           const std::vector<cv::Vec4i>& lines);

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_HOUGHP__LIB__UTILS_HPP_
