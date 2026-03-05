#ifndef LINE_DETECTION_RANSAC__LIB__UTILS_HPP_
#define LINE_DETECTION_RANSAC__LIB__UTILS_HPP_

#include <opencv2/imgproc.hpp>
#include <vector>

namespace vortex::line_detection {

/**
 * @brief Convert an image to 8-bit, single-channel grayscale (CV_8UC1).
 *
 * This helper is intended to produce a suitable input for edge detection (e.g.,
 * Canny) from a variety of common OpenCV image formats.
 *
 * Conversion rules:
 * - If @p input is already CV_8UC1, the function returns a **shallow copy** (no
 * pixel data copied).
 * - If @p input has 3 channels, it is converted using @c cv::COLOR_BGR2GRAY
 * (assumes BGR ordering).
 * - If @p input has 4 channels, it is converted using @c cv::COLOR_BGRA2GRAY
 * (assumes BGRA ordering).
 *
 * - If the resulting grayscale image is 16-bit unsigned (CV_16U), it is scaled
 * to 8-bit using
 *   @c convertTo with @c alpha = 1/256 (i.e., approximately a right shift by 8
 * bits).
 * - For other depths (signed integers or floating point), the image is min-max
 * normalized to [0,255] using @c cv::normalize(..., cv::NORM_MINMAX) and
 * written as CV_8U.
 *
 * @param input Input image. Supported channel counts: 1, 3 (BGR), or 4 (BGRA).
 *              Supported depths: any OpenCV depth; non-8U depths are converted
 * as described above.
 *
 * @return A grayscale image of type CV_8UC1. May share data with @p input if no
 * conversion is needed.
 *
 * @throws std::runtime_error If @p input is empty or has an unsupported channel
 * count.
 */
cv::Mat to_gray8(const cv::Mat& input);

/**
 * @brief Convert an image to 8-bit, 3-channel BGR (CV_8UC3).
 *
 * Conversion rules:
 * - If @p input is already CV_8UC3, the function returns a **shallow copy** (no
 * pixel data copied).
 * - If @p input is single-channel, it is expanded to BGR using @c
 * cv::COLOR_GRAY2BGR. In this case, the already-computed @p gray8 (CV_8UC1)
 * must be provided to avoid recomputing grayscale conversion / scaling.
 * - If @p input has 4 channels, it is converted using @c cv::COLOR_BGRA2BGR
 * (assumes BGRA ordering).
 * - If the resulting BGR image is 16-bit unsigned (CV_16U), it is scaled to
 * 8-bit using
 *   @c convertTo with @c alpha = 1/256 (i.e., approximately a right shift by 8
 * bits).
 * - For other depths (signed integers or floating point), the image is min-max
 * normalized to [0,255] using @c cv::normalize(..., cv::NORM_MINMAX) and
 * written as CV_8U.
 *
 * @note The normalization step computes min/max per call. This is usually fine
 * for visualization, but it may produce different brightness/contrast per frame
 * when viewing a stream.
 *
 * @param input Input image. Supported channel counts: 1, 3 (assumed BGR), or 4
 * (assumed BGRA).
 * @param gray8 A grayscale image of type CV_8UC1 corresponding to @p input (or
 * derived from it). Required when @p input is single-channel; ignored for 3- or
 * 4-channel inputs.
 *
 * @return A BGR image of type CV_8UC3. May share data with @p input if no
 * conversion is needed.
 *
 * @throws std::runtime_error If @p input is empty, has an unsupported channel
 * count, or if @p input is single-channel and @p gray8 is empty or not CV_8UC1.
 */
cv::Mat to_bgr8(const cv::Mat& input, const cv::Mat& gray8 /*CV_8UC1*/);

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
cv::Mat make_overlay_boundaries(const cv::Mat& edges,
                                const std::vector<cv::Vec4i>& lines);

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_RANSAC__LIB__UTILS_HPP_
