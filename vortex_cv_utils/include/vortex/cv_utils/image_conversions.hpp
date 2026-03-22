#ifndef VORTEX__CV_UTILS__IMAGE_CONVERSIONS_HPP_
#define VORTEX__CV_UTILS__IMAGE_CONVERSIONS_HPP_

#include <opencv2/imgproc.hpp>

namespace vortex::cv_utils {

/**
 * @brief Convert an image to 8-bit, single-channel grayscale (CV_8UC1).
 *
 * Conversion rules:
 * - If @p input is already CV_8UC1, returns a shallow copy.
 * - 3-channel input is converted using cv::COLOR_BGR2GRAY.
 * - 4-channel input is converted using cv::COLOR_BGRA2GRAY.
 * - 16U depth is scaled to 8U with alpha = 1/256.
 * - Other depths are min-max normalized to [0, 255].
 *
 * @param input Input image (1, 3, or 4 channels).
 * @return CV_8UC1 grayscale image.
 * @throws std::runtime_error If input is empty or has unsupported channel
 * count.
 */
cv::Mat to_gray8(const cv::Mat& input);

/**
 * @brief Convert an image to 8-bit, 3-channel BGR (CV_8UC3).
 *
 * Conversion rules:
 * - If @p input is already CV_8UC3, returns a shallow copy.
 * - Single-channel input is expanded using cv::COLOR_GRAY2BGR from @p gray8.
 * - 4-channel input is converted using cv::COLOR_BGRA2BGR.
 * - 16U depth is scaled to 8U with alpha = 1/256.
 * - Other depths are min-max normalized to [0, 255].
 *
 * @param input Input image (1, 3, or 4 channels).
 * @param gray8 Pre-computed CV_8UC1 grayscale. Required when input is
 * single-channel.
 * @return CV_8UC3 BGR image.
 * @throws std::runtime_error If input is empty, has unsupported channel count,
 * or gray8 is invalid when needed.
 */
cv::Mat to_bgr8(const cv::Mat& input, const cv::Mat& gray8);

}  // namespace vortex::cv_utils

#endif  // VORTEX__CV_UTILS__IMAGE_CONVERSIONS_HPP_
