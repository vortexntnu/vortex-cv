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
 * @note The normalize fallback (NORM_MINMAX) rescales contrast per image and
 * is intended for visualization / debug overlays, not algorithmic processing.
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
 * - Single-channel input is converted to grayscale via @c to_gray8, then
 *   expanded to BGR.
 * - 4-channel input is converted using cv::COLOR_BGRA2BGR.
 * - 16U depth is scaled to 8U with alpha = 1/256.
 * - Other depths are min-max normalized to [0, 255].
 *
 * @note The normalize fallback (NORM_MINMAX) rescales contrast per image and
 * is intended for visualization / debug overlays, not algorithmic processing.
 *
 * @param input Input image (1, 3, or 4 channels).
 * @return CV_8UC3 BGR image.
 * @throws std::runtime_error If input is empty or has unsupported channel
 * count.
 */
cv::Mat to_bgr8(const cv::Mat& input);

/**
 * @brief Convert an image to 8-bit, 3-channel BGR (CV_8UC3), reusing a
 * pre-computed grayscale image.
 *
 * Behaves like @c to_bgr8(input), but when @p input is single-channel and
 * @p gray8 is non-empty, it uses @p gray8 directly instead of recomputing
 * the grayscale conversion. This avoids redundant work when the caller
 * already has a CV_8UC1 image (e.g. from an earlier @c to_gray8 call).
 *
 * @param input Input image (1, 3, or 4 channels).
 * @param gray8 Pre-computed CV_8UC1 grayscale. Used only when input is
 * single-channel. If empty, grayscale is computed internally.
 * @return CV_8UC3 BGR image.
 * @throws std::runtime_error If input is empty, has unsupported channel count,
 * or gray8 does not match input size / type when provided.
 */
cv::Mat to_bgr8(const cv::Mat& input, const cv::Mat& gray8);

}  // namespace vortex::cv_utils

#endif  // VORTEX__CV_UTILS__IMAGE_CONVERSIONS_HPP_
