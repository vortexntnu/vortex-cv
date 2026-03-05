#ifndef LINE_DETECTION_HOUGHP__LIB__UTILS_HPP_
#define LINE_DETECTION_HOUGHP__LIB__UTILS_HPP_

#include "line_detection_ransac/lib/utils.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>

namespace vortex::line_detection {

cv::Mat to_gray8(const cv::Mat& input) {
  if (input.empty()) {
    throw std::runtime_error("to_gray8: input is empty");
  }

  if (input.type() == CV_8UC1) {
    return input;
  }

  cv::Mat gray;
  const int ch = input.channels();

  // Convert to grayscale
  if (ch == 1) {
    gray = input;
  } else if (ch == 3) {
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);  // assumes BGR
  } else if (ch == 4) {
    cv::cvtColor(input, gray, cv::COLOR_BGRA2GRAY);  // assumes BGRA
  } else {
    throw std::runtime_error("to_gray8: unsupported channel count");
  }

  // Now ensure 8-bit depth.
  if (gray.depth() == CV_8U) {
    return gray;
  }

  if (gray.depth() == CV_16U) {
    cv::Mat out;
    gray.convertTo(out, CV_8U, 1.0 / 256.0);  // quick/common 16U->8U
    return out;
  }

  // Generic fallback for signed ints / floats etc.
  cv::Mat out;
  cv::normalize(gray, out, 0, 255, cv::NORM_MINMAX, CV_8U);
  return out;
}

cv::Mat to_bgr8(const cv::Mat& input, const cv::Mat& gray8 /*CV_8UC1*/) {
  if (input.empty()) {
    throw std::runtime_error("to_bgr8_for_overlay: input is empty");
  }

  if (input.type() == CV_8UC3) {
    return input;
  }

  const int ch = input.channels();

  if (ch == 1) {
    if (gray8.empty() || gray8.type() != CV_8UC1) {
      throw std::runtime_error("to_bgr8_for_overlay: gray8 must be CV_8UC1");
    }
    cv::Mat bgr;
    cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }

  if (ch == 4) {
    cv::Mat bgr;
    cv::cvtColor(input, bgr, cv::COLOR_BGRA2BGR);  // assumes BGRA

    if (bgr.depth() == CV_8U) {
      return bgr;
    }
    if (bgr.depth() == CV_16U) {
      cv::Mat out;
      bgr.convertTo(out, CV_8U, 1.0 / 256.0);
      return out;
    }
    cv::Mat out;
    cv::normalize(bgr, out, 0, 255, cv::NORM_MINMAX, CV_8U);
    return out;
  }

  if (ch == 3) {
    if (input.depth() == CV_8U) {
      return input;
    }
    if (input.depth() == CV_16U) {
      cv::Mat out;
      input.convertTo(out, CV_8U, 1.0 / 256.0);
      return out;
    }
    cv::Mat out;
    cv::normalize(input, out, 0, 255, cv::NORM_MINMAX, CV_8U);
    return out;
  }

  throw std::runtime_error("to_bgr8_for_overlay: unsupported channel count");
}

constexpr int k_line_thickness = 2;
constexpr int k_line_type = cv::LINE_AA;
const cv::Scalar k_line_color(0, 0, 255);  // BGR red

cv::Mat make_overlay_color(const cv::Mat& input_bgr,
                           const cv::Mat& gray8,
                           const std::vector<cv::Vec4i>& lines) {
  if (input_bgr.empty()) {
    throw std::runtime_error("make_overlay_color: input_bgr is empty");
  }
  cv::Mat out_bgr = vortex::line_detection::to_bgr8(input_bgr, gray8);
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

#endif  // LINE_DETECTION_HOUGHP__LIB__UTILS_HPP_
