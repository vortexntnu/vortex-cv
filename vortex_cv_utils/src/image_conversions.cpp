#include "vortex/cv_utils/image_conversions.hpp"

#include <opencv2/imgproc.hpp>

namespace vortex::cv_utils {

namespace {

cv::Mat to_8u(const cv::Mat& img) {
    if (img.depth() == CV_8U) {
        return img;
    }
    if (img.depth() == CV_16U) {
        cv::Mat out;
        img.convertTo(out, CV_8U, 1.0 / 256.0);
        return out;
    }
    cv::Mat out;
    cv::normalize(img, out, 0, 255, cv::NORM_MINMAX, CV_8U);
    return out;
}

}  // namespace

cv::Mat to_gray8(const cv::Mat& input) {
    if (input.empty()) {
        throw std::runtime_error("to_gray8: input is empty");
    }

    const int ch = input.channels();
    cv::Mat gray;

    if (ch == 1) {
        gray = input;
    } else if (ch == 3) {
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    } else if (ch == 4) {
        cv::cvtColor(input, gray, cv::COLOR_BGRA2GRAY);
    } else {
        throw std::runtime_error("to_gray8: unsupported channel count");
    }

    return to_8u(gray);
}

cv::Mat to_bgr8(const cv::Mat& input) {
    if (input.empty()) {
        throw std::runtime_error("to_bgr8: input is empty");
    }

    const int ch = input.channels();

    if (ch == 1) {
        cv::Mat gray = to_gray8(input);
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    cv::Mat bgr;
    if (ch == 3) {
        bgr = input;
    } else if (ch == 4) {
        cv::cvtColor(input, bgr, cv::COLOR_BGRA2BGR);
    } else {
        throw std::runtime_error("to_bgr8: unsupported channel count");
    }

    return to_8u(bgr);
}

cv::Mat to_bgr8(const cv::Mat& input, const cv::Mat& gray8) {
    if (input.empty()) {
        throw std::runtime_error("to_bgr8: input is empty");
    }

    const int ch = input.channels();

    if (ch == 1) {
        cv::Mat gray = gray8.empty() ? to_gray8(input) : gray8;
        if (gray.type() != CV_8UC1 || gray.size() != input.size()) {
            throw std::runtime_error(
                "to_bgr8: gray8 must match input size and be CV_8UC1");
        }
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    cv::Mat bgr;
    if (ch == 3) {
        bgr = input;
    } else if (ch == 4) {
        cv::cvtColor(input, bgr, cv::COLOR_BGRA2BGR);
    } else {
        throw std::runtime_error("to_bgr8: unsupported channel count");
    }

    return to_8u(bgr);
}

}  // namespace vortex::cv_utils
