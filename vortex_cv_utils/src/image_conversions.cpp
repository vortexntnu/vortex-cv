#include "vortex/cv_utils/image_conversions.hpp"

#include <opencv2/imgproc.hpp>

namespace vortex::cv_utils {

cv::Mat to_gray8(const cv::Mat& input) {
    if (input.empty()) {
        throw std::runtime_error("to_gray8: input is empty");
    }

    if (input.type() == CV_8UC1) {
        return input;
    }

    cv::Mat gray;
    const int ch = input.channels();

    if (ch == 1) {
        gray = input;
    } else if (ch == 3) {
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    } else if (ch == 4) {
        cv::cvtColor(input, gray, cv::COLOR_BGRA2GRAY);
    } else {
        throw std::runtime_error("to_gray8: unsupported channel count");
    }

    if (gray.depth() == CV_8U) {
        return gray;
    }

    if (gray.depth() == CV_16U) {
        cv::Mat out;
        gray.convertTo(out, CV_8U, 1.0 / 256.0);
        return out;
    }

    cv::Mat out;
    cv::normalize(gray, out, 0, 255, cv::NORM_MINMAX, CV_8U);
    return out;
}

cv::Mat to_bgr8(const cv::Mat& input, const cv::Mat& gray8) {
    if (input.empty()) {
        throw std::runtime_error("to_bgr8: input is empty");
    }

    if (input.type() == CV_8UC3) {
        return input;
    }

    const int ch = input.channels();

    if (ch == 1) {
        if (gray8.empty() || gray8.type() != CV_8UC1) {
            throw std::runtime_error("to_bgr8: gray8 must be CV_8UC1");
        }
        cv::Mat bgr;
        cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    if (ch == 4) {
        cv::Mat bgr;
        cv::cvtColor(input, bgr, cv::COLOR_BGRA2BGR);

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

    throw std::runtime_error("to_bgr8: unsupported channel count");
}

}  // namespace vortex::cv_utils
