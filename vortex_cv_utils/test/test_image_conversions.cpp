#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <vortex/cv_utils/image_conversions.hpp>

namespace vortex::cv_utils {

// ---------------------------------------------------------------------------
// to_gray8
// ---------------------------------------------------------------------------

TEST(ToGray8, PassthroughForCV8UC1) {
    cv::Mat input(10, 10, CV_8UC1, cv::Scalar(128));
    cv::Mat result = to_gray8(input);

    EXPECT_EQ(result.type(), CV_8UC1);
    EXPECT_EQ(result.data, input.data);  // shallow copy
}

TEST(ToGray8, ConvertsBGRToGray) {
    cv::Mat input(10, 10, CV_8UC3, cv::Scalar(100, 150, 200));
    cv::Mat result = to_gray8(input);

    EXPECT_EQ(result.type(), CV_8UC1);
    EXPECT_EQ(result.size(), input.size());
}

TEST(ToGray8, ConvertsBGRAToGray) {
    cv::Mat input(10, 10, CV_8UC4, cv::Scalar(100, 150, 200, 255));
    cv::Mat result = to_gray8(input);

    EXPECT_EQ(result.type(), CV_8UC1);
    EXPECT_EQ(result.size(), input.size());
}

TEST(ToGray8, Converts16UToU8) {
    cv::Mat input(10, 10, CV_16UC1, cv::Scalar(256 * 128));
    cv::Mat result = to_gray8(input);

    EXPECT_EQ(result.type(), CV_8UC1);
    EXPECT_EQ(result.at<uint8_t>(0, 0), 128);
}

TEST(ToGray8, NormalizesFloatDepth) {
    cv::Mat input(10, 10, CV_32FC1, cv::Scalar(0.5));
    input.at<float>(0, 0) = 0.0f;
    input.at<float>(0, 1) = 1.0f;

    cv::Mat result = to_gray8(input);

    EXPECT_EQ(result.type(), CV_8UC1);
    EXPECT_EQ(result.at<uint8_t>(0, 0), 0);
    EXPECT_EQ(result.at<uint8_t>(0, 1), 255);
}

TEST(ToGray8, ThrowsOnEmpty) {
    cv::Mat empty;
    EXPECT_THROW(to_gray8(empty), std::runtime_error);
}

TEST(ToGray8, ThrowsOnUnsupportedChannels) {
    cv::Mat input(10, 10, CV_8UC(5), cv::Scalar::all(0));
    EXPECT_THROW(to_gray8(input), std::runtime_error);
}

// ---------------------------------------------------------------------------
// to_bgr8 (1-arg overload)
// ---------------------------------------------------------------------------

TEST(ToBgr8OneArg, PassthroughForCV8UC3) {
    cv::Mat input(10, 10, CV_8UC3, cv::Scalar(50, 100, 150));
    cv::Mat result = to_bgr8(input);

    EXPECT_EQ(result.type(), CV_8UC3);
    EXPECT_EQ(result.data, input.data);  // shallow copy
}

TEST(ToBgr8OneArg, ConvertsSingleChannelToBGR) {
    cv::Mat input(10, 10, CV_8UC1, cv::Scalar(128));
    cv::Mat result = to_bgr8(input);

    EXPECT_EQ(result.type(), CV_8UC3);
    EXPECT_EQ(result.size(), input.size());

    // Gray→BGR means all channels equal
    cv::Vec3b pixel = result.at<cv::Vec3b>(0, 0);
    EXPECT_EQ(pixel[0], 128);
    EXPECT_EQ(pixel[1], 128);
    EXPECT_EQ(pixel[2], 128);
}

TEST(ToBgr8OneArg, ConvertsBGRAToBGR) {
    cv::Mat input(10, 10, CV_8UC4, cv::Scalar(50, 100, 150, 255));
    cv::Mat result = to_bgr8(input);

    EXPECT_EQ(result.type(), CV_8UC3);
    cv::Vec3b pixel = result.at<cv::Vec3b>(0, 0);
    EXPECT_EQ(pixel[0], 50);
    EXPECT_EQ(pixel[1], 100);
    EXPECT_EQ(pixel[2], 150);
}

TEST(ToBgr8OneArg, Converts16UC3ToU8) {
    cv::Mat input(10, 10, CV_16UC3, cv::Scalar(256 * 50, 256 * 100, 256 * 150));
    cv::Mat result = to_bgr8(input);

    EXPECT_EQ(result.type(), CV_8UC3);
    cv::Vec3b pixel = result.at<cv::Vec3b>(0, 0);
    EXPECT_EQ(pixel[0], 50);
    EXPECT_EQ(pixel[1], 100);
    EXPECT_EQ(pixel[2], 150);
}

TEST(ToBgr8OneArg, ThrowsOnEmpty) {
    cv::Mat empty;
    EXPECT_THROW(to_bgr8(empty), std::runtime_error);
}

TEST(ToBgr8OneArg, ThrowsOnUnsupportedChannels) {
    cv::Mat input(10, 10, CV_8UC(5), cv::Scalar::all(0));
    EXPECT_THROW(to_bgr8(input), std::runtime_error);
}

// ---------------------------------------------------------------------------
// to_bgr8 (2-arg overload)
// ---------------------------------------------------------------------------

TEST(ToBgr8TwoArg, UsesProvidedGray8) {
    cv::Mat input(10, 10, CV_16UC1, cv::Scalar(256 * 200));
    cv::Mat gray8(10, 10, CV_8UC1, cv::Scalar(42));

    cv::Mat result = to_bgr8(input, gray8);

    EXPECT_EQ(result.type(), CV_8UC3);
    // Should use the provided gray8 (value 42), not convert from input
    cv::Vec3b pixel = result.at<cv::Vec3b>(0, 0);
    EXPECT_EQ(pixel[0], 42);
    EXPECT_EQ(pixel[1], 42);
    EXPECT_EQ(pixel[2], 42);
}

TEST(ToBgr8TwoArg, FallsBackWhenGray8Empty) {
    cv::Mat input(10, 10, CV_8UC1, cv::Scalar(128));
    cv::Mat empty_gray;

    cv::Mat result = to_bgr8(input, empty_gray);

    EXPECT_EQ(result.type(), CV_8UC3);
    cv::Vec3b pixel = result.at<cv::Vec3b>(0, 0);
    EXPECT_EQ(pixel[0], 128);
    EXPECT_EQ(pixel[1], 128);
    EXPECT_EQ(pixel[2], 128);
}

TEST(ToBgr8TwoArg, ThrowsOnSizeMismatch) {
    cv::Mat input(10, 10, CV_8UC1, cv::Scalar(128));
    cv::Mat gray8(5, 5, CV_8UC1, cv::Scalar(128));

    EXPECT_THROW(to_bgr8(input, gray8), std::runtime_error);
}

TEST(ToBgr8TwoArg, ThrowsOnWrongGray8Type) {
    cv::Mat input(10, 10, CV_8UC1, cv::Scalar(128));
    cv::Mat gray8(10, 10, CV_8UC3, cv::Scalar(128, 128, 128));

    EXPECT_THROW(to_bgr8(input, gray8), std::runtime_error);
}

TEST(ToBgr8TwoArg, IgnoresGray8ForMultiChannel) {
    cv::Mat input(10, 10, CV_8UC3, cv::Scalar(50, 100, 150));
    cv::Mat gray8(10, 10, CV_8UC1, cv::Scalar(42));

    cv::Mat result = to_bgr8(input, gray8);

    EXPECT_EQ(result.type(), CV_8UC3);
    // gray8 should be ignored for 3-channel input
    cv::Vec3b pixel = result.at<cv::Vec3b>(0, 0);
    EXPECT_EQ(pixel[0], 50);
    EXPECT_EQ(pixel[1], 100);
    EXPECT_EQ(pixel[2], 150);
}

}  // namespace vortex::cv_utils
