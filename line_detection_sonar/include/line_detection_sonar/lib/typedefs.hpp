#ifndef LINE_DETECTION_SONAR__LIB__TYPEDEFS_HPP_
#define LINE_DETECTION_SONAR__LIB__TYPEDEFS_HPP_

#include <opencv2/core.hpp>
#include <variant>
#include <vortex/utils/types.hpp>

namespace vortex::line_detection {

/**
 * @brief Config for the OpenCV Canny edge algorithm
 * used to detect edges in an 8-bit single channel image.
 */
struct EdgeDetectionConfig {
    int low_threshold{50};    // Low threshold for the hysteresis procedure.
    int high_threshold{150};  // High threshold for the hysteresis procedure.
    int aperture_size{3};     // Aperture size for the Sobel operator.
    bool L2_gradient{false};  // L2_gradient flag, indicating whether to use a
                              // more accurate edge equation algorithm.
};

/**
 * @brief Config for the OpenCV HoughLinesP algorithm
 * used to detect lines segments in an 8-bit single channel image.
 */
struct LineDetectionConfig {
    double rho{1.0};  // Distance resolution of the accumulator in pixels.
    double theta{CV_PI / 180.0};  // Theta angle resolution
                                  // of the accumulator in radians.
    int threshold{50};  // Accumulator threshold used for accepting lines.
    double min_line_length{0.0};  // Minimum allowed length for returned lines.
    double max_line_gap{0.0};     // Allowed gap between points
                                  // on the same line to link them.
};

/**
 * @brief Operating mode of the detector.
 *
 * - standard: compute and return line segments only.
 * - visualize: return line segments +
 *   an overlay image on the input color image.
 * - debug: return line segments + edge map +
 *   overlay color and edge image for debugging.
 */
enum class Mode { standard, visualize, debug };

/**
 * @brief Empty output payload for Mode::standard.
 *
 * Used as the "no extra outputs" alternative in the Output variant.
 */
struct NoOutput {};

/**
 * @brief Visualization payload for Mode::visualize.
 *
 * - overlay_color: a BGR visualization derived from the input color image with
 * drawings applied.
 */
struct VisualizeOutput {
    cv::Mat overlay_color;  // Input image with line segments overlaid
                            // (typically CV_8UC3).
};

/**
 * @brief Debug payload for Mode::debug.
 *
 * - canny: the edge map produced by Canny (8-bit, single-channel).
 * - overlay_canny: a BGR visualization derived from the edge map with drawings
 * applied.
 * - overlay_color: a BGR visualization derived from the input color image with
 * drawings applied.
 */
struct DebugOutput {
    cv::Mat canny;          // Canny edge map (CV_8UC1).
    cv::Mat overlay_canny;  // Edge map with line segments overlaid (typically
                            // CV_8UC3).
    cv::Mat overlay_color;  // Input image with line segments overlaid
                            // (typically CV_8UC3).
};

/**
 * @brief Variant holding the mode-dependent output payload.
 *
 * Exactly one alternative is active:
 * - NoOutput for Mode::standard
 * - VisualizeOutput for Mode::visualize
 * - DebugOutput for Mode::debug
 */
using Output = std::variant<NoOutput, VisualizeOutput, DebugOutput>;

/**
 * @brief Result returned from a detection call.
 *
 * line_segments is always populated. output contains additional products
 * depending on the requested Mode.
 */
struct Result {
    std::vector<vortex::utils::types::LineSegment2D>
        line_segments;  // Detected line segments.
    Output output;      // Mode-dependent extra outputs.
};

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_SONAR__LIB__TYPEDEFS_HPP_
