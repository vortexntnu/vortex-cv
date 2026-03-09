#ifndef LINE_DETECTION_RANSAC__LIB__TYPEDEFS_HPP_
#define LINE_DETECTION_RANSAC__LIB__TYPEDEFS_HPP_

#include <opencv2/core.hpp>
#include <variant>
#include <vector>
#include <vortex/utils/types.hpp>

namespace vortex::line_detection {

/**
 * @brief Config for the boundary detection algorithm
 * used to detect edges using rays from the sonar origin
 */
struct BoundaryConfig {
    int threshold{100};  // Threshold for the boundary detection along the ray.
    float step{2.0f};    // the length of the steps moving along a ray.
    int rays{1};         // the number of rays to cast for boundary detection.
    int sample_size{3};  // the side length of the square sample taken at each
                         // step (must be odd).
    int angle{150};  // the total angle range of the sonar image to search for
                     // boundaries (degrees).
};

struct RansacConfig {
    int points_checked{4};           // Number of points to check for lines
    float distance_threshold{2.0f};  // Maximum distance from line for inliers
    int min_remaining_points{10};    // Minimum number of points remaining to
                                     // continue RANSAC iterations
    int min_inliers{5};  // Minimum number of inliers to accept a line
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
enum class DetectorMode { standard, visualize, debug };

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
    cv::Mat boundaries;          // detected points along the border.
    cv::Mat overlay_boundaries;  // Points along the border with line segments
                                 // overlaid (typically CV_8UC3).
    cv::Mat overlay_color;       // Input image with line segments overlaid
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

#endif  // LINE_DETECTION_RANSAC__LIB__TYPEDEFS_HPP_
