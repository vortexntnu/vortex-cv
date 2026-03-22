#ifndef LINE_DETECTION_RANSAC__LIB__DETECTION_UTILS_HPP_
#define LINE_DETECTION_RANSAC__LIB__DETECTION_UTILS_HPP_

#include <vector>
#include "line_detection_ransac/lib/line_detection_ransac.hpp"
#include "line_detection_ransac/lib/typedefs.hpp"

namespace vortex::line_detection {

void detect_lines(const RansacConfig& ransac_config,
                  const std::vector<cv::Point>& boundary_points,
                  std::vector<cv::Vec4i>& cv_lines);

void detect_boundaries(const BoundaryConfig& boundary_config,
                       const cv::Mat& input_image,
                       std::vector<cv::Point>& boundary_points);

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_RANSAC__LIB__DETECTION_UTILS_HPP_
