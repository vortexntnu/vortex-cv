#ifndef LINE_DETECTION_HOUGHP__LIB__LINE_DETECTION_HOUGHP_HPP_
#define LINE_DETECTION_HOUGHP__LIB__LINE_DETECTION_HOUGHP_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <vortex/utils/types.hpp>
#include "typedefs.hpp"

namespace vortex::line_detection {

class LineDetectorHoughP {
 public:
  explicit LineDetectorHoughP(const CannyConfig& edge_config,
                              const HoughPConfig& line_config);

  Result detect(const cv::Mat& input_image,
                DetectorMode mode = DetectorMode::standard) const;

 private:
  CannyConfig canny_config_;
  HoughPConfig houghp_config_;

  void detect_edges(const cv::Mat& input_image, cv::Mat& edge_image) const;

  void detect_line_segments(const cv::Mat& edge_image,
                            std::vector<cv::Vec4i>& cv_lines) const;

  static std::vector<vortex::utils::types::LineSegment2D> to_line_segments(
      const std::vector<cv::Vec4i>& cv_lines);
};

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_HOUGHP__LIB__LINE_DETECTION_HOUGHP_HPP_
