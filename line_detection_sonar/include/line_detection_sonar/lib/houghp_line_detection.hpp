#ifndef LINE_DETECTION_SONAR__LIB__HOUGHP_LINE_DETECTION_HPP_
#define LINE_DETECTION_SONAR__LIB__HOUGHP_LINE_DETECTION_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vortex/utils/types.hpp>
#include "typedefs.hpp"

namespace vortex::line_detection {

class HoughPLineDetector {
   public:
    explicit HoughPLineDetector(EdgeDetectionConfig edge_cfg,
                                LineDetectionConfig line_cfg);

    Result detect(const cv::Mat& input_image, Mode mode = Mode::standard) const;

   private:
    EdgeDetectionConfig edge_config_;
    LineDetectionConfig config_;

    void detect_edges(const cv::Mat& input_image, cv::Mat& edges) const;

    void detect_line_segments(const cv::Mat& edges,
                              std::vector<cv::Vec4i>& lines) const;

    static std::vector<vortex::utils::types::LineSegment2D> to_line_segments(
        const std::vector<cv::Vec4i>& lines);

    static void make_overlay_color(const cv::Mat& input_bgr,
                                   const cv::Mat& edges,
                                   const std::vector<cv::Vec4i>& lines,
                                   cv::Mat& out_bgr);

    static void make_overlay_canny(const cv::Mat& edges,
                                   const std::vector<cv::Vec4i>& lines,
                                   cv::Mat& out_bgr);
};

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_SONAR__LIB__HOUGHP_LINE_DETECTION_HPP_
