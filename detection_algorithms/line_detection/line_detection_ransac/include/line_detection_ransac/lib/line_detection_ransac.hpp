#ifndef LINE_DETECTION_RANSAC__LIB__LINE_DETECTION_RANSAC_HPP_
#define LINE_DETECTION_RANSAC__LIB__LINE_DETECTION_RANSAC_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <vortex/utils/types.hpp>
#include "typedefs.hpp"

namespace vortex::line_detection {

class LineDetectorRansac {
   public:
    explicit LineDetectorRansac(const BoundaryConfig& boundary_detection_config,
                                const RansacConfig& ransac_config);

    Result detect(const cv::Mat& input_image,
                  DetectorMode mode = DetectorMode::standard) const;

   private:
    BoundaryConfig boundary_config_;
    RansacConfig ransac_config_;

    static std::vector<vortex::utils::types::LineSegment2D> to_line_segments(
        const std::vector<cv::Vec4i>& cv_lines);
};

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_RANSAC__LIB__LINE_DETECTION_RANSAC_HPP_
