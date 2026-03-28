#ifndef VALVE_DETECTION__DEPTH_IMAGE_PROCESSING_HPP_
#define VALVE_DETECTION__DEPTH_IMAGE_PROCESSING_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include "valve_detection/types.hpp"

namespace valve_detection {

/**
 * @brief Back-projects a depth pixel to a 3D point in the camera frame.
 *
 * Assumes no lens distortion — this is valid for depth images, which are
 * typically rectified before publication by the camera driver.
 */
void project_pixel_to_point(int u,
                            int v,
                            float depth,
                            double fx,
                            double fy,
                            double cx,
                            double cy,
                            pcl::PointXYZ& out);

// Projects a color image pixel to depth image coordinates.
// u_c, v_c: pixel coordinates in the color image.
// Z:        depth of the point in the color camera frame (metres).
cv::Point2f project_color_pixel_to_depth(float u_c,
                                         float v_c,
                                         float Z,
                                         const ImageProperties& color_props,
                                         const ImageProperties& depth_props,
                                         const DepthColorExtrinsic& extr);

}  // namespace valve_detection

#endif  // VALVE_DETECTION__DEPTH_IMAGE_PROCESSING_HPP_
