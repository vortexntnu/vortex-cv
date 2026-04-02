#ifndef VALVE_DETECTION__DEPTH_IMAGE_PROCESSING_HPP_
#define VALVE_DETECTION__DEPTH_IMAGE_PROCESSING_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/calib3d.hpp>
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

/// @brief Undistorts a bounding box by undistorting the edge midpoints and
/// center, then refitting an oriented bounding box.
/// TODO: Had problems with this, when it undistorted the orientation would be off by a constant offset
/// Workaround is to just disable this and undistort the whole image instead (done upstream in perception_setup)
BoundingBox undistort_bbox(const BoundingBox& bbox,
                           const CameraIntrinsics& intr);

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
