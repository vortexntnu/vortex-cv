#ifndef VALVE_DETECTION__PCL_EXTRACTION_HPP_
#define VALVE_DETECTION__PCL_EXTRACTION_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include "valve_detection/types.hpp"

namespace valve_detection {

// Iterates depth pixels, back-projects each with depth intrinsics, applies
// the extrinsic transform, then checks whether the resulting color-frame
// projection falls inside the elliptic annulus defined by color_bbox.
// Output points are in the color camera frame.
void extract_annulus_pcl_aligned(
    const cv::Mat& depth_image,     // CV_32FC1 metres, depth frame
    const BoundingBox& color_bbox,  // annulus defined in color pixels
    const ImageProperties& color_props,
    const ImageProperties& depth_props,
    const DepthColorExtrinsic& extrinsic,
    float annulus_radius_ratio,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

// Extracts all valid depth points whose color-frame projection falls inside
// the oriented bounding box. Output points are in the color camera frame.
void extract_bbox_pcl_aligned(
    const cv::Mat& depth_image,     // CV_32FC1 metres, depth frame
    const BoundingBox& color_bbox,  // OBB defined in color pixels
    const ImageProperties& color_props,
    const ImageProperties& depth_props,
    const DepthColorExtrinsic& extrinsic,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

// Projects the 4 corners of the color OBB into depth image space once, fits
// an OBB to those projected corners, then tests depth pixels directly against
// that depth-image OBB — no per-pixel matrix multiply needed. Output points
// are in the depth camera frame.
void extract_bbox_pcl_depth(
    const cv::Mat& depth_image,     // CV_32FC1 metres, depth frame
    const BoundingBox& color_bbox,  // OBB defined in color pixels
    const ImageProperties& color_props,
    const ImageProperties& depth_props,
    const DepthColorExtrinsic& extrinsic,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

}  // namespace valve_detection

#endif  // VALVE_DETECTION__PCL_EXTRACTION_HPP_
