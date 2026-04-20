#ifndef VALVE_DETECTION__POSE_ESTIMATOR_HPP_
#define VALVE_DETECTION__POSE_ESTIMATOR_HPP_

#include "valve_detection/pcl_extraction.hpp"
#include "valve_detection/types.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <cmath>
#include <opencv2/core.hpp>
#include <variant>

namespace valve_detection {

enum class DetectorMode { standard, debug };

struct DebugOutput {
    pcl::PointCloud<pcl::PointXYZ>::Ptr annulus_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud;
};

struct DetectionResult {
    Pose pose;
    bool valid{false};
    // Populated when DetectorMode::debug is used:
    pcl::PointCloud<pcl::PointXYZ>::Ptr annulus_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud;
};

class PoseEstimator {
   public:
    PoseEstimator(int yolo_img_width,
                  int yolo_img_height,
                  float annulus_radius_ratio,
                  float plane_ransac_threshold,
                  int plane_ransac_max_iterations,
                  float valve_handle_offset);

    void set_color_image_properties(const ImageProperties& props);
    void set_depth_image_properties(const ImageProperties& props);
    void set_depth_color_extrinsic(const DepthColorExtrinsic& extr);
    void set_clamp_rotation(bool clamp) { clamp_rotation_ = clamp; }

    /// @brief Computes letterbox scale and padding from color image dimensions
    /// and YOLO input size. Must be called after set_color_image_properties().
    void compute_letterbox_transform();

    /// @brief Remaps a bounding box from YOLO letterbox coordinates to original
    /// image coordinates.
    BoundingBox letterbox_to_image_coords(const BoundingBox& bbox) const;

    DetectionResult compute_pose_from_depth(
        const cv::Mat& depth_image,
        const BoundingBox& valve_bbox,
        float handle_angle_rad,
        DetectorMode mode = DetectorMode::standard) const;

   private:
    bool segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       pcl::ModelCoefficients::Ptr& coeff,
                       pcl::PointIndices::Ptr& inliers) const;

    Eigen::Vector3f compute_plane_normal(
        const pcl::ModelCoefficients::Ptr& coeff,
        const Eigen::Vector3f& ray_direction) const;
    Eigen::Vector3f find_ray_plane_intersection(
        const pcl::ModelCoefficients::Ptr& coeff,
        const Eigen::Vector3f& ray_direction,
        const Eigen::Vector3f& ray_origin = Eigen::Vector3f::Zero()) const;
    Eigen::Vector3f shift_point_along_normal(
        const Eigen::Vector3f& intersection_point,
        const Eigen::Vector3f& plane_normal) const;
    // Variant that works entirely in depth frame: color rays are rotated by
    // R_depth_from_color = R^T before intersecting the depth-frame plane.
    Eigen::Matrix3f create_rotation_matrix_depth(
        const pcl::ModelCoefficients::Ptr& coefficients,
        const Eigen::Vector3f& plane_normal,
        float angle,
        const Eigen::Vector3f& ray_origin,
        const Eigen::Matrix3f& R_depth_from_color) const;

    ImageProperties color_image_properties_{};
    ImageProperties depth_image_properties_{};
    DepthColorExtrinsic depth_color_extrinsic_{};

    int yolo_img_width_;
    int yolo_img_height_;
    float annulus_radius_ratio_;
    float plane_ransac_threshold_;
    int plane_ransac_max_iterations_;
    float valve_handle_offset_;

    double letterbox_scale_factor_{1.0};
    double letterbox_pad_x_{0};
    double letterbox_pad_y_{0};

    mutable Eigen::Vector3f filter_direction_{Eigen::Vector3f::Zero()};
    bool clamp_rotation_{false};
};

}  // namespace valve_detection

#endif  // VALVE_DETECTION__POSE_ESTIMATOR_HPP_
