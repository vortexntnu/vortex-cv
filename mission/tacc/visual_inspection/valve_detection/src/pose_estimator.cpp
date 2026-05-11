// pose_estimator.cpp
// RANSAC plane fitting, normal/ray-plane intersection, and 3D pose computation.
#include "valve_detection/pose_estimator.hpp"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace valve_detection {

// Initializes YOLO dimensions, annulus ratio, RANSAC parameters, and handle
// offset.
PoseEstimator::PoseEstimator(int yolo_img_width,
                             int yolo_img_height,
                             float annulus_radius_ratio,
                             float plane_ransac_threshold,
                             int plane_ransac_max_iterations,
                             float valve_handle_offset)
    : yolo_img_width_(yolo_img_width),
      yolo_img_height_(yolo_img_height),
      annulus_radius_ratio_(annulus_radius_ratio),
      plane_ransac_threshold_(plane_ransac_threshold),
      plane_ransac_max_iterations_(plane_ransac_max_iterations),
      valve_handle_offset_(valve_handle_offset) {}

// Stores color camera intrinsics and image dimensions.
void PoseEstimator::set_color_image_properties(const ImageProperties& props) {
    color_image_properties_ = props;
}

// Stores depth camera intrinsics and image dimensions.
void PoseEstimator::set_depth_image_properties(const ImageProperties& props) {
    depth_image_properties_ = props;
}

// Stores the depth-to-color extrinsic transform.
void PoseEstimator::set_depth_color_extrinsic(const DepthColorExtrinsic& extr) {
    depth_color_extrinsic_ = extr;
}

// Computes letterbox scale factor and padding from the color image and YOLO
// dimensions.
void PoseEstimator::compute_letterbox_transform() {
    int org_image_width = color_image_properties_.dim.width;
    int org_image_height = color_image_properties_.dim.height;

    letterbox_scale_factor_ =
        std::min(static_cast<double>(yolo_img_width_) / org_image_width,
                 static_cast<double>(yolo_img_height_) / org_image_height);

    double resized_width = org_image_width * letterbox_scale_factor_;
    double resized_height = org_image_height * letterbox_scale_factor_;

    letterbox_pad_x_ = (yolo_img_width_ - resized_width) / 2.0;
    letterbox_pad_y_ = (yolo_img_height_ - resized_height) / 2.0;
}

// Remaps a bounding box from YOLO letterbox space back to original image
// coordinates.
BoundingBox PoseEstimator::letterbox_to_image_coords(
    const BoundingBox& bbox) const {
    BoundingBox transformed = bbox;
    transformed.center_x =
        (bbox.center_x - letterbox_pad_x_) / letterbox_scale_factor_;
    transformed.center_y =
        (bbox.center_y - letterbox_pad_y_) / letterbox_scale_factor_;
    transformed.size_x /= letterbox_scale_factor_;
    transformed.size_y /= letterbox_scale_factor_;
    return transformed;
}

// Fits a plane to the point cloud using RANSAC; returns false if no inliers are
// found.
bool PoseEstimator::segment_plane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::ModelCoefficients::Ptr& coefficients,
    pcl::PointIndices::Ptr& inliers) const {
    if (cloud->points.empty())
        return false;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_ransac_threshold_);
    seg.setMaxIterations(plane_ransac_max_iterations_);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    return !inliers->indices.empty();
}

// Extracts and normalizes the plane normal, flipping it to face the camera.
Eigen::Vector3f PoseEstimator::compute_plane_normal(
    const pcl::ModelCoefficients::Ptr& coefficients,
    const Eigen::Vector3f& ray_direction) const {
    if (!coefficients || coefficients->values.size() < 3)
        return Eigen::Vector3f::Zero();

    Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1],
                           coefficients->values[2]);
    normal.normalize();

    if (normal.dot(ray_direction) > 0.0f)
        normal = -normal;
    return normal;
}

// Finds the 3D point where a ray intersects the fitted plane.
// ray_origin defaults to the camera origin (zero), which is correct for the
// color-frame path. Pass the actual origin when the ray does not start at
// the frame origin (e.g. the color camera origin expressed in depth frame).
Eigen::Vector3f PoseEstimator::find_ray_plane_intersection(
    const pcl::ModelCoefficients::Ptr& coefficients,
    const Eigen::Vector3f& ray_direction,
    const Eigen::Vector3f& ray_origin) const {
    // A plane ax+by+cz+d=0 has 4 coefficients: [a, b, c, d].
    if (!coefficients || coefficients->values.size() < 4)
        return Eigen::Vector3f::Zero();

    const Eigen::Vector3f plane_normal(coefficients->values[0],
                                       coefficients->values[1],
                                       coefficients->values[2]);
    const float D = coefficients->values[3];
    const float denom = plane_normal.dot(ray_direction);

    if (std::abs(denom) < 1e-6f)
        return Eigen::Vector3f::Zero();

    const float lambda = -(plane_normal.dot(ray_origin) + D) / denom;
    return ray_origin + lambda * ray_direction;
}

// Shifts a 3D point along the plane normal by the valve handle offset.
Eigen::Vector3f PoseEstimator::shift_point_along_normal(
    const Eigen::Vector3f& intersection_point,
    const Eigen::Vector3f& plane_normal) const {
    return intersection_point + (plane_normal * valve_handle_offset_);
}

// Builds a 3×3 rotation matrix from the plane normal (Z) and the projected
// bbox angle (X), working entirely in the depth camera frame.  Color-image
// rays are rotated into depth frame before intersecting the plane, and
// ray_origin is the color camera origin expressed in depth frame.
//
// Axis convention (valve body frame):
//   X = handle dir     -> roll  = valve face tilt
//   Y = cross product  -> pitch = valve face tilt
//   Z = plane normal   -> yaw   = handle rotation around normal
Eigen::Matrix3f PoseEstimator::create_rotation_matrix_depth(
    const pcl::ModelCoefficients::Ptr& coefficients,
    const Eigen::Vector3f& plane_normal,
    float angle,
    const Eigen::Vector3f& ray_origin,
    const Eigen::Matrix3f& R_depth_from_color) const {
    if (!coefficients || coefficients->values.size() < 4)
        return Eigen::Matrix3f::Identity();

    (void)coefficients;
    (void)ray_origin;

    const Eigen::Vector3f z_axis = plane_normal;
    const float fx = static_cast<float>(color_image_properties_.intr.fx);
    const float fy = static_cast<float>(color_image_properties_.intr.fy);

    // Build the handle direction directly from the image-plane angle, with no
    // dependence on per-pixel depth.  The image plane is perpendicular to the
    // color camera's Z axis, so a 2D displacement of (cos θ, sin θ) pixels
    // corresponds to a 3D direction (cos θ / fx, sin θ / fy, 0) in color
    // camera coordinates (the 1/f terms undo focal-length anisotropy).
    // Rotate that into depth frame and project onto the fitted plane so the
    // result lies in the valve face.  This avoids ray-plane intersections at
    // the handle endpoints, which otherwise amplify plane-normal noise into
    // large yaw jitter.
    const Eigen::Vector3f dir_color(std::cos(angle) / fx, std::sin(angle) / fy,
                                    0.f);
    Eigen::Vector3f x_axis = R_depth_from_color * dir_color;
    x_axis = (x_axis - x_axis.dot(z_axis) * z_axis).normalized();

    // Ensure consistent direction (avoid flipping between frames).
    // On the first valid computation filter_direction_ is zero; just accept
    // whatever direction the geometry gives us.
    if (!filter_direction_.isZero()) {
        if (filter_direction_.dot(x_axis) < 0)
            x_axis = -x_axis;
    }
    filter_direction_ = x_axis;

    const Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();
    x_axis = y_axis.cross(z_axis).normalized();

    Eigen::Matrix3f rot;
    rot.col(0) = x_axis;
    rot.col(1) = y_axis;
    rot.col(2) = z_axis;
    return rot;
}

// Extracts a point cloud from the depth image, fits a plane, and returns the
// valve pose.
DetectionResult PoseEstimator::compute_pose_from_depth(
    const cv::Mat& depth_image,
    const BoundingBox& bbox_org,
    float handle_angle_rad,
    DetectorMode mode) const {
    // Reject bounding boxes whose center projects entirely outside the depth
    // image.  The scale factor maps from color pixel space to depth pixel
    // space.
    const float scale =
        (depth_image_properties_.intr.fx > 0.0 &&
         color_image_properties_.intr.fx > 0.0)
            ? static_cast<float>(depth_image_properties_.intr.fx /
                                 color_image_properties_.intr.fx)
            : 1.0f;
    const float u_d = bbox_org.center_x * scale;
    const float v_d = bbox_org.center_y * scale;
    if (u_d < 0.0f || u_d >= static_cast<float>(depth_image.cols) ||
        v_d < 0.0f || v_d >= static_cast<float>(depth_image.rows))
        return {};

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // Extract points directly in the depth camera frame; only the OBB
    // membership test is done in the color frame (a per-pixel projection
    // that cannot be avoided without approximating the bbox in depth space).
    extract_bbox_pcl_depth(depth_image, bbox_org, color_image_properties_,
                           depth_image_properties_, depth_color_extrinsic_,
                           cloud);

    if (cloud->points.size() < 4)
        return {};

    DetectionResult result;

    if (mode == DetectorMode::debug) {
        result.annulus_cloud =
            std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
    }

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!segment_plane(cloud, coeff, inliers))
        return {};

    if (mode == DetectorMode::debug) {
        result.plane_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (int idx : inliers->indices)
            result.plane_cloud->points.push_back(cloud->points[idx]);
        result.plane_cloud->width =
            static_cast<uint32_t>(result.plane_cloud->points.size());
        result.plane_cloud->height = 1;
        result.plane_cloud->is_dense = false;
    }

    // The color bbox center defines a ray that originates at the color
    // camera, not the depth camera.  Express that ray in depth frame:
    //   origin    = -R^T * t      (color camera origin in depth frame)
    //   direction = R^T * K_c⁻¹ * [cx, cy, 1]ᵀ  (normalized)
    const Eigen::Matrix3f R_depth_from_color =
        depth_color_extrinsic_.R.transpose();
    const Eigen::Vector3f color_origin_in_depth_frame =
        -(R_depth_from_color * depth_color_extrinsic_.t);

    const float fx_c = static_cast<float>(color_image_properties_.intr.fx);
    const float fy_c = static_cast<float>(color_image_properties_.intr.fy);
    const float cx_c = static_cast<float>(color_image_properties_.intr.cx);
    const float cy_c = static_cast<float>(color_image_properties_.intr.cy);
    const Eigen::Vector3f r_c((bbox_org.center_x - cx_c) / fx_c,
                              (bbox_org.center_y - cy_c) / fy_c, 1.0f);
    const Eigen::Vector3f ray = (R_depth_from_color * r_c).normalized();

    const Eigen::Vector3f normal = compute_plane_normal(coeff, ray);
    if (normal.isZero())
        return {};

    const Eigen::Vector3f pos =
        find_ray_plane_intersection(coeff, ray, color_origin_in_depth_frame);
    if (pos.isZero())
        return {};

    const Eigen::Vector3f pos_shifted = shift_point_along_normal(pos, normal);

    const Eigen::Matrix3f rot = create_rotation_matrix_depth(
        coeff, normal, handle_angle_rad, color_origin_in_depth_frame,
        R_depth_from_color);
    result.pose =
        Pose::from_eigen(pos_shifted.cast<double>(),
                         Eigen::Quaternionf(rot).normalized().cast<double>());
    result.valid = true;
    return result;
}

}  // namespace valve_detection
