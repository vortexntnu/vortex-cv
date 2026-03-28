#include "valve_detection/pcl_extraction.hpp"
#include "valve_detection/depth_image_processing.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace valve_detection {

// Iterates depth pixels, back-projects each with depth intrinsics, applies
// the extrinsic transform, and checks whether the resulting color-frame
// projection falls inside the elliptic annulus defined by color_bbox.
// Output points are in the color camera frame.
void extract_annulus_pcl_aligned(const cv::Mat& depth_image,
                                 const BoundingBox& color_bbox,
                                 const ImageProperties& color_props,
                                 const ImageProperties& depth_props,
                                 const DepthColorExtrinsic& extrinsic,
                                 const float annulus_radius_ratio,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    out->clear();

    // Annulus defined in color-image coordinates.
    const float cx_c = color_bbox.center_x;
    const float cy_c = color_bbox.center_y;
    const float outer_rx = color_bbox.size_x * 0.5f;
    const float outer_ry = color_bbox.size_y * 0.5f;

    // Require at least 4 pixels in each dimension (2 px half-extent) before
    // sampling — smaller boxes contain no usable depth points.
    if (outer_rx < 2.0f || outer_ry < 2.0f)
        return;

    // Inner ellipse scaled down by annulus_radius_ratio, forming the ring.
    const float inner_rx = outer_rx * annulus_radius_ratio;
    const float inner_ry = outer_ry * annulus_radius_ratio;

    // The depth and color images have different resolutions and focal lengths.
    // Scale color-space bbox bounds into depth-image coordinates to get a
    // coarse candidate region, then add a margin to cover the lateral shift
    // introduced by the depth-to-color translation.
    const float scale =
        (depth_props.intr.fx > 0.0 && color_props.intr.fx > 0.0)
            ? static_cast<float>(depth_props.intr.fx / color_props.intr.fx)
            : 1.0f;
    constexpr int kMargin = 30;

    const int u0_d =
        std::max(0, static_cast<int>((cx_c - outer_rx) * scale) - kMargin);
    const int u1_d =
        std::min(depth_image.cols - 1,
                 static_cast<int>((cx_c + outer_rx) * scale) + kMargin);
    const int v0_d =
        std::max(0, static_cast<int>((cy_c - outer_ry) * scale) - kMargin);
    const int v1_d =
        std::min(depth_image.rows - 1,
                 static_cast<int>((cy_c + outer_ry) * scale) + kMargin);

    for (int v_d = v0_d; v_d <= v1_d; ++v_d) {
        for (int u_d = u0_d; u_d <= u1_d; ++u_d) {
            const float z_d = depth_image.at<float>(v_d, u_d);
            if (z_d <= 0.0f || std::isnan(z_d) || std::isinf(z_d))
                continue;

            // Back-project depth pixel to 3-D point in depth camera frame.
            Eigen::Vector3f P_d;
            P_d.x() = static_cast<float>((u_d - depth_props.intr.cx) * z_d /
                                         depth_props.intr.fx);
            P_d.y() = static_cast<float>((v_d - depth_props.intr.cy) * z_d /
                                         depth_props.intr.fy);
            P_d.z() = z_d;

            // Apply extrinsic to move the point into the color camera frame,
            // where the annulus is defined.
            const Eigen::Vector3f P_c = extrinsic.R * P_d + extrinsic.t;
            if (P_c.z() <= 0.0f)
                continue;

            // Project the color-frame point onto the color image plane.
            const float u_c = static_cast<float>(
                color_props.intr.fx * P_c.x() / P_c.z() + color_props.intr.cx);
            const float v_c = static_cast<float>(
                color_props.intr.fy * P_c.y() / P_c.z() + color_props.intr.cy);

            // Normalise offset by outer half-extents; keep only points inside
            // the outer ellipse (sum of squares ≤ 1).
            const float dxo = (u_c - cx_c) / outer_rx;
            const float dyo = (v_c - cy_c) / outer_ry;
            if (dxo * dxo + dyo * dyo > 1.0f)
                continue;  // outside outer ellipse

            // Discard points inside the inner ellipse to form the ring.
            const float dxi = (u_c - cx_c) / inner_rx;
            const float dyi = (v_c - cy_c) / inner_ry;
            if (dxi * dxi + dyi * dyi <= 1.0f)
                continue;  // inside inner ellipse

            // Store the point in the color camera frame.
            pcl::PointXYZ p;
            p.x = P_c.x();
            p.y = P_c.y();
            p.z = P_c.z();
            out->points.push_back(p);
        }
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;
}

// Extracts depth points whose color-frame projection falls inside the oriented
// bounding box.
void extract_bbox_pcl_aligned(const cv::Mat& depth_image,
                              const BoundingBox& color_bbox,
                              const ImageProperties& color_props,
                              const ImageProperties& depth_props,
                              const DepthColorExtrinsic& extrinsic,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    out->clear();

    // Pre-compute the OBB axes in color-image space for a fast rotated-rect
    // test.
    const float cos_t = std::cos(color_bbox.theta);
    const float sin_t = std::sin(color_bbox.theta);
    const float half_w = color_bbox.size_x * 0.5f;
    const float half_h = color_bbox.size_y * 0.5f;

    // Approximate search region in depth-image space.
    const float scale =
        (depth_props.intr.fx > 0.0 && color_props.intr.fx > 0.0)
            ? static_cast<float>(depth_props.intr.fx / color_props.intr.fx)
            : 1.0f;
    const float r = std::sqrt(half_w * half_w + half_h * half_h);
    constexpr int kMargin = 30;

    const int u0_d = std::max(
        0, static_cast<int>((color_bbox.center_x - r) * scale) - kMargin);
    const int u1_d =
        std::min(depth_image.cols - 1,
                 static_cast<int>((color_bbox.center_x + r) * scale) + kMargin);
    const int v0_d = std::max(
        0, static_cast<int>((color_bbox.center_y - r) * scale) - kMargin);
    const int v1_d =
        std::min(depth_image.rows - 1,
                 static_cast<int>((color_bbox.center_y + r) * scale) + kMargin);

    for (int v_d = v0_d; v_d <= v1_d; ++v_d) {
        for (int u_d = u0_d; u_d <= u1_d; ++u_d) {
            const float z_d = depth_image.at<float>(v_d, u_d);
            if (z_d <= 0.0f || std::isnan(z_d) || std::isinf(z_d))
                continue;

            // Back-project to depth camera frame.
            Eigen::Vector3f P_d;
            P_d.x() = static_cast<float>((u_d - depth_props.intr.cx) * z_d /
                                         depth_props.intr.fx);
            P_d.y() = static_cast<float>((v_d - depth_props.intr.cy) * z_d /
                                         depth_props.intr.fy);
            P_d.z() = z_d;

            // Transform to color camera frame.
            const Eigen::Vector3f P_c = extrinsic.R * P_d + extrinsic.t;
            if (P_c.z() <= 0.0f)
                continue;

            // Project onto color image plane.
            const float u_c = static_cast<float>(
                color_props.intr.fx * P_c.x() / P_c.z() + color_props.intr.cx);
            const float v_c = static_cast<float>(
                color_props.intr.fy * P_c.y() / P_c.z() + color_props.intr.cy);

            // Rotate into the OBB local frame and test against the
            // half-extents.
            const float dx = u_c - color_bbox.center_x;
            const float dy = v_c - color_bbox.center_y;
            const float local_x = cos_t * dx + sin_t * dy;
            const float local_y = -sin_t * dx + cos_t * dy;

            if (std::abs(local_x) > half_w || std::abs(local_y) > half_h)
                continue;

            out->points.push_back({P_c.x(), P_c.y(), P_c.z()});
        }
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;
}

// Projects the 4 corners of the color OBB into depth image space once, fits
// an OBB to those projected corners, then tests depth pixels directly against
// that depth-image OBB — no per-pixel matrix multiply needed.  Output points
// are stored in the depth camera frame.
void extract_bbox_pcl_depth(const cv::Mat& depth_image,
                            const BoundingBox& color_bbox,
                            const ImageProperties& color_props,
                            const ImageProperties& depth_props,
                            const DepthColorExtrinsic& extrinsic,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    out->clear();

    // Sample a representative depth near the bbox center to project the color
    // OBB corners into depth image space.  The valve is roughly flat, so a
    // single Z gives a good approximation for all 4 corners.
    const float scale =
        (depth_props.intr.fx > 0.0 && color_props.intr.fx > 0.0)
            ? static_cast<float>(depth_props.intr.fx / color_props.intr.fx)
            : 1.0f;
    const int u_c_d = std::clamp(static_cast<int>(color_bbox.center_x * scale),
                                 0, depth_image.cols - 1);
    const int v_c_d = std::clamp(static_cast<int>(color_bbox.center_y * scale),
                                 0, depth_image.rows - 1);

    float Z_est = 0.0f;
    constexpr int kSampleRadius = 5;
    for (int dv = -kSampleRadius; dv <= kSampleRadius && Z_est <= 0.0f; ++dv) {
        for (int du = -kSampleRadius; du <= kSampleRadius && Z_est <= 0.0f;
             ++du) {
            const int u = std::clamp(u_c_d + du, 0, depth_image.cols - 1);
            const int v = std::clamp(v_c_d + dv, 0, depth_image.rows - 1);
            const float z = depth_image.at<float>(v, u);
            if (z > 0.0f && !std::isnan(z) && !std::isinf(z))
                Z_est = z;
        }
    }
    if (Z_est <= 0.0f)
        return;

    // Project the 4 color OBB corners into depth image coordinates.
    const float angle_deg =
        color_bbox.theta * 180.0f / static_cast<float>(M_PI);
    cv::RotatedRect color_rrect(
        cv::Point2f(color_bbox.center_x, color_bbox.center_y),
        cv::Size2f(color_bbox.size_x, color_bbox.size_y), angle_deg);
    cv::Point2f color_corners[4];
    color_rrect.points(color_corners);

    std::vector<cv::Point2f> depth_corners(4);
    for (int i = 0; i < 4; ++i) {
        depth_corners[i] =
            project_color_pixel_to_depth(color_corners[i].x, color_corners[i].y,
                                         Z_est, color_props, depth_props, extrinsic);
    }

    // Fit a rotated rect to the 4 projected corners in depth image space.
    const cv::RotatedRect depth_rrect = cv::minAreaRect(depth_corners);
    const float angle_d = depth_rrect.angle * static_cast<float>(M_PI) / 180.0f;
    const float cos_d = std::cos(angle_d);
    const float sin_d = std::sin(angle_d);
    const float half_w_d = depth_rrect.size.width * 0.5f;
    const float half_h_d = depth_rrect.size.height * 0.5f;
    const float cx_d = depth_rrect.center.x;
    const float cy_d = depth_rrect.center.y;

    // Bounding rectangle of the depth OBB.
    const float r_d = std::sqrt(half_w_d * half_w_d + half_h_d * half_h_d);
    const int u0_d = std::max(0, static_cast<int>(cx_d - r_d) - 1);
    const int u1_d =
        std::min(depth_image.cols - 1, static_cast<int>(cx_d + r_d) + 1);
    const int v0_d = std::max(0, static_cast<int>(cy_d - r_d) - 1);
    const int v1_d =
        std::min(depth_image.rows - 1, static_cast<int>(cy_d + r_d) + 1);

    for (int v_d = v0_d; v_d <= v1_d; ++v_d) {
        for (int u_d = u0_d; u_d <= u1_d; ++u_d) {
            // Test against the depth-image OBB — no matrix multiply.
            const float dx = u_d - cx_d;
            const float dy = v_d - cy_d;
            if (std::abs(cos_d * dx + sin_d * dy) > half_w_d ||
                std::abs(-sin_d * dx + cos_d * dy) > half_h_d)
                continue;

            const float z_d = depth_image.at<float>(v_d, u_d);
            if (z_d <= 0.0f || std::isnan(z_d) || std::isinf(z_d))
                continue;

            // Back-project to 3D depth frame.
            out->points.push_back(
                {static_cast<float>((u_d - depth_props.intr.cx) * z_d /
                                    depth_props.intr.fx),
                 static_cast<float>((v_d - depth_props.intr.cy) * z_d /
                                    depth_props.intr.fy),
                 z_d});
        }
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;
}

}  // namespace valve_detection
