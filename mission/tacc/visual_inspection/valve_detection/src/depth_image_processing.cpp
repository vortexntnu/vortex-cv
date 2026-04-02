// depth_image_processing.cpp
// Depth-to-3D back-projection and color-to-depth pixel projection.
#include "valve_detection/depth_image_processing.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace valve_detection {

/// TODO: Had problems with this, when it undistorted the orientation would be
/// off by a constant offset Workaround is to just disable this and undistort
/// the whole image instead (done upstream in perception_setup)
BoundingBox undistort_bbox(const BoundingBox& bbox,
                           const CameraIntrinsics& intr) {
    const cv::Mat K = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.cx, 0,
                       intr.fy, intr.cy, 0, 0, 1);
    const cv::Mat D = (cv::Mat_<double>(5, 1) << intr.dist[0], intr.dist[1],
                       intr.dist[2], intr.dist[3], intr.dist[4]);

    const float angle_deg = bbox.theta * 180.0f / static_cast<float>(M_PI);
    cv::RotatedRect rrect(cv::Point2f(bbox.center_x, bbox.center_y),
                          cv::Size2f(bbox.size_x, bbox.size_y), angle_deg);

    // Extract 4 corners to compute edge midpoints.
    cv::Point2f corners[4];
    rrect.points(corners);

    // 5 sample points: 4 edge midpoints + bbox center.
    std::vector<cv::Point2f> pts;
    pts.reserve(5);
    for (int i = 0; i < 4; ++i)
        pts.emplace_back((corners[i] + corners[(i + 1) % 4]) * 0.5f);
    pts.emplace_back(bbox.center_x, bbox.center_y);

    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(pts, undistorted, K, D, cv::noArray(), K);

    // Anchor result to the undistorted center (last point).
    const cv::Point2f undist_center = undistorted.back();

    // Refit an OBB to the 4 undistorted edge midpoints.
    std::vector<cv::Point2f> midpoints(undistorted.begin(),
                                       undistorted.begin() + 4);
    cv::RotatedRect fitted = cv::minAreaRect(midpoints);

    // minAreaRect returns angles in [-90, 0) and can flip which side it treats
    // as the major axis, producing an angle ~90° off from the original for
    // nearly-square boxes (e.g. circular valves). Normalise to stay within 90°
    // of the input angle and swap width/height accordingly.
    float fitted_angle_deg = fitted.angle;
    float diff = fitted_angle_deg - angle_deg;
    while (diff > 90.0f) {
        diff -= 180.0f;
        fitted_angle_deg -= 180.0f;
    }
    while (diff < -90.0f) {
        diff += 180.0f;
        fitted_angle_deg += 180.0f;
    }
    const bool swapped = std::abs(fitted_angle_deg - fitted.angle) > 45.0f;

    BoundingBox result = bbox;
    result.center_x = undist_center.x;
    result.center_y = undist_center.y;
    result.size_x = swapped ? fitted.size.height : fitted.size.width;
    result.size_y = swapped ? fitted.size.width : fitted.size.height;
    result.theta = fitted_angle_deg * static_cast<float>(M_PI) / 180.0f;
    return result;
}

// Back-projects a depth pixel (u, v, depth) to a 3D point using camera
// intrinsics.
void project_pixel_to_point(const int u,
                            const int v,
                            const float depth,
                            const double fx,
                            const double fy,
                            const double cx,
                            const double cy,
                            pcl::PointXYZ& out) {
    if (depth <= 0.0f || std::isnan(depth) || std::isinf(depth)) {
        out.x = out.y = out.z = std::numeric_limits<float>::quiet_NaN();
        return;
    }
    out.x = static_cast<float>((u - cx) * depth / fx);
    out.y = static_cast<float>((v - cy) * depth / fy);
    out.z = depth;
}

// Projects a color image pixel to depth image coordinates using the full
// intrinsic + extrinsic pipeline.
cv::Point2f project_color_pixel_to_depth(const float u_c,
                                         const float v_c,
                                         const float Z,
                                         const ImageProperties& color_props,
                                         const ImageProperties& depth_props,
                                         const DepthColorExtrinsic& extr) {
    // Back-project color pixel -> 3-D point in color camera frame.
    const float Xc = (u_c - static_cast<float>(color_props.intr.cx)) * Z /
                     static_cast<float>(color_props.intr.fx);
    const float Yc = (v_c - static_cast<float>(color_props.intr.cy)) * Z /
                     static_cast<float>(color_props.intr.fy);
    const Eigen::Vector3f Pc(Xc, Yc, Z);

    // Transform to depth camera frame: P_depth = R^T * (P_color - t)
    const Eigen::Vector3f Pd = extr.R.transpose() * (Pc - extr.t);

    if (Pd.z() <= 0.0f)
        return {u_c, v_c};  // degenerate – return original

    // Project into depth image.
    const float u_d =
        static_cast<float>(depth_props.intr.fx) * Pd.x() / Pd.z() +
        static_cast<float>(depth_props.intr.cx);
    const float v_d =
        static_cast<float>(depth_props.intr.fy) * Pd.y() / Pd.z() +
        static_cast<float>(depth_props.intr.cy);
    return {u_d, v_d};
}

}  // namespace valve_detection
