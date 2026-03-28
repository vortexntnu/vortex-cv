// depth_image_processing.cpp
// Depth-to-3D back-projection and color-to-depth pixel projection.
#include "valve_detection/depth_image_processing.hpp"
#include <Eigen/Dense>
#include <limits>

namespace valve_detection {

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
    // Back-project color pixel → 3-D point in color camera frame.
    const float Xc = (u_c - static_cast<float>(color_props.intr.cx)) * Z /
                     static_cast<float>(color_props.intr.fx);
    const float Yc = (v_c - static_cast<float>(color_props.intr.cy)) * Z /
                     static_cast<float>(color_props.intr.fy);
    const Eigen::Vector3f Pc(Xc, Yc, Z);

    // Transform to depth camera frame:  P_depth = R^T * (P_color - t)
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
