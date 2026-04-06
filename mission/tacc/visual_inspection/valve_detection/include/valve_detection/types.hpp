#ifndef VALVE_DETECTION__TYPES_HPP_
#define VALVE_DETECTION__TYPES_HPP_

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <vortex/utils/types.hpp>

namespace valve_detection {

struct CameraIntrinsics {
    double fx{0}, fy{0}, cx{0}, cy{0};
    std::array<double, 5> dist{0, 0, 0, 0, 0};  // k1, k2, p1, p2, k3
};

struct ImageDimensions {
    int width{0}, height{0};
};

struct ImageProperties {
    CameraIntrinsics intr;
    ImageDimensions dim;
};

struct BoundingBox {
    float center_x{0};
    float center_y{0};
    float size_x{0};
    float size_y{0};
    float theta{0};  // radians
};

using Pose = vortex::utils::types::Pose;

// Rigid transform from depth camera frame to color camera frame.
// Rotation R and translation t satisfy:  P_color = R * P_depth + t
struct DepthColorExtrinsic {
    Eigen::Matrix3f R{Eigen::Matrix3f::Identity()};
    Eigen::Vector3f t{Eigen::Vector3f::Zero()};
};

}  // namespace valve_detection

#endif  // VALVE_DETECTION__TYPES_HPP_
