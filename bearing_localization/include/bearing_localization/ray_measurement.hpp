#pragma once

#include <Eigen/Dense>
#include <rclcpp/time.hpp>

namespace bearing_localization {

struct RayMeasurement {
    rclcpp::Time stamp;
    Eigen::Vector3d origin_world;
    Eigen::Vector3d direction_world;
    double weight{1.0};
};

}  // namespace bearing_localization
