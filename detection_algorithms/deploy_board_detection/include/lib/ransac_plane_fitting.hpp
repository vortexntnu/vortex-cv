#pragma once

#include <vector>
#include <geometry_msgs/msg/point32.hpp>

namespace vortex::board_detection {

struct Plane {
    double a;
    double b;
    double c;
    double d;
};

Plane ransacPlaneFitting(
    const std::vector<geometry_msgs::msg::Point32>& relative_board_point_cloud,
    double distance_threshold,
    int num_iterations,
    int min_inliers
);

} // namespace vortex::board_detection