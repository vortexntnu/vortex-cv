#include "lib/ransac_plane_fitting.hpp"

namespace vortex::board_detection {

Plane ransacPlaneFitting(
    const std::vector<geometry_msgs::msg::Point32>& relative_board_point_cloud,
    double distance_threshold_,
    int num_iterations_,
    int min_inliers_
) {
    Plane result{};

    // TODO: Implement RANSAC

    return result;
}

}