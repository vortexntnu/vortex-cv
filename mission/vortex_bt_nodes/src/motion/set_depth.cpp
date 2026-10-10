#include "vortex_bt_nodes/motion/set_depth.hpp"

#include <spdlog/spdlog.h>

#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_bt_nodes::motion {

BT::PortsList SetDepth::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<double>("z", "Depth [m] (odom z, down positive)")});
}

std::optional<NavAction::Goal> SetDepth::make_goal() {
    const auto z = getInput<double>("z");
    if (!z) {
        spdlog::warn("[{}] missing z", name());
        return std::nullopt;
    }
    Pose pose;
    pose.position.z = *z;
    Goal goal;
    goal.waypoints.push_back(
        make_waypoint(pose, vortex_msgs::msg::WaypointMode::ONLY_Z));
    goal.convergence_threshold = 0.1;
    return goal;
}

}  // namespace vortex_bt_nodes::motion
