#include "vortex_bt_nodes/motion/surface.hpp"

#include <spdlog/spdlog.h>
#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_bt_nodes::motion {

Surface::Surface(const std::string& name,
                 const BT::NodeConfig& config,
                 rclcpp::Node::SharedPtr node)
    : NavAction(name, config, std::move(node)) {}

BT::PortsList Surface::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<double>("z", 0.2, "Surface depth [m]")});
}

std::optional<NavAction::Goal> Surface::make_goal() {
    const auto z = getInput<double>("z");

    if (!z) {
        spdlog::error("[{}] {}", name(), z.error());
        return std::nullopt;
    }

    if (*z < 0.0) {
        spdlog::error("[{}] z is negative: {}", name(), *z);
        return std::nullopt;
    }

    geometry_msgs::msg::Pose pose;
    pose.position.z = *z;

    Goal goal;
    goal.waypoints.push_back(
        make_waypoint(pose, vortex_msgs::msg::WaypointMode::ONLY_Z));
    goal.frame = Goal::WORLD;

    return goal;
}

}  // namespace vortex_bt_nodes::motion
