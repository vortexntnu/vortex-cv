#include "vortex_bt_nodes/motion/set_depth.hpp"

#include <spdlog/spdlog.h>
#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_bt_nodes::motion {

SetDepth::SetDepth(const std::string& name,
                   const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node)
    : NavAction(name, config, std::move(node)) {}

BT::PortsList SetDepth::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<double>("z", "Target depth [m]")
    });
}

std::optional<NavAction::Goal> SetDepth::make_goal() {
    const auto z = getInput<double>("z");

    if (!z) {
        spdlog::error("[{}] {}", name(), z.error());
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