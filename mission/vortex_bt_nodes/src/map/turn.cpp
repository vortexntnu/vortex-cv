#include "vortex_bt_nodes/map/turn.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <utility>

#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_bt_nodes::map {

Turn::Turn(const std::string& name,
           const BT::NodeConfig& config,
           rclcpp::Node::SharedPtr node,
           std::shared_ptr<const LandmarkMap> map)
    : NavAction(name, config, std::move(node)), map_(std::move(map)) {}

BT::PortsList Turn::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<double>("yaw_deg", "Heading in the map frame [deg]"),
         BT::InputPort<double>("relative_deg",
                               "Turn by this much, positive to the right "
                               "[deg]")});
}

std::optional<NavAction::Goal> Turn::make_goal() {
    const auto yaw_deg = getInput<double>("yaw_deg");
    const auto relative_deg = getInput<double>("relative_deg");
    if (yaw_deg.has_value() == relative_deg.has_value()) {
        spdlog::error("[{}] set exactly one of yaw_deg and relative_deg",
                      name());
        return std::nullopt;
    }

    
    double yaw = 0.0;
    if (yaw_deg) {

        const auto map_in_odom =
            map_->lookup(map_->odom_frame(), map_->frame_name("map"));
        if (!map_in_odom) {
            spdlog::warn("[{}] map is not in TF", name());
            return std::nullopt;
        }
        yaw = *yaw_deg * M_PI / 180.0 + yaw_of(*map_in_odom);
    } else {
        const auto vehicle = map_->vehicle_in_odom();
        if (!vehicle) {
            spdlog::warn("[{}] the vehicle is not in TF", name());
            return std::nullopt;
        }
        yaw = yaw_of(*vehicle) + *relative_deg * M_PI / 180.0;
    }

    Goal goal;
    goal.waypoints.push_back(
        make_waypoint(make_pose(0.0, 0.0, 0.0, yaw),
                      vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION));
    return goal;
}

}  // namespace vortex_bt_nodes::map
