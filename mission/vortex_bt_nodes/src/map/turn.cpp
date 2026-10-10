#include "vortex_bt_nodes/map/turn.hpp"

#include <spdlog/spdlog.h>

#include <cmath>

#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_bt_nodes::map {

BT::PortsList Turn::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<double>("yaw_deg", "Heading in the map frame [deg]"),
         BT::InputPort<double>("relative_deg", "Turn by this much [deg]")});
}

std::optional<NavAction::Goal> Turn::make_goal() {
    const auto absolute = getInput<double>("yaw_deg");
    const auto relative = getInput<double>("relative_deg");
    Goal goal;
    goal.convergence_threshold = 0.1;
    const auto mode = vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION;
    if (relative && !absolute) {
        goal.frame = Goal::BODY_RELATIVE;
        goal.waypoints.push_back(make_waypoint(
            make_pose(0.0, 0.0, 0.0, *relative * M_PI / 180.0), mode));
        return goal;
    }
    if (!absolute || relative) {
        spdlog::warn("[{}] give yaw_deg or relative_deg", name());
        return std::nullopt;
    }
    // Map heading -> odom heading through the map -> odom correction.
    const auto map_in_odom =
        cache_->lookup(cache_->odom_frame(), cache_->frame_name("map"));
    if (!map_in_odom) {
        spdlog::warn("[{}] no TF odom -> map", name());
        return std::nullopt;
    }
    const double yaw = *absolute * M_PI / 180.0 + yaw_of(*map_in_odom);
    goal.waypoints.push_back(
        make_waypoint(make_pose(0.0, 0.0, 0.0, yaw), mode));
    return goal;
}

}  // namespace vortex_bt_nodes::map
