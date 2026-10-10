#include "vortex_bt_nodes/map/go_to_pose.hpp"

#include <spdlog/spdlog.h>

#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

BT::PortsList GoToPose::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<Pose>("pose", "Committed pose (odom)"),
         BT::InputPort<Pose>("offset", "0;0;0", "x;y;z[;yaw_deg] in the pose"),
         BT::InputPort<std::string>("mode", "POSITION_AND_YAW",
                                    "WaypointMode name")});
}

std::optional<NavAction::Goal> GoToPose::make_goal() {
    const auto pose = getInput<Pose>("pose");
    const auto offset = getInput<Pose>("offset");
    const auto mode = waypoint_mode_from_string(
        getInput<std::string>("mode").value_or("POSITION_AND_YAW"));
    if (!pose || !offset || !mode) {
        spdlog::warn("[{}] no pose, bad offset or unknown mode", name());
        return std::nullopt;
    }
    const Pose target = LandmarkCache::compose(*pose, *offset);
    spdlog::info("[{}] target [{:.2f}, {:.2f}, {:.2f}] (odom, committed)",
                 name(), target.position.x, target.position.y,
                 target.position.z);
    vortex_msgs::msg::Waypoint wp;
    wp.pose = target;
    wp.waypoint_mode = *mode;
    Goal goal;
    goal.waypoints.push_back(wp);
    goal.convergence_threshold = 0.1;
    return goal;
}

}  // namespace vortex_bt_nodes::map
