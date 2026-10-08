#include "vortex_bt_nodes/motion/move_relative.hpp"

#include <spdlog/spdlog.h>

namespace vortex_bt_nodes::motion {

BT::PortsList MoveRelative::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<Pose>("offset", "x;y;z[;yaw_deg]"),
         BT::InputPort<std::string>("frame", "BODY", "BODY or WORLD"),
         BT::InputPort<std::string>("mode", "POSITION_AND_YAW",
                                    "WaypointMode name")});
}

std::optional<NavAction::Goal> MoveRelative::make_goal() {
    const auto offset = getInput<Pose>("offset");
    const auto mode = waypoint_mode_from_string(
        getInput<std::string>("mode").value_or("POSITION_AND_YAW"));
    const std::string frame = getInput<std::string>("frame").value_or("BODY");
    if (!offset || !mode || (frame != "BODY" && frame != "WORLD")) {
        spdlog::warn("[{}] bad offset, mode or frame", name());
        return std::nullopt;
    }
    Goal goal;
    goal.frame = frame == "BODY" ? Goal::BODY_RELATIVE : Goal::WORLD_RELATIVE;
    goal.waypoints.push_back(make_waypoint(*offset, mode->mode));
    goal.convergence_threshold = 0.1;
    return goal;
}

}  // namespace vortex_bt_nodes::motion
