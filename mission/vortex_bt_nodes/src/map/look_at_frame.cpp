#include "vortex_bt_nodes/map/look_at_frame.hpp"

#include <spdlog/spdlog.h>

#include <cmath>

#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_bt_nodes::map {

BT::PortsList LookAtFrame::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<std::string>("frame", "TF frame to face")});
}

std::optional<NavAction::Goal> LookAtFrame::make_goal() {
    const auto frame = getInput<std::string>("frame");
    const auto target =
        frame ? cache_->lookup(cache_->odom_frame(), cache_->frame_name(*frame))
              : std::nullopt;
    const auto vehicle = cache_->vehicle_in_odom();
    if (!target || !vehicle) {
        spdlog::warn("[{}] frame '{}' or the vehicle not in TF", name(),
                     frame.value_or(""));
        return std::nullopt;
    }
    const double yaw = std::atan2(target->position.y - vehicle->position.y,
                                  target->position.x - vehicle->position.x);
    Goal goal;
    goal.convergence_threshold = 0.1;
    goal.waypoints.push_back(
        make_waypoint(make_pose(0.0, 0.0, 0.0, yaw),
                      vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION));
    return goal;
}

}  // namespace vortex_bt_nodes::map
