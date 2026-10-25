#include "vortex_bt_nodes/map/look_at_frame.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <utility>

#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_bt_nodes::map {

namespace {
// Closer than this horizontally the direction to the frame is meaningless.
constexpr double kMinDistanceM = 0.1;
}  // namespace

LookAtFrame::LookAtFrame(const std::string& name,
                         const BT::NodeConfig& config,
                         rclcpp::Node::SharedPtr node,
                         std::shared_ptr<const LandmarkMap> map)
    : NavAction(name, config, std::move(node)), map_(std::move(map)) {}

BT::PortsList LookAtFrame::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<std::string>("frame", "TF frame to face")});
}

std::optional<NavAction::Goal> LookAtFrame::make_goal() {
    const auto frame = getInput<std::string>("frame");
    if (!frame) {
        spdlog::error("[{}] {}", name(), frame.error());
        return std::nullopt;
    }


    const auto target =
        map_->lookup(map_->odom_frame(), map_->frame_name(*frame));
    if (!target) {
        spdlog::warn("[{}] frame '{}' is not in TF", name(),
                     map_->frame_name(*frame));
        return std::nullopt;
    }
    const auto vehicle = map_->vehicle_in_odom();
    if (!vehicle) {
        spdlog::warn("[{}] the vehicle is not in TF", name());
        return std::nullopt;
    }

    const double dx = target->position.x - vehicle->position.x;
    const double dy = target->position.y - vehicle->position.y;
    if (std::hypot(dx, dy) < kMinDistanceM) {
        spdlog::warn("[{}] '{}' is straight above or below the vehicle",
                     name(), *frame);
        return std::nullopt;
    }

    Goal goal;
    goal.waypoints.push_back(
        make_waypoint(make_pose(0.0, 0.0, 0.0, std::atan2(dy, dx)),
                      vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION));
    return goal;
}

}  // namespace vortex_bt_nodes::map
