#include "vortex_bt_nodes/motion/move_relative_node.hpp"

#include <spdlog/spdlog.h>

#include <optional>
#include <string>

namespace vortex_bt_nodes::motion {

namespace {

constexpr char kFrameBody[] = "BODY";
constexpr char kFrameWorld[] = "WORLD";
constexpr char kModeDefault[] = "POSITION_AND_YAW";

}  // namespace

MoveRelative::MoveRelative(const std::string& name,
                           const BT::NodeConfig& config,
                           rclcpp::Node::SharedPtr node)
    : NavAction(name, config, std::move(node)) {}

BT::PortsList MoveRelative::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<Pose>("offset", "0;0;0",
                             "Offset from the pose at start [m], yaw [deg]"),
         BT::InputPort<std::string>("frame", kFrameBody,
                                    "BODY = vehicle axes, WORLD = odom axes"),
         BT::InputPort<std::string>("mode", kModeDefault,
                                    "WaypointMode name")});
}

std::optional<MoveRelative::Goal> MoveRelative::make_goal() {
    const auto offset = getInput<Pose>("offset");
    if (!offset) {
        spdlog::error("[{}] {}", name(), offset.error());
        return std::nullopt;
    }

    const std::string frame =
        getInput<std::string>("frame").value_or(kFrameBody);
    if (frame != kFrameBody && frame != kFrameWorld) {
        spdlog::error("[{}] frame must be BODY or WORLD, not {}", name(),
                      frame);
        return std::nullopt;
    }

    const std::string mode_name =
        getInput<std::string>("mode").value_or(kModeDefault);
    const auto mode = waypoint_mode_from_string(mode_name);
    if (!mode) {
        spdlog::error("[{}] unknown mode: {}", name(), mode_name);
        return std::nullopt;
    }

    Goal goal;
    goal.frame =
        frame == kFrameBody ? Goal::BODY_RELATIVE : Goal::WORLD_RELATIVE;
    goal.waypoints.push_back(make_waypoint(*offset, mode->mode));
    return goal;
}

}  // namespace vortex_bt_nodes::motion
