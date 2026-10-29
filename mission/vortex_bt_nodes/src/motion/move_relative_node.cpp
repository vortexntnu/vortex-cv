#ifndef _VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_

#include <spdlog/spdlog.h>
#include <optional>
#include <vortex_bt_nodes/motion/move_relative_node.hpp>

namespace vortex_bt_nodes::motion {

MoveRelative::MoveRelative(const std::string& name,
                           const BT::NodeConfig& config,
                           const rclcpp::Node::SharedPtr& node,
                           const std::shared_ptr<const map::LandmarkMap> map)
    : NavAction(name, config, node), map_(map) {}

BT::PortsList MoveRelative::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort("offset", "0;0;0",
                       "Move to offset relative to position"),
         BT::InputPort("frame", "baselink", "Move relative to this frame"),
         BT::InputPort("mode", "ONLY_POSITION", "Mode to move in")});
}

std::optional<Pose> MoveRelative::target_in_odom() const {
    const auto frame = getInput<std::string>("frame");
    const auto offset = getInput<Pose>("offset");
    if (!frame || !offset) {
        return std::nullopt;
    }
    // The controller works in odom, so ask for the frame there and not in
    // map: a frame fixed in map moves whenever the map corrects drift.
    const auto frame_in_odom =
        map_->lookup(map_->odom_frame(), map_->frame_name(*frame));
    if (!frame_in_odom) {
        return std::nullopt;
    }
    Pose target = LandmarkMap::compose(*frame_in_odom, *offset);

    const auto tool = getInput<std::string>("tool_frame");
    if (tool && !tool->empty()) {
        // Put tool_frame on the target instead of base_link:
        // T_odom_base = T_odom_tool * T_tool_base.
        const auto base_in_tool =
            map_->lookup(map_->frame_name(*tool), map_->frame_name(kBaseFrame));
        if (!base_in_tool) {
            return std::nullopt;
        }
        target = LandmarkMap::compose(target, *base_in_tool);
    }
    return target;
}
optional<NavAction::Goal> MoveRelative::make_goal() {
    const auto frame = getInput<std::string>("frame");
    if (!frame) {
        spdlog::error("[{}] {}", name(), frame.error());
        return std::nullopt;
    }
    if (frame != "BODY" || frame != "WORLD") {
        spdlog::error("[{}] Only WORLD and BODY frame allowed, not ", name(),
                      frame);
    }
    const auto target = target_in_odom();

    const auto mode = getInput<std::string>("mode");

    last_target_ = *target;

    NavAction::Goal goal;
    goal.waypoints.push_back(
        make_waypoint(*target, static_cast<std::uint8_t>(mode)));
}

}  // namespace vortex_bt_nodes::motion

#endif  // _VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_
