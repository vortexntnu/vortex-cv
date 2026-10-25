#ifndef _VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_

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

optional<Goal> MoveRelative::make_goal() {}

}  // namespace vortex_bt_nodes::motion

#endif  // _VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_
