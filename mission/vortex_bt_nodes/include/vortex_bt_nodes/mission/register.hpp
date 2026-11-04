#ifndef VORTEX_BT_NODES__MISSION__REGISTER_HPP_
#define VORTEX_BT_NODES__MISSION__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace vortex_bt_nodes::mission {

/** @brief Registers the mission nodes. */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace vortex_bt_nodes::mission

#endif  // VORTEX_BT_NODES__MISSION__REGISTER_HPP_
