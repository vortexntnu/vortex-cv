#ifndef VORTEX_BT_NODES__APPROACH__REGISTER_HPP_
#define VORTEX_BT_NODES__APPROACH__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace vortex_bt_nodes::approach {

/** @brief Registers the approach nodes. */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace vortex_bt_nodes::approach

#endif  // VORTEX_BT_NODES__APPROACH__REGISTER_HPP_
