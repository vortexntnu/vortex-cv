#ifndef VORTEX_BT_NODES__REGISTER_NODES_HPP_
#define VORTEX_BT_NODES__REGISTER_NODES_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace vortex_bt_nodes {

/**
 * @brief Register all custom nodes with the factory, before the trees are
 * loaded. Calls each area's register_nodes (<area>/register.hpp).
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace vortex_bt_nodes

#endif  // VORTEX_BT_NODES__REGISTER_NODES_HPP_
