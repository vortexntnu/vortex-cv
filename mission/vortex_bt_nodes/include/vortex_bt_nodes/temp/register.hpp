#ifndef VORTEX_BT_NODES__TEMP__REGISTER_HPP_
#define VORTEX_BT_NODES__TEMP__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

// Temporary nodes to try the measure workflow in the simulator. To be
// replaced by the members' own nodes.
namespace vortex_bt_nodes::temp {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace vortex_bt_nodes::temp

#endif  // VORTEX_BT_NODES__TEMP__REGISTER_HPP_
