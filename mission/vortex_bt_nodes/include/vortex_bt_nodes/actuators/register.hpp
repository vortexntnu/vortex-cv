#ifndef VORTEX_BT_NODES__ACTUATORS__REGISTER_HPP_
#define VORTEX_BT_NODES__ACTUATORS__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace vortex_bt_nodes::actuators {

/**
 * @brief Register the actuators nodes (actuators).
 *
 * Planned: DropMarker, FireTorpedo, SetGripper, MarkUsed.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace vortex_bt_nodes::actuators

#endif  // VORTEX_BT_NODES__ACTUATORS__REGISTER_HPP_
