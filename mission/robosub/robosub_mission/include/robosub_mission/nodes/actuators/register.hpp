#ifndef ROBOSUB_MISSION__NODES__ACTUATORS__REGISTER_HPP_
#define ROBOSUB_MISSION__NODES__ACTUATORS__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace robosub_mission::nodes::actuators {

/**
 * @brief Register the actuators nodes (actuators).
 *
 * Planned: DropMarker, FireTorpedo, SetGripper, MarkUsed.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace robosub_mission::nodes::actuators

#endif  // ROBOSUB_MISSION__NODES__ACTUATORS__REGISTER_HPP_
