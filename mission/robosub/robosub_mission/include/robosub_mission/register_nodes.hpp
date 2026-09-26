#ifndef ROBOSUB_MISSION__REGISTER_NODES_HPP_
#define ROBOSUB_MISSION__REGISTER_NODES_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace robosub_mission {

/**
 * @brief Register the mission's custom nodes (conditions and actions) with
 * the factory, before the trees are loaded. Nodes that talk to ROS get
 * @p node.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace robosub_mission

#endif  // ROBOSUB_MISSION__REGISTER_NODES_HPP_
