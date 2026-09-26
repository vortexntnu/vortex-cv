#ifndef ROBOSUB_MISSION__REGISTER_NODES_HPP_
#define ROBOSUB_MISSION__REGISTER_NODES_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace robosub_mission {

/**
 * @brief Register all custom nodes with the factory, before the trees are
 * loaded. Calls each area's register_nodes (nodes/<area>/register.hpp).
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace robosub_mission

#endif  // ROBOSUB_MISSION__REGISTER_NODES_HPP_
