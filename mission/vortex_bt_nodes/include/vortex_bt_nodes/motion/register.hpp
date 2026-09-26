#ifndef VORTEX_BT_NODES__MOTION__REGISTER_HPP_
#define VORTEX_BT_NODES__MOTION__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace vortex_bt_nodes::motion {

/**
 * @brief Register the motion nodes (motion through waypoint_manager).
 *
 * Planned: SetDepth, Surface, Turn, HoldPosition, GoTo, MoveRelative,
 * GoToCourse, MoveCourse, FollowPoses, SelectGatePanel.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__REGISTER_HPP_
