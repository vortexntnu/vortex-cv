#ifndef ROBOSUB_MISSION__NODES__MOTION__REGISTER_HPP_
#define ROBOSUB_MISSION__NODES__MOTION__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace robosub_mission::nodes::motion {

/**
 * @brief Register the motion nodes (M1: motion through waypoint_manager).
 *
 * Planned: SetDepth, Surface, Turn, HoldPosition, GoTo, MoveRelative,
 * GoToCourse, MoveCourse, FollowPoses, SelectGatePanel.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace robosub_mission::nodes::motion

#endif  // ROBOSUB_MISSION__NODES__MOTION__REGISTER_HPP_
