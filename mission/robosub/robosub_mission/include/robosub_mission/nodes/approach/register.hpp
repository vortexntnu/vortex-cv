#ifndef ROBOSUB_MISSION__NODES__APPROACH__REGISTER_HPP_
#define ROBOSUB_MISSION__NODES__APPROACH__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace robosub_mission::nodes::approach {

/**
 * @brief Register the approach nodes (approaching landmarks with
 * landmark_targets).
 *
 * Planned: ApproachLandmark, CommitEstimate, LookAtLandmark.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace robosub_mission::nodes::approach

#endif  // ROBOSUB_MISSION__NODES__APPROACH__REGISTER_HPP_
