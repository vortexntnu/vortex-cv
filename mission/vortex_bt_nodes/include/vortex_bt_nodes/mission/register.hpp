#ifndef VORTEX_BT_NODES__MISSION__REGISTER_HPP_
#define VORTEX_BT_NODES__MISSION__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace vortex_bt_nodes::mission {

/**
 * @brief Register the mission nodes (mission flow and safety).
 *
 * Planned: LoadMissionConfig, Wait, SetOperationMode,
 * ResetWorld, LogError, SavePose, GoToSavedPose, StartRun, MissionClock,
 * TaskSlot, RecordBag, ResolveRole, VerifyInside.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace vortex_bt_nodes::mission

#endif  // VORTEX_BT_NODES__MISSION__REGISTER_HPP_
