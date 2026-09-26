#ifndef ROBOSUB_MISSION__NODES__MISSION__REGISTER_HPP_
#define ROBOSUB_MISSION__NODES__MISSION__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace robosub_mission::nodes::mission {

/**
 * @brief Register the mission nodes (mission flow and safety).
 *
 * Planned: VehicleHealthy, LoadMissionConfig, Wait, SetOperationMode,
 * ResetWorld, LogError, SavePose, GoToSavedPose, StartRun, MissionClock,
 * TaskSlot, RecordBag, ResolveRole, VerifyInside.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace robosub_mission::nodes::mission

#endif  // ROBOSUB_MISSION__NODES__MISSION__REGISTER_HPP_
