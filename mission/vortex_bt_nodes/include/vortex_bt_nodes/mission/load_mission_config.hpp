#ifndef VORTEX_BT_NODES__MISSION__LOAD_MISSION_CONFIG_HPP_
#define VORTEX_BT_NODES__MISSION__LOAD_MISSION_CONFIG_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>

namespace vortex_bt_nodes::mission {

/**
 * @brief Writes every key of a yaml file to the blackboard as text. Nested
 * keys become a.b.
 */
class LoadMissionConfig : public BT::SyncActionNode {
   public:
    using BT::SyncActionNode::SyncActionNode;
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

}  // namespace vortex_bt_nodes::mission

#endif  // VORTEX_BT_NODES__MISSION__LOAD_MISSION_CONFIG_HPP_
