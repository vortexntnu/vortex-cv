#ifndef VORTEX_BT_NODES__MISSION__LOG_HPP_
#define VORTEX_BT_NODES__MISSION__LOG_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>

namespace vortex_bt_nodes::mission {

/** @brief Logs message (level info, warn or error) and returns SUCCESS. */
class Log : public BT::SyncActionNode {
   public:
    using BT::SyncActionNode::SyncActionNode;
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

}  // namespace vortex_bt_nodes::mission

#endif  // VORTEX_BT_NODES__MISSION__LOG_HPP_
