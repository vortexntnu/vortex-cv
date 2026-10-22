#ifndef VORTEX_BT_NODES__MISSION__LOG_HPP_
#define VORTEX_BT_NODES__MISSION__LOG_HPP_

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace vortex_bt_nodes::mission {

/** @brief Logs a message at the requested level and returns SUCCESS. */
class Log : public BT::SyncActionNode {
   public:
    Log(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus tick() override;
};

}  // namespace vortex_bt_nodes::mission

#endif  // VORTEX_BT_NODES__MISSION__LOG_HPP_