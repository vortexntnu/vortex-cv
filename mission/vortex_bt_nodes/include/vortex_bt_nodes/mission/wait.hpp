#ifndef VORTEX_BT_NODES__MISSION__WAIT_HPP_
#define VORTEX_BT_NODES__MISSION__WAIT_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace vortex_bt_nodes::mission {

/** @brief RUNNING for a number of seconds, then SUCCESS. */
class Wait : public BT::StatefulActionNode {
   public:
    Wait(const std::string& name,
         const BT::NodeConfig& config,
         rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Time end_time_;
};

}  // namespace vortex_bt_nodes::mission

#endif  // VORTEX_BT_NODES__MISSION__WAIT_HPP_
