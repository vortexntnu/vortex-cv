#ifndef VORTEX_BT_NODES__MISSION__WAIT_FOR_START_HPP_
#define VORTEX_BT_NODES__MISSION__WAIT_FOR_START_HPP_

#include <behaviortree_cpp/action_node.h>

#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/srv/get_operation_mode.hpp>

namespace vortex_bt_nodes::mission {

/**
 * @brief RUNNING until the killswitch is off and the vehicle is in
 * autonomous mode (asks get_operation_mode twice a second: the mode is only
 * published when it changes).
 */
class WaitForStart : public BT::StatefulActionNode {
   public:
    WaitForStart(const std::string& name,
                 const BT::NodeConfig& config,
                 rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node)) {}

    static BT::PortsList providedPorts();

   private:
    using Srv = vortex_msgs::srv::GetOperationMode;

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<Srv>::SharedPtr client_;
    std::optional<rclcpp::Client<Srv>::FutureAndRequestId> pending_;
    rclcpp::Time last_ask_;
    bool logged_{false};
};

}  // namespace vortex_bt_nodes::mission

#endif  // VORTEX_BT_NODES__MISSION__WAIT_FOR_START_HPP_
