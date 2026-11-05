
#ifndef VORTEX_BT_NODES__MISSION__WAIT_FOR_START_HPP_
#define VORTEX_BT_NODES__MISSION__WAIT_FOR_START_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <vortex_msgs/msg/operation_mode.hpp>
#include <vortex_msgs/srv/get_operation_mode.hpp>

namespace vortex_bt_nodes::mission {

class WaitForStart : public BT::StatefulActionNode {
   public:
    WaitForStart(const std::string& name,
                 const BT::NodeConfig& config,
                 rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

   private:
    using Service = vortex_msgs::srv::GetOperationMode;
    using Client = rclcpp::Client<Service>;
    using Clock = std::chrono::steady_clock;

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    rclcpp::Node::SharedPtr node_;
    Client::SharedPtr client_;
    std::optional<Client::FutureAndRequestId> future_;

    Clock::time_point next_request_{};

    std::chrono::milliseconds request_interval_{500};
};

}  // namespace vortex_bt_nodes::mission

#endif  // VORTEX_BT_NODES__MISSION__WAIT_FOR_START_HPP_
