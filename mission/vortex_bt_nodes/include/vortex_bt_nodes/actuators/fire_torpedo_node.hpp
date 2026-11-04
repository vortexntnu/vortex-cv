#ifndef VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_NODE_HPP_
#define VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

namespace vortex_bt_nodes::actuators {

/**
 * @brief Calls the `service` trigger, then RUNNING for `settle_s` and SUCCESS.
 *
 * FAILURE if the service is not there after `timeout_s`, or says no. Halting
 * does not undo a shot that already went out.
 */
class FireTorpedo : public BT::StatefulActionNode {
   public:
    FireTorpedo(const std::string& name,
                const BT::NodeConfig& config,
                const rclcpp::Node::SharedPtr& node);

    static BT::PortsList providedPorts();

   private:
    using Srv = std_srvs::srv::Trigger;

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<Srv>::SharedPtr client_;
    std::optional<rclcpp::Client<Srv>::SharedFutureAndRequestId> pending_;
    double settle_s_{0.0};
    rclcpp::Time give_up_at_;
    std::optional<rclcpp::Time> end_time_;
};

}  // namespace vortex_bt_nodes::actuators
#endif  // !VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_NODE_HPP_
