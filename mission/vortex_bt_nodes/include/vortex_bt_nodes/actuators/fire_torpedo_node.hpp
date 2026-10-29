#ifndef VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_NODE_HPP_
#define VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <future>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vortex_msgs/srv/fire_torpedo.hpp>

namespace vortex_bt_nodes::actuators {

/**
 * @brief Calls the FireTorpedo service once, then RUNNING until the answer.
 *
 * FAILURE on undefined side.
 */

class FireTorpedo : public BT::StatefulActionNode {
   public:
    using Srv = vortex_msgs::srv::FireTorpedo;
    FireTorpedo(const std::string& name,
                const BT::NodeConfig& config,
                const rclcpp::Node::SharedPtr& node);

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    uint8_t side_{0};  // 0 left, 1 right
    bool torpedo_request_sent_{false};
    bool torpedo_launched_{false};

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<Srv>::SharedPtr fire_torpedo_cli_;
    std::shared_future<Srv::Response::SharedPtr> future_;
    std::optional<int64_t> req_ID_;
};

}  // namespace vortex_bt_nodes::actuators
#endif  // VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_NODE_HPP_
