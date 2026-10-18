#ifndef VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_NODE_HPP_
#define VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <string>

namespace vortex_bt_nodes::actuators {

/**
 * @brief Publishes the launch once on `topic` (0 left, 1 right), then RUNNING
 * for `settle_s` and SUCCESS.
 *
 * FAILURE on an unknown side or a negative settle_s. Halting does not undo a
 * launch that already went out.
 */
class FireTorpedo : public BT::StatefulActionNode {
   public:
    FireTorpedo(const std::string& name,
                const BT::NodeConfig& config,
                const rclcpp::Node::SharedPtr& node);

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    uint8_t side_{0};  // 0 left, 1 right
    std::string topic_;
    bool torpedo_launched_{false};

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fire_torpedo_pub_;

    rclcpp::Time end_time_;
};

}  // namespace vortex_bt_nodes::actuators
#endif  // !VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_NODE_HPP_
