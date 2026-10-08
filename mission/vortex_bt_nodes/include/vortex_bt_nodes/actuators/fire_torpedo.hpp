#ifndef VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_HPP_
#define VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

namespace vortex_bt_nodes::actuators {

/**
 * @brief Fires the left (0) or right (1) torpedo: std_msgs/Int8 on topic,
 * then waits settle_s. FAILURE on an unknown side. The topic is a placeholder
 * until the drone has an interface.
 */
class FireTorpedo : public BT::StatefulActionNode {
   public:
    FireTorpedo(const std::string& name,
                const BT::NodeConfig& config,
                rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node)) {}

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_;
    std::string topic_;
    rclcpp::Time start_;
};

}  // namespace vortex_bt_nodes::actuators

#endif  // VORTEX_BT_NODES__ACTUATORS__FIRE_TORPEDO_HPP_
