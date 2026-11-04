#ifndef VORTEX_BT_NODES__MAP__MEASURE_HPP_
#define VORTEX_BT_NODES__MAP__MEASURE_HPP_

#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/measure_landmark.hpp>

namespace vortex_bt_nodes::map {

/**
 * @brief Calls landmark_server/measure. SUCCESS when measured, or when the
 * fallback was copied and accept_fallback is true.
 */
class Measure : public BT::StatefulActionNode {
   public:
    using Action = vortex_msgs::action::MeasureLandmark;
    using Handle = rclcpp_action::ClientGoalHandle<Action>;

    Measure(const std::string& name,
            const BT::NodeConfig& config,
            rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<Action>::SharedPtr client_;
    Handle::SharedPtr handle_;
    std::optional<Handle::WrappedResult> result_;
    bool rejected_{false};
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__MEASURE_HPP_
