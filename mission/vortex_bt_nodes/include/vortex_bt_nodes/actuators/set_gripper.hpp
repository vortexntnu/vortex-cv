#ifndef VORTEX_BT_NODES__ACTUATORS__SET_GRIPPER_HPP_
#define VORTEX_BT_NODES__ACTUATORS__SET_GRIPPER_HPP_

#include <behaviortree_cpp/action_node.h>

#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/gripper_reference_filter_waypoint.hpp>

namespace vortex_bt_nodes::actuators {

/** @brief Sends roll and pinch to the gripper action. */
class SetGripper : public BT::StatefulActionNode {
   public:
    using Action = vortex_msgs::action::GripperReferenceFilterWaypoint;
    using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

    SetGripper(const std::string& name,
               const BT::NodeConfig& config,
               rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node)) {}

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<Action>::SharedPtr client_;
    GoalHandle::SharedPtr handle_;
    std::optional<bool> result_;
    bool rejected_{false};
    rclcpp::Time start_;
};

}  // namespace vortex_bt_nodes::actuators

#endif  // VORTEX_BT_NODES__ACTUATORS__SET_GRIPPER_HPP_
