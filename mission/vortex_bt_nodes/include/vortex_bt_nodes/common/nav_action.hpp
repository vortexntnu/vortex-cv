#ifndef VORTEX_BT_NODES__COMMON__NAV_ACTION_HPP_
#define VORTEX_BT_NODES__COMMON__NAV_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/waypoint.hpp>

#include "vortex_bt_nodes/common/types.hpp"

namespace vortex_bt_nodes {

/**
 * @brief Base for nodes that move the vehicle through waypoint_manager.
 *
 * A derived node only builds the goal in make_goal(). NavAction sends it,
 * returns RUNNING until the result arrives and cancels it when halted.
 * The ports position_tolerance, orientation_tolerance_deg and hold_s fill
 * in every waypoint that leaves them at 0.
 */
class NavAction : public BT::StatefulActionNode {
   public:
    using Action = vortex_msgs::action::WaypointManager;
    using Goal = Action::Goal;
    using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

    NavAction(const std::string& name,
              const BT::NodeConfig& config,
              rclcpp::Node::SharedPtr node,
              const std::string& action_name = "waypoint_manager");

    static BT::PortsList providedBasicPorts(BT::PortsList addition);
    static BT::PortsList providedPorts() { return providedBasicPorts({}); }

   protected:
    /** @brief Returning nullopt fails the node. */
    virtual std::optional<Goal> make_goal() = 0;

    /** @brief Called every tick. Return a goal to replace the running one. */
    virtual std::optional<Goal> update_goal() { return std::nullopt; }

    virtual void on_feedback(const Action::Feedback& /*feedback*/) {}

    virtual BT::NodeStatus on_result(const GoalHandle::WrappedResult& result);

    static vortex_msgs::msg::Waypoint make_waypoint(
        const geometry_msgs::msg::Pose& pose,
        std::uint8_t mode) {
        vortex_msgs::msg::Waypoint wp;
        wp.pose = pose;
        wp.waypoint_mode.mode = mode;
        return wp;
    }

    rclcpp::Node::SharedPtr node_;

   private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void apply_tolerances(Goal& goal);
    void send_goal();
    void cancel_goal();

    rclcpp_action::Client<Action>::SharedPtr client_;
    std::optional<Goal> goal_;
    // Callbacks of older goals are ignored.
    std::uint64_t goal_seq_{0};
    bool sent_{false};
    bool rejected_{false};
    GoalHandle::SharedPtr goal_handle_;
    std::optional<GoalHandle::WrappedResult> result_;
    rclcpp::Time start_time_;
};

}  // namespace vortex_bt_nodes

#endif  // VORTEX_BT_NODES__COMMON__NAV_ACTION_HPP_
