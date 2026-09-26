#ifndef VORTEX_BT_NODES__COMMON__NAV_ACTION_HPP_
#define VORTEX_BT_NODES__COMMON__NAV_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vortex_msgs/action/waypoint_manager.hpp>

namespace vortex_bt_nodes {

/**
 * @brief Base for every node that moves the vehicle through waypoint_manager.
 *
 * A derived node only builds the goal (make_goal). NavAction sends it,
 * returns RUNNING until the result arrives, and cancels the goal when the
 * tree halts the node. It never blocks a tick: the runner spins the ROS node
 * between ticks, which runs the action callbacks.
 *
 * Returns SUCCESS when waypoint_manager reports success, FAILURE when
 * make_goal gives nothing, the server is missing, or the goal is rejected or
 * aborted. Timeouts are written in the XML (Timeout halts the node, which
 * cancels the goal).
 *
 * Every NavAction has the ports position_tolerance [m],
 * orientation_tolerance_deg and hold_s. When set (> 0) they fill in every
 * waypoint whose own value is 0.
 *
 * Example:
 * @code
 * class SetDepth : public NavAction {
 *   public:
 *     using NavAction::NavAction;
 *     static BT::PortsList providedPorts() {
 *         return providedBasicPorts({BT::InputPort<double>("depth")});
 *     }
 *   protected:
 *     std::optional<Goal> make_goal() override { ... }
 * };
 * // register: factory.registerNodeType<SetDepth>("SetDepth", node);
 * @endcode
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

    /** @brief The shared ports plus the node's own. */
    static BT::PortsList providedBasicPorts(BT::PortsList addition);
    static BT::PortsList providedPorts() { return providedBasicPorts({}); }

   protected:
    /**
     * @brief Build the goal from the ports and blackboard. Return
     * std::nullopt (after logging why) when the input is missing or bad;
     * the node then fails.
     */
    virtual std::optional<Goal> make_goal() = 0;

    /** @brief Called on every feedback message. Default: nothing. */
    virtual void on_feedback(const Action::Feedback& /*feedback*/) {}

    /** @brief Turn the result into SUCCESS or FAILURE. */
    virtual BT::NodeStatus on_result(const GoalHandle::WrappedResult& result);

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
    // Identifies the current goal, so callbacks of an old goal are ignored.
    std::uint64_t goal_seq_{0};
    bool sent_{false};
    bool rejected_{false};
    GoalHandle::SharedPtr goal_handle_;
    std::optional<GoalHandle::WrappedResult> result_;
    rclcpp::Time start_time_;
};

}  // namespace vortex_bt_nodes

#endif  // VORTEX_BT_NODES__COMMON__NAV_ACTION_HPP_
