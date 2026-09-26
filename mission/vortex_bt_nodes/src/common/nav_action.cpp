#include "vortex_bt_nodes/common/nav_action.hpp"

#include <spdlog/spdlog.h>
#include <cmath>

namespace vortex_bt_nodes {

namespace {
// How long to wait for waypoint_manager to show up before failing.
constexpr double kServerWaitS = 2.0;
}  // namespace

NavAction::NavAction(const std::string& name,
                     const BT::NodeConfig& config,
                     rclcpp::Node::SharedPtr node,
                     const std::string& action_name)
    : BT::StatefulActionNode(name, config), node_(std::move(node)) {
    client_ = rclcpp_action::create_client<Action>(node_, action_name);
}

BT::PortsList NavAction::providedBasicPorts(BT::PortsList addition) {
    BT::PortsList ports = {
        BT::InputPort<double>("position_tolerance", 0.0, "[m], 0 = default"),
        BT::InputPort<double>("orientation_tolerance_deg", 0.0,
                              "[deg], 0 = default"),
        BT::InputPort<double>("hold_s", 0.0,
                              "Time inside the tolerances before success")};
    ports.insert(addition.begin(), addition.end());
    return ports;
}

BT::NodeStatus NavAction::onStart() {
    ++goal_seq_;
    sent_ = false;
    rejected_ = false;
    goal_handle_.reset();
    result_.reset();
    start_time_ = node_->now();

    goal_ = make_goal();
    if (!goal_) {
        spdlog::warn("[{}] no goal, failing", name());
        return BT::NodeStatus::FAILURE;
    }
    apply_tolerances(*goal_);
    return onRunning();
}

BT::NodeStatus NavAction::onRunning() {
    const double elapsed = (node_->now() - start_time_).seconds();

    if (!sent_) {
        if (client_->action_server_is_ready()) {
            send_goal();
            return BT::NodeStatus::RUNNING;
        }
        if (elapsed > kServerWaitS) {
            spdlog::error("[{}] waypoint_manager is not available", name());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    if (rejected_) {
        spdlog::warn("[{}] goal rejected by waypoint_manager", name());
        return BT::NodeStatus::FAILURE;
    }
    if (result_) {
        return on_result(*result_);
    }
    return BT::NodeStatus::RUNNING;
}

void NavAction::onHalted() {
    cancel_goal();
}

BT::NodeStatus NavAction::on_result(const GoalHandle::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
        result.result->success) {
        return BT::NodeStatus::SUCCESS;
    }
    spdlog::warn("[{}] waypoint_manager did not succeed: {}", name(),
                 result.result ? result.result->message : "no result");
    return BT::NodeStatus::FAILURE;
}

void NavAction::apply_tolerances(Goal& goal) {
    const double pos_tol = getInput<double>("position_tolerance").value_or(0.0);
    const double ori_tol_deg =
        getInput<double>("orientation_tolerance_deg").value_or(0.0);
    const double hold_s = getInput<double>("hold_s").value_or(0.0);
    for (auto& wp : goal.waypoints) {
        if (wp.position_tolerance == 0.0) {
            wp.position_tolerance = pos_tol;
        }
        if (wp.orientation_tolerance == 0.0) {
            wp.orientation_tolerance = ori_tol_deg * M_PI / 180.0;
        }
        if (wp.hold_time_sec == 0.0) {
            wp.hold_time_sec = hold_s;
        }
    }
}

void NavAction::send_goal() {
    sent_ = true;
    const std::uint64_t seq = goal_seq_;

    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this, seq](GoalHandle::SharedPtr handle) {
        if (seq != goal_seq_) {
            // The node was halted or restarted before the goal was accepted.
            if (handle) {
                client_->async_cancel_goal(handle);
            }
            return;
        }
        if (!handle) {
            rejected_ = true;
            return;
        }
        goal_handle_ = handle;
    };
    options.feedback_callback =
        [this, seq](GoalHandle::SharedPtr,
                    const std::shared_ptr<const Action::Feedback> feedback) {
            if (seq == goal_seq_) {
                on_feedback(*feedback);
            }
        };
    options.result_callback = [this,
                               seq](const GoalHandle::WrappedResult& result) {
        if (seq == goal_seq_) {
            result_ = result;
        }
    };
    client_->async_send_goal(*goal_, options);
}

void NavAction::cancel_goal() {
    // A new sequence number makes the callbacks of this goal stale; a goal
    // that is still waiting for acceptance is cancelled when it arrives.
    ++goal_seq_;
    if (goal_handle_ && !result_) {
        client_->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
}

}  // namespace vortex_bt_nodes
