#ifndef BT_TEST_UTILS_HPP_
#define BT_TEST_UTILS_HPP_

// Helpers for testing one node alone: a fake waypoint_manager and a fixture
// that builds a tree from an XML string and ticks it like the runner does.

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>
#include <vortex_msgs/action/waypoint_manager.hpp>

#include "vortex_bt_nodes/common/types.hpp"

namespace vortex_bt_nodes::test {

using Action = vortex_msgs::action::WaypointManager;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Action>;

// Stands in for waypoint_manager.
class FakeWaypointManager {
   public:
    enum class Behaviour { kSucceed, kAbort, kReject, kNeverFinish };

    explicit FakeWaypointManager(const rclcpp::Node::SharedPtr& node) {
        server_ = rclcpp_action::create_server<Action>(
            node, "waypoint_manager",
            [this](const rclcpp_action::GoalUUID&,
                   std::shared_ptr<const Action::Goal> goal) {
                last_goal = *goal;
                ++goals_received;
                return behaviour == Behaviour::kReject
                           ? rclcpp_action::GoalResponse::REJECT
                           : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<ServerGoalHandle>) {
                cancel_requested = true;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<ServerGoalHandle> handle) {
                active_ = handle;
            });
    }

    // Finish the active goal the way the behaviour says.
    void step() {
        if (!active_ || !active_->is_active()) {
            return;
        }
        auto result = std::make_shared<Action::Result>();
        if (active_->is_canceling()) {
            active_->canceled(result);
        } else if (behaviour == Behaviour::kSucceed) {
            result->success = true;
            active_->succeed(result);
        } else if (behaviour == Behaviour::kAbort) {
            result->message = "aborted by the fake";
            active_->abort(result);
        }
    }

    Behaviour behaviour{Behaviour::kSucceed};
    Action::Goal last_goal;
    int goals_received{0};
    bool cancel_requested{false};

   private:
    rclcpp_action::Server<Action>::SharedPtr server_;
    std::shared_ptr<ServerGoalHandle> active_;
};

/**
 * Register the node under test in SetUp (after BtNodeTest::SetUp), with
 * client_node_ as its ROS node. start_server() adds the fake
 * waypoint_manager; blackboard() fills keys before run().
 */
class BtNodeTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void SetUp() override {
        client_node_ = rclcpp::Node::make_shared("bt_client");
        server_node_ = rclcpp::Node::make_shared("fake_waypoint_manager");
        executor_.add_node(client_node_);
        executor_.add_node(server_node_);
    }

    BT::Blackboard::Ptr blackboard() { return tree_blackboard_; }

    void start_server() {
        server_ = std::make_unique<FakeWaypointManager>(server_node_);
    }

    // node_xml is the body of one BehaviorTree; the tree shares
    // blackboard(), so keys set there are visible to the nodes.
    BT::Tree make_tree(const std::string& node_xml) {
        factory_.registerBehaviorTreeFromText(
            R"(<root BTCPP_format="4"><BehaviorTree ID="Main">)" + node_xml +
            "</BehaviorTree></root>");
        return factory_.createTree("Main", tree_blackboard_);
    }

    // Tick like the runner does: spin, then tick, until the tree is done.
    BT::NodeStatus run(BT::Tree& tree, double max_s = 5.0) {
        const auto end = std::chrono::steady_clock::now() +
                         std::chrono::duration<double>(max_s);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        while (status == BT::NodeStatus::RUNNING &&
               std::chrono::steady_clock::now() < end) {
            executor_.spin_some();
            if (server_) {
                server_->step();
            }
            status = tree.tickOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return status;
    }

    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Node::SharedPtr server_node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<FakeWaypointManager> server_;
    BT::Blackboard::Ptr tree_blackboard_ = BT::Blackboard::create();
};

}  // namespace vortex_bt_nodes::test

#endif  // BT_TEST_UTILS_HPP_
