#include <gtest/gtest.h>

#include "bt_test_utils.hpp"
#include "vortex_bt_nodes/common/nav_action.hpp"

using vortex_bt_nodes::NavAction;
using vortex_bt_nodes::test::BtNodeTest;
using vortex_bt_nodes::test::FakeWaypointManager;

namespace {

// A minimal NavAction: go to the depth on its port.
class GoToDepth : public NavAction {
   public:
    using NavAction::NavAction;
    static BT::PortsList providedPorts() {
        return providedBasicPorts({BT::InputPort<double>("depth")});
    }

   protected:
    std::optional<Goal> make_goal() override {
        const auto depth = getInput<double>("depth");
        if (!depth) {
            return std::nullopt;
        }
        Goal goal;
        vortex_msgs::msg::Waypoint wp;
        wp.pose.position.z = *depth;
        wp.waypoint_mode.mode = vortex_msgs::msg::WaypointMode::ONLY_Z;
        goal.waypoints.push_back(wp);
        goal.convergence_threshold = 0.1;
        return goal;
    }
};

class NavActionTest : public BtNodeTest {
   protected:
    void SetUp() override {
        BtNodeTest::SetUp();
        factory_.registerNodeType<GoToDepth>("GoToDepth", client_node_);
    }
};

}  // namespace

TEST_F(NavActionTest, SucceedsAndSendsTheGoal) {
    start_server();
    auto tree = make_tree(R"(<GoToDepth depth="1.5"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server_->last_goal.waypoints.size(), 1u);
    EXPECT_DOUBLE_EQ(server_->last_goal.waypoints[0].pose.position.z, 1.5);
    EXPECT_EQ(server_->last_goal.waypoints[0].waypoint_mode.mode,
              vortex_msgs::msg::WaypointMode::ONLY_Z);
}

TEST_F(NavActionTest, ReadsTheBlackboard) {
    start_server();
    blackboard()->set("slalom_depth", 2.25);
    auto tree = make_tree(R"(<GoToDepth depth="{slalom_depth}"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(server_->last_goal.waypoints[0].pose.position.z, 2.25);
}

TEST_F(NavActionTest, FailsWhenAborted) {
    start_server();
    server_->behaviour = FakeWaypointManager::Behaviour::kAbort;
    auto tree = make_tree(R"(<GoToDepth depth="1.5"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
}

TEST_F(NavActionTest, FailsWhenRejected) {
    start_server();
    server_->behaviour = FakeWaypointManager::Behaviour::kReject;
    auto tree = make_tree(R"(<GoToDepth depth="1.5"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
}

TEST_F(NavActionTest, FailsWithoutGoal) {
    start_server();
    auto tree = make_tree(R"(<GoToDepth/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
    EXPECT_EQ(server_->goals_received, 0);
}

TEST_F(NavActionTest, FailsWithoutServer) {
    auto tree = make_tree(R"(<GoToDepth depth="1.5"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
}

TEST_F(NavActionTest, AppliesTolerances) {
    start_server();
    auto tree = make_tree(R"(<GoToDepth depth="1.5" position_tolerance="0.3" )"
                          R"(orientation_tolerance_deg="90" hold_s="2.0"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::SUCCESS);
    ASSERT_EQ(server_->last_goal.waypoints.size(), 1u);
    const auto& wp = server_->last_goal.waypoints[0];
    EXPECT_DOUBLE_EQ(wp.position_tolerance, 0.3);
    EXPECT_NEAR(wp.orientation_tolerance, M_PI / 2.0, 1e-9);
    EXPECT_DOUBLE_EQ(wp.hold_time_sec, 2.0);
}

TEST_F(NavActionTest, TimeoutInXmlCancels) {
    start_server();
    server_->behaviour = FakeWaypointManager::Behaviour::kNeverFinish;
    auto tree =
        make_tree(R"(<Timeout msec="500"><GoToDepth depth="1.5"/></Timeout>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
    run(tree, 0.3);  // let the cancel request arrive
    EXPECT_TRUE(server_->cancel_requested);
}

TEST_F(NavActionTest, HaltCancelsTheGoal) {
    start_server();
    server_->behaviour = FakeWaypointManager::Behaviour::kNeverFinish;
    auto tree = make_tree(R"(<GoToDepth depth="1.5"/>)");
    EXPECT_EQ(run(tree, 0.5), BT::NodeStatus::RUNNING);
    tree.haltTree();
    for (int i = 0; i < 30 && !server_->cancel_requested; ++i) {
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(server_->cancel_requested);
}
